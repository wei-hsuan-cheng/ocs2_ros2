/**
 * MobileManipulatorTwistTarget
 *
 * Publishes MpcTargetTrajectories by integrating a constant end‑effector twist
 * over a short horizon. Uses the latest MPC observation time as t0 and, if
 * available, computes the current end‑effector pose from Pinocchio based on
 * the latest state to start the integration from a consistent pose.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

using namespace ocs2;
using namespace ocs2::mobile_manipulator;

namespace {

struct Params {
  double publish_rate_hz = 20.0;
  double horizon_T = 1.0;
  double dt = 0.1;
  bool twist_in_world = true;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
};

inline Eigen::Quaterniond deltaQuatFromAngular(const Eigen::Vector3d& w, double dt) {
  const double wn = w.norm();
  if (wn < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  const double half_theta = 0.5 * wn * dt;
  const double s = std::sin(half_theta);
  const double c = std::cos(half_theta);
  const Eigen::Vector3d u = w / wn;
  return Eigen::Quaterniond(c, u.x() * s, u.y() * s, u.z() * s);
}

} // namespace

class TwistTargetNode : public rclcpp::Node {
 public:
  explicit TwistTargetNode(const rclcpp::NodeOptions& options)
      : Node("mobile_manipulator_twist_target", options) {
    // Allow reading externally provided parameters
    this->declare_parameter<std::string>("taskFile", "");
    this->declare_parameter<std::string>("urdfFile", "");
    this->declare_parameter<std::string>("libFolder", "");
    this->declare_parameter<std::string>("robotName", std::string("mobile_manipulator"));

    this->declare_parameter<double>("publishRate", params_.publish_rate_hz);
    this->declare_parameter<double>("horizon", params_.horizon_T);
    this->declare_parameter<double>("dt", params_.dt);
    this->declare_parameter<bool>("twistInWorld", params_.twist_in_world);
    this->declare_parameter<double>("vx", 0.0);
    this->declare_parameter<double>("vy", 0.0);
    this->declare_parameter<double>("vz", 0.0);
    this->declare_parameter<double>("wx", 0.0);
    this->declare_parameter<double>("wy", 0.0);
    this->declare_parameter<double>("wz", 0.0);

    // Read parameters
    taskFile_ = this->get_parameter("taskFile").as_string();
    urdfFile_ = this->get_parameter("urdfFile").as_string();
    libFolder_ = this->get_parameter("libFolder").as_string();
    robotName_ = this->get_parameter("robotName").as_string();

    params_.publish_rate_hz = this->get_parameter("publishRate").as_double();
    params_.horizon_T = this->get_parameter("horizon").as_double();
    params_.dt = this->get_parameter("dt").as_double();
    params_.twist_in_world = this->get_parameter("twistInWorld").as_bool();
    params_.v << this->get_parameter("vx").as_double(), this->get_parameter("vy").as_double(),
        this->get_parameter("vz").as_double();
    params_.w << this->get_parameter("wx").as_double(), this->get_parameter("wy").as_double(),
        this->get_parameter("wz").as_double();

    // Optional: interface for FK initialization
    if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
      try {
        interface_ = std::make_unique<MobileManipulatorInterface>(taskFile_, libFolder_, urdfFile_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
      }
    }

    // Publisher for target trajectories
    targetPub_ = this->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
        robotName_ + std::string("_mpc_target"), 1);

    // Observation subscriber (for t0 and optional FK starting pose)
    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + std::string("_mpc_observation"), 1,
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(obsMtx_);
          latestObs_ = ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    // Timer to publish at rate
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, params_.publish_rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&TwistTargetNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Twist target node started. Publishing to %s_mpc_target at %.1f Hz",
                robotName_.c_str(), params_.publish_rate_hz);
  }

 private:
  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(obsMtx_);
      if (!haveObs_) return;
      obs = latestObs_;
    }

    // Determine initial EE pose (p, q)
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    if (interface_) {
      try {
        const auto& pin = interface_->getPinocchioInterface();
        const auto& model = pin.getModel();
        auto data = pin.getData();
        pinocchio::forwardKinematics(model, data, obs.state);
        pinocchio::updateFramePlacements(model, data);
        const auto& info = interface_->getManipulatorModelInfo();
        const auto ee_id = model.getFrameId(info.eeFrame);
        const auto& ee = data.oMf[ee_id];
        p = ee.translation();
        q = Eigen::Quaterniond(ee.rotation());
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "FK failed: %s", e.what());
      }
    }

    // Build horizon by integrating twist
    const double t0 = obs.time;
    const int N = std::max(1, static_cast<int>(std::ceil(params_.horizon_T / std::max(1e-6, params_.dt))));

    scalar_array_t timeTraj;
    timeTraj.reserve(N + 1);
    vector_array_t stateTraj;
    stateTraj.reserve(N + 1);
    vector_array_t inputTraj;
    inputTraj.reserve(N + 1);

    const auto& info = interface_ ? interface_->getManipulatorModelInfo() : ManipulatorModelInfo();
    const size_t inputDim = obs.input.size() > 0 ? obs.input.size() : info.inputDim;
    const vector_t zeroInput = (inputDim > 0) ? vector_t::Zero(inputDim) : vector_t();

    Eigen::Vector3d p_k = p;
    Eigen::Quaterniond q_k = q.normalized();

    for (int k = 0; k <= N; ++k) {
      const double tk = t0 + k * params_.dt;
      timeTraj.push_back(tk);

      // pack [p, q]
      vector_t target(7);
      target << p_k, Eigen::Vector4d(q_k.x(), q_k.y(), q_k.z(), q_k.w());
      stateTraj.push_back(std::move(target));
      inputTraj.push_back(zeroInput);

      if (k == N) break; // no need to integrate after last sample

      // integrate one step
      if (params_.twist_in_world) {
        p_k += params_.v * params_.dt;
        const Eigen::Quaterniond dq = deltaQuatFromAngular(params_.w, params_.dt);
        q_k = (dq * q_k).normalized();
      } else {
        // twist given in EE frame: rotate linear velocity to world frame first
        p_k += (q_k * params_.v) * params_.dt;
        const Eigen::Quaterniond dq = deltaQuatFromAngular(params_.w, params_.dt);
        q_k = (q_k * dq).normalized();
      }
    }

    TargetTrajectories traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
    const auto msg = ros_msg_conversions::createTargetTrajectoriesMsg(traj);
    targetPub_->publish(msg);
  }

  // Params and config
  Params params_{};
  std::string taskFile_, urdfFile_, libFolder_, robotName_;

  // Optional interface for FK
  std::unique_ptr<MobileManipulatorInterface> interface_;

  // ROS entities
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetPub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest observation
  std::mutex obsMtx_;
  SystemObservation latestObs_;
  bool haveObs_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  // We explicitly declare parameters in the node; avoid double declaration
  // when launch files pass parameters via overrides.
  opts.automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<TwistTargetNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

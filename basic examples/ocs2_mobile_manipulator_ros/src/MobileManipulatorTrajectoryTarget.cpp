/**
 * MobileManipulatorTrajectoryTarget
 *
 * Publishes a time-parameterized TargetTrajectories following an "eight" shape
 * in XY, centered at the initial end-effector position. Orientation remains
 * constant (initial FK orientation). Uses the latest MPC observation time as
 * anchor and advances the phase relative to the first observation time.
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
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

using namespace ocs2;
using namespace ocs2::mobile_manipulator;

namespace {
struct Params {
  double publish_rate_hz = 20.0;
  double horizon_T = 2.0;
  double dt = 0.05;
  double amplitude = 0.2;   // meters
  double frequency_hz = 0.1; // cycles per second
};

inline void orthonormalBasisFromAxis(const Eigen::Vector3d& axis_unit,
                                     Eigen::Vector3d& u, Eigen::Vector3d& v) {
  // Choose a reference not parallel to axis
  Eigen::Vector3d ref = (std::abs(axis_unit.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  u = axis_unit.cross(ref);
  double n = u.norm();
  if (n < 1e-9) {
    // Fallback if degenerate
    ref = Eigen::Vector3d::UnitY();
    u = axis_unit.cross(ref);
    n = u.norm();
  }
  u /= n;
  v = axis_unit.cross(u);
}

inline void eightShapeTrajectory(const Eigen::Vector3d& center_p, const Eigen::Quaterniond& q0,
                        const Eigen::Vector3d& plane_axis_unit,
                        double start_time, double t0, int N, double dt, double amp, double freq_hz,
                        scalar_array_t& timeTraj, vector_array_t& stateTraj) {
  const double omega = 2.0 * M_PI * freq_hz;
  Eigen::Vector3d u, v;  // in-plane unit basis
  orthonormalBasisFromAxis(plane_axis_unit, u, v);
  timeTraj.reserve(N + 1);
  stateTraj.reserve(N + 1);
  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * dt;
    const double tau = tk - start_time; // phase relative to trajectory start
    const double x = amp * std::sin(omega * tau);
    const double y = 0.5 * amp * std::sin(2.0 * omega * tau);
    Eigen::Vector3d p = center_p + u * x + v * y;

    vector_t target(7);
    target << p, Eigen::Vector4d(q0.x(), q0.y(), q0.z(), q0.w());

    timeTraj.push_back(tk);
    stateTraj.push_back(std::move(target));
  }
}
} // namespace

class TrajectoryTargetNode : public rclcpp::Node {
 public:
  explicit TrajectoryTargetNode(const rclcpp::NodeOptions& options)
      : Node("mobile_manipulator_trajectory_target", options) {
    // Parameters
    this->declare_parameter<std::string>("taskFile", "");
    this->declare_parameter<std::string>("urdfFile", "");
    this->declare_parameter<std::string>("libFolder", "");
    this->declare_parameter<std::string>("robotName", std::string("mobile_manipulator"));

    this->declare_parameter<double>("publishRate", params_.publish_rate_hz);
    this->declare_parameter<double>("horizon", params_.horizon_T);
    this->declare_parameter<double>("dt", params_.dt);
    this->declare_parameter<double>("amplitude", params_.amplitude);
    this->declare_parameter<double>("frequency", params_.frequency_hz);
    this->declare_parameter<double>("axisX", 0.0);
    this->declare_parameter<double>("axisY", 0.0);
    this->declare_parameter<double>("axisZ", 1.0);

    // Read
    taskFile_ = this->get_parameter("taskFile").as_string();
    urdfFile_ = this->get_parameter("urdfFile").as_string();
    libFolder_ = this->get_parameter("libFolder").as_string();
    robotName_ = this->get_parameter("robotName").as_string();

    params_.publish_rate_hz = this->get_parameter("publishRate").as_double();
    params_.horizon_T = this->get_parameter("horizon").as_double();
    params_.dt = this->get_parameter("dt").as_double();
    params_.amplitude = this->get_parameter("amplitude").as_double();
    params_.frequency_hz = this->get_parameter("frequency").as_double();
    axis_.x() = this->get_parameter("axisX").as_double();
    axis_.y() = this->get_parameter("axisY").as_double();
    axis_.z() = this->get_parameter("axisZ").as_double();
    if (axis_.norm() < 1e-9) axis_ = Eigen::Vector3d::UnitZ();
    axis_.normalize();

    // Optional interface for FK
    if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
      try {
        interface_ = std::make_unique<MobileManipulatorInterface>(taskFile_, libFolder_, urdfFile_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
      }
    }

    // Publishers
    targetPub_ = this->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
        robotName_ + std::string("_mpc_target"), 1);
    targetMarkerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mobile_manipulator/targetStateTrajectory", 1);
    targetPosePub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/mobile_manipulator/targetPoseTrajectory", 1);

    // Observation subscriber
    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + std::string("_mpc_observation"), 1,
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latestObs_ = ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, params_.publish_rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&TrajectoryTargetNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory target node started. Publishing to %s_mpc_target at %.1f Hz",
                robotName_.c_str(), params_.publish_rate_hz);
  }

 private:
  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!haveObs_) return;
      obs = latestObs_;
    }

    if (!initialized_) {
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
          center_p_ = ee.translation();
          q0_ = Eigen::Quaterniond(ee.rotation());
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "FK failed: %s", e.what());
          center_p_.setZero();
          q0_ = Eigen::Quaterniond::Identity();
        }
      }
      start_time_ = obs.time;
      initialized_ = true;
    }

    const int N = std::max(1, static_cast<int>(std::ceil(params_.horizon_T / std::max(1e-6, params_.dt))));

    // MPC TargetTrajectory
    // 1) Time trajectory
    scalar_array_t timeTraj;

    // 2) State trajectory
    vector_array_t stateTraj;
    eightShapeTrajectory(center_p_, q0_.normalized(), axis_, start_time_, obs.time, N, params_.dt,
                         params_.amplitude, params_.frequency_hz, timeTraj, stateTraj);
    
    // 3) Input trajectory
    // Zero inputs per sample
    vector_array_t inputTraj;
    inputTraj.reserve(timeTraj.size());
    const size_t inputDim = obs.input.size();
    const vector_t zeroInput = (inputDim > 0) ? vector_t::Zero(inputDim) : vector_t();
    for (size_t i = 0; i < timeTraj.size(); ++i) inputTraj.push_back(zeroInput);

    // Build and publish marker + pose array for the target path
    {
      std::vector<geometry_msgs::msg::Point> pts;
      pts.reserve(stateTraj.size());
      geometry_msgs::msg::PoseArray poseArray;
      poseArray.poses.reserve(stateTraj.size());

      for (const auto& s : stateTraj) {
        geometry_msgs::msg::Point p;
        p.x = s(0);
        p.y = s(1);
        p.z = s(2);
        pts.emplace_back(p);

        geometry_msgs::msg::Pose pose;
        pose.position = p;
        pose.orientation.x = s(3);
        pose.orientation.y = s(4);
        pose.orientation.z = s(5);
        pose.orientation.w = s(6);
        poseArray.poses.emplace_back(std::move(pose));
      }

      visualization_msgs::msg::MarkerArray ma;
      const scalar_t TRAJECTORYLINEWIDTH = 0.0025;
      const std::array<scalar_t, 3> targetTrajectoryColor{0.4660, 0.6740, 0.1880}; // green
      // const std::array<scalar_t, 3> targetTrajectoryColor{0.9290, 0.6940, 0.1250}; // orange
      auto marker = ros_msg_helpers::getLineMsg(std::move(pts), targetTrajectoryColor, TRAJECTORYLINEWIDTH);
      marker.header = ros_msg_helpers::getHeaderMsg("world", this->now());
      marker.ns = "Target Trajectory";
      marker.id = 0;
      ma.markers.emplace_back(std::move(marker));
      targetMarkerPub_->publish(ma);

      poseArray.header = ros_msg_helpers::getHeaderMsg("world", this->now());
      targetPosePub_->publish(poseArray);
    }

    TargetTrajectories traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
    const auto msg = ros_msg_conversions::createTargetTrajectoriesMsg(traj);
    targetPub_->publish(msg);
  }

  Params params_{};
  std::string taskFile_, urdfFile_, libFolder_, robotName_;

  std::unique_ptr<MobileManipulatorInterface> interface_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targetMarkerPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr targetPosePub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  SystemObservation latestObs_;
  bool haveObs_ = false;
  bool initialized_ = false;
  double start_time_ = 0.0;
  Eigen::Vector3d center_p_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q0_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d axis_ = Eigen::Vector3d::UnitZ();
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<TrajectoryTargetNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

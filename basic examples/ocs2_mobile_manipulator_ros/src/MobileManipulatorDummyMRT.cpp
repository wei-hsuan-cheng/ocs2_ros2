/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "rclcpp/rclcpp.hpp"
// Pause/Resume service
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>
#include <cmath>

// Use pinocchio to compute current end-effector pose(s) from the initial state
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace ocs2;
using namespace mobile_manipulator;

namespace {

/******************************************************************************
This code shares some of the implementations in /robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Dummy_Loop.cpp
but in a more custom manner.
 ******************************************************************************/


/* Helpers */
// Helper:
// No-op hook matching MRT_ROS_Dummy_Loop API
inline void modifyObservation(SystemObservation& /*observation*/) {}

// Helper:
// Forward integrate one step (matches MRT_ROS_Dummy_Loop::forwardSimulation)
inline SystemObservation forwardSimulation(ocs2::MRT_ROS_Interface& mrt,
                                           double mrtDesiredFrequency,
                                           const SystemObservation& currentObservation) {
  const scalar_t dt = 1.0 / mrtDesiredFrequency;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt;
  if (mrt.isRolloutSet()) {
    // If available, use the provided rollout as to
    // integrate the dynamics.
    mrt.rolloutPolicy(currentObservation.time, currentObservation.state, dt,
                      nextObservation.state, nextObservation.input,
                      nextObservation.mode);
  } else {
    // Otherwise, we fake integration by interpolating the current MPC
    // policy at t+dt
    mrt.evaluatePolicy(currentObservation.time + dt, currentObservation.state,
                       nextObservation.state, nextObservation.input,
                       nextObservation.mode);
  }

  return nextObservation;
}

// Helper:
// Check if policy is updated and starts at the given time (check policy freshness in synchronized mode).
// Due to ROS message conversion delay and very fast MPC loop, we might get an
// old policy instead of the latest one.
inline bool policyUpdatedForTime(ocs2::MRT_ROS_Interface& mrt,
                                 double mpcDesiredFrequency,
                                 double time) {
  constexpr double tolFactor = 0.1;  // policy must start within this fraction of dt
  if (!mrt.updatePolicy()) return false;
  const auto& pol = mrt.getPolicy();
  if (pol.timeTrajectory_.empty()) return false;
  const double t0 = pol.timeTrajectory_.front();
  return std::abs(t0 - time) < (tolFactor / std::max(mpcDesiredFrequency, 1e-6));
}

// Helper:
// Synchronized dummy loop (publishes observation at MPC update boundaries)
inline void synchronizedDummyLoop(ocs2::MRT_ROS_Interface& mrt,
                                  double mrtDesiredFrequency,
                                  double mpcDesiredFrequency,
                                  const SystemObservation& initObservation,
                                  const TargetTrajectories& /*initTargetTrajectories*/,
                                  const std::vector<std::shared_ptr<ocs2::DummyObserver>>& observers,
                                  std::atomic<bool>& running,
                                  std::atomic<bool>& resume_requested) {
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio =
      std::max(static_cast<size_t>(mrtDesiredFrequency / std::max(mpcDesiredFrequency, 1e-6)),
               static_cast<size_t>(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;
  bool wasRunning = true;

  rclcpp::Rate simRate(mrtDesiredFrequency);
  while (rclcpp::ok()) {
    if (running.load()) {
      if (resume_requested.load()) {
        // Push current observation once on resume
        mrt.setCurrentObservation(currentObservation);
        resume_requested.store(false);
      }

      // Trigger MRT callbacks
      mrt.spinMRT();

      // Wait for fresh policy at update boundaries
      if (loopCounter % mpcUpdateRatio == 0) {
        while (!policyUpdatedForTime(mrt, mpcDesiredFrequency, currentObservation.time) && rclcpp::ok()) {
          mrt.spinMRT();
        }
      }

      // Forward simulate one step
      auto nextObservation = forwardSimulation(mrt, mrtDesiredFrequency, currentObservation);

      // User-defined modifications before publishing
      modifyObservation(nextObservation);

      // Publish observation only when a new policy will be requested next step
      if ((loopCounter + 1) % mpcUpdateRatio == 0) {
        mrt.setCurrentObservation(nextObservation);
      }

      // Update observers
      for (auto& observer : observers) {
        observer->update(nextObservation, mrt.getPolicy(), mrt.getCommand());
      }

      currentObservation = nextObservation;
      ++loopCounter;
    } else {
      // Paused: do not interact with MRT/MPC; hold the observation constant
      // Optionally still update visualization at a low rate using the last state
    }

    wasRunning = running.load();

    mrt.spinMRT();
    simRate.sleep();
  }
}

// Realtime dummy loop (publishes observation each step, best-effort policy updates)
inline void realtimeDummyLoop(ocs2::MRT_ROS_Interface& mrt,
                              double mrtDesiredFrequency,
                              double /*mpcDesiredFrequency*/,
                              const SystemObservation& initObservation,
                              const TargetTrajectories& /*initTargetTrajectories*/,
                              const std::vector<std::shared_ptr<ocs2::DummyObserver>>& observers,
                              std::atomic<bool>& running,
                              std::atomic<bool>& resume_requested) {
  SystemObservation currentObservation = initObservation;
  bool wasRunning = true;

  rclcpp::Rate simRate(mrtDesiredFrequency);
  while (rclcpp::ok()) {
    if (running.load()) {
      if (resume_requested.load()) {
        // Push current observation once on resume
        mrt.setCurrentObservation(currentObservation);
        resume_requested.store(false);
      }

      // Trigger MRT callbacks
      mrt.spinMRT();

      // Update the policy if a new one was received
      if (mrt.updatePolicy()) {
        // Policy available starting at: mrt.getPolicy().timeTrajectory_.front()
      }

      // Forward simulate one step
      auto nextObservation = forwardSimulation(mrt, mrtDesiredFrequency, currentObservation);

      // User-defined modifications before publishing
      modifyObservation(nextObservation);

      // Publish observation every step
      mrt.setCurrentObservation(nextObservation);

      // Update observers
      for (auto& observer : observers) {
        observer->update(nextObservation, mrt.getPolicy(), mrt.getCommand());
      }

      currentObservation = nextObservation;
    } else {
      // Paused: do not interact with MRT/MPC; hold the observation constant
      // Optionally still update visualization at a low rate using the last state
    }

    wasRunning = running.load();

    simRate.sleep();
  }
}

// Orchestrator matching MRT_ROS_Dummy_Loop::run with pause/resume support
inline void run(ocs2::MRT_ROS_Interface& mrt,
                double mrtDesiredFrequency,
                double mpcDesiredFrequency,
                const SystemObservation& initObservation,
                const TargetTrajectories& initTargetTrajectories,
                const std::vector<std::shared_ptr<ocs2::DummyObserver>>& observers,
                std::atomic<bool>& running,
                std::atomic<bool>& resume_requested) {
  RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorDummyMRT"), "Waiting for the initial policy ...");
  
  // Reset MPC node
  mrt.resetMpcNode(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt.initialPolicyReceived() && rclcpp::ok()) {
    mrt.spinMRT();
    mrt.setCurrentObservation(initObservation);
    rclcpp::Rate(mrtDesiredFrequency).sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorDummyMRT"), "Initial policy has been received.");

  // Pick simulation loop mode (synchronized or best-effort)
  if (mpcDesiredFrequency > 0.0) {
    synchronizedDummyLoop(mrt, mrtDesiredFrequency, mpcDesiredFrequency, initObservation, initTargetTrajectories, observers, running, resume_requested);
  } else {
    realtimeDummyLoop(mrt, mrtDesiredFrequency, mpcDesiredFrequency, initObservation, initTargetTrajectories, observers, running, resume_requested);
  }
}

} // namespace

int main(int argc, char** argv)
{
    const std::string robotName = "mobile_manipulator";

    // Initialize ros node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_mrt",
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));
    // Get node parameters
    std::string taskFile = node->get_parameter("taskFile").as_string();
    std::string libFolder = node->get_parameter("libFolder").as_string();
    std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;
    // Robot Interface
    MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

    // MRT
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(node);

    // Visualization
    auto dummyVisualization =
        std::make_shared<MobileManipulatorDummyVisualization>(
            node, interface);

    // Observers (visualization)
    const double mrtDesiredFrequency = interface.mpcSettings().mrtDesiredFrequency_;
    auto observers = std::vector<std::shared_ptr<ocs2::DummyObserver>>{dummyVisualization};

    // initial state
    SystemObservation initObservation;
    initObservation.state = interface.getInitialState();
    initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
    initObservation.time = 0.0;

    // initial command: set to current end-effector pose(s) to avoid any motion
    vector_t initTarget;

    // Compute forward kinematics for the initial state (via pinocchio interface)
    const auto& pinInterface = interface.getPinocchioInterface();
    const auto& model = pinInterface.getModel();
    auto data = pinInterface.getData();
    pinocchio::forwardKinematics(model, data, initObservation.state);
    pinocchio::updateFramePlacements(model, data);

    const auto& info = interface.getManipulatorModelInfo();

    if (interface.dual_arm_)
    {   
        // Dual-arm mode
        initTarget.resize(14);

        // Left arm end-effector pose
        const auto left_id = model.getFrameId(info.eeFrame);
        const auto& left = data.oMf[left_id];
        Eigen::Quaterniond qL(left.rotation());
        initTarget.segment<3>(0) = left.translation();
        initTarget.segment<4>(3) = qL.coeffs(); // [qx, qy, qz, qw]

        // Right arm end-effector pose
        const auto right_id = model.getFrameId(info.eeFrame1);
        const auto& right = data.oMf[right_id];
        Eigen::Quaterniond qR(right.rotation());
        initTarget.segment<3>(7)  = right.translation();
        initTarget.segment<4>(10) = qR.coeffs(); // [qx, qy, qz, qw]
    }
    else
    {   
        // Singel-arm mode
        initTarget.resize(7);

        const auto ee_id = model.getFrameId(info.eeFrame);
        const auto& ee = data.oMf[ee_id];
        Eigen::Quaterniond q(ee.rotation());
        initTarget.head<3>() = ee.translation();
        initTarget.tail<4>() = q.coeffs(); // [qx, qy, qz, qw]
    }
    const vector_t zeroInput =
        vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
    const TargetTrajectories initTargetTrajectories({initObservation.time},
                                                    {initTarget}, {zeroInput});

    // Pause/Resume control
    std::atomic<bool> running{true};
    std::atomic<bool> resume_requested{false};
    auto srv = node->create_service<std_srvs::srv::SetBool>(
        "toggle_mrt",
        [&](const std::shared_ptr<rmw_request_id_t> /*req_header*/, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
        {
            bool new_state = req->data;
            bool old_state = running.load();
            running.store(new_state);
            if (new_state && !old_state)
            {
                resume_requested.store(true);
                res->message = "MRT resumed";
            }
            else if (!new_state && old_state)
            {
                res->message = "MRT paused";
            }
            else
            {
                res->message = new_state ? "MRT already running" : "MRT already paused";
            }
            res->success = true;
        });
        
    // Orchestrate like MRT_ROS_Dummy_Loop
    run(mrt,
        mrtDesiredFrequency,
        interface.mpcSettings().mpcDesiredFrequency_,
        initObservation,
        initTargetTrajectories,
        observers,
        running,
        resume_requested);

    return 0; // Successful exit
}

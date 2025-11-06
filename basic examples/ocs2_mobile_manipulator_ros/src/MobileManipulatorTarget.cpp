/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_ros_interfaces/command/UnifiedTargetTrajectoriesInteractiveMarker.h>
#include <ocs2_ros_interfaces/command/JoystickMarkerWrapper.h>
#include <ocs2_ros_interfaces/command/MarkerAutoPositionWrapper.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace ocs2;
using namespace ocs2::mobile_manipulator;

/**
 * Read dualArmMode configuration from taskFile
 */
bool readDualArmModeFromTaskFile(const std::string& taskFile)
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        bool dualArmMode = false;
        // Try to read dualArmMode from endEffector or finalEndEffector configuration
        loadData::loadPtreeValue(pt, dualArmMode, "endEffector.dualArmMode", false);
        if (!dualArmMode)
        {
            loadData::loadPtreeValue(pt, dualArmMode, "finalEndEffector.dualArmMode", false);
        }

        return dualArmMode;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error reading dualArmMode from task file: " << e.what() << std::endl;
        return false;
    }
}

/**
 * Read frame information from taskFile, determine which frame to use based on manipulatorModelType
 */
std::string getMarkerFrameFromTaskFile(const std::string& taskFile)
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        size_t manipulatorModelType = 1; // Default is 1 (WheelBasedMobileManipulator)
        loadData::loadPtreeValue(pt, manipulatorModelType, "model_information.manipulatorModelType", false);

        std::string baseFrame = "base_link"; // Default value
        loadData::loadPtreeValue(pt, baseFrame, "model_information.baseFrame", false);

        if (manipulatorModelType == 0) // DefaultManipulator
        {
            return baseFrame;
        }
        else
        {
            return "world";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error reading frame information from task file: " << e.what() << std::endl;
        return "world";
    }
}

/**
 * Converts the pose of the interactive marker to TargetTrajectories (single arm).
 */
TargetTrajectories goalPoseToTargetTrajectories(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
    const SystemObservation& observation)
{
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};
    // state trajectory: 3 + 4 for desired position vector and orientation
    // quaternion
    const vector_t target =
        (vector_t(7) << position, orientation.coeffs()).finished();
    const vector_array_t stateTrajectory{target};
    // input trajectory
    const vector_array_t inputTrajectory{
        vector_t::Zero(observation.input.size())
    };

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Converts the poses of dual arm interactive markers to TargetTrajectories.
 * This function combines both left and right arm target poses into a single trajectory.
 */
TargetTrajectories dualArmGoalPoseToTargetTrajectories(
    const Eigen::Vector3d& leftPosition, const Eigen::Quaterniond& leftOrientation,
    const Eigen::Vector3d& rightPosition, const Eigen::Quaterniond& rightOrientation,
    const SystemObservation& observation)
{
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};

    // state trajectory: 14 dimensions (7 for left arm + 7 for right arm)
    // [left_x, left_y, left_z, left_qw, left_qx, left_qy, left_qz,
    //  right_x, right_y, right_z, right_qw, right_qx, right_qy, right_qz]
    const vector_t target = (vector_t(14) <<
        leftPosition, leftOrientation.coeffs(),
        rightPosition, rightOrientation.coeffs()).finished();

    const vector_array_t stateTrajectory{target};

    // input trajectory
    const vector_array_t inputTrajectory{
        vector_t::Zero(observation.input.size())
    };

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[])
{
    const std::string robotName = "mobile_manipulator";
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_target",
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));

    std::string taskFile = node->get_parameter("taskFile").as_string();
    // Optional: urdfFile and libFolder for FK-based initialization
    std::string urdfFile = "";
    std::string libFolder = "";
    try { urdfFile = node->get_parameter("urdfFile").as_string(); } catch (...) {}
    try { libFolder = node->get_parameter("libFolder").as_string(); } catch (...) {}
    bool dualArmMode = readDualArmModeFromTaskFile(taskFile);

    bool enableDynamicFrame = false;
    if (node->has_parameter("enableDynamicFrame"))
    {
        enableDynamicFrame = node->get_parameter("enableDynamicFrame").as_bool();
        RCLCPP_INFO(node->get_logger(), "enableDynamicFrame parameter found: %s", enableDynamicFrame ? "true" : "false");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "enableDynamicFrame parameter not found, using default: false");
    }

    std::string markerFrame = "world";
    if (enableDynamicFrame)
    {
        markerFrame = getMarkerFrameFromTaskFile(taskFile);
        RCLCPP_INFO(node->get_logger(), "Dynamic frame selection enabled. Using marker frame: %s", markerFrame.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Dynamic frame selection disabled. Using default frame: %s", markerFrame.c_str());
    }

    bool enableJoystick = false;
    try
    {
        enableJoystick = node->get_parameter("enableJoystick").as_bool();
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException&)
    {
        enableJoystick = false;
    }

    bool enableAutoPosition = false;
    try
    {
        enableAutoPosition = node->get_parameter("enableAutoPosition").as_bool();
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException&)
    {
        enableAutoPosition = false;
    }

    std::unique_ptr<JoystickMarkerWrapper> joystickControl;
    std::unique_ptr<MarkerAutoPositionWrapper> autoPositionWrapper;

    // Prepare optional FK interface (for initializing and service hold)
    std::unique_ptr<MobileManipulatorInterface> interfacePtr;
    if (!urdfFile.empty() && !libFolder.empty())
    {
        try
        {
            interfacePtr = std::make_unique<MobileManipulatorInterface>(taskFile, libFolder, urdfFile);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
        }
    }

    // Subscribe to latest observation for service-based hold
    SystemObservation latestObs;
    bool haveObs = false;
    auto obsSub = node->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName + std::string("_mpc_observation"), 1,
        [&](const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
        {
            latestObs = ros_msg_conversions::readObservationMsg(*msg);
            haveObs = true;
        });

    if (dualArmMode)
    {
        // Create dual arm interactive marker
        RCLCPP_INFO(node->get_logger(), "Dual arm mode enabled - creating dual arm interactive markers");
        UnifiedTargetTrajectoriesInteractiveMarker targetPoseCommand(node, robotName,
                                                                     &dualArmGoalPoseToTargetTrajectories, 10.0, markerFrame);

        // Initialize markers to current EE poses (FK of initial state)
        if (interfacePtr)
        {
            try
            {
                const auto& pin = interfacePtr->getPinocchioInterface();
                const auto& model = pin.getModel();
                auto data = pin.getData();
                const auto q0 = interfacePtr->getInitialState();
                pinocchio::forwardKinematics(model, data, q0);
                pinocchio::updateFramePlacements(model, data);

                const auto& info = interfacePtr->getManipulatorModelInfo();
                // Left EE
                const auto left_id = model.getFrameId(info.eeFrame);
                const auto& left = data.oMf[left_id];
                Eigen::Vector3d lp = left.translation();
                Eigen::Quaterniond lq(left.rotation());
                targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::LEFT, lp, lq);
                targetPoseCommand.updateMarkerDisplay("LeftArmGoal", lp, lq);
                // Right EE
                const auto right_id = model.getFrameId(info.eeFrame1);
                const auto& right = data.oMf[right_id];
                Eigen::Vector3d rp = right.translation();
                Eigen::Quaterniond rq(right.rotation());
                targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::RIGHT, rp, rq);
                targetPoseCommand.updateMarkerDisplay("RightArmGoal", rp, rq);
                RCLCPP_INFO(node->get_logger(), "Initialized dual-arm markers from initial EE pose.");
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(node->get_logger(), "FK init for markers failed: %s", e.what());
            }
        }

        // Service to start/stop tracking. True=start (continuous), False=stop and hold current pose
        auto srv = node->create_service<std_srvs::srv::SetBool>(
            "toggle_mpc",
            [&](const std::shared_ptr<rmw_request_id_t> /*req_header*/, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                std::shared_ptr<std_srvs::srv::SetBool::Response> res)
            {
                if (!req->data)
                {
                    // Stop: hold at current EE poses
                    try
                    {
                        Eigen::Vector3d lp, rp;
                        Eigen::Quaterniond lq, rq;
                        if (interfacePtr && haveObs)
                        {
                            const auto& pin = interfacePtr->getPinocchioInterface();
                            const auto& model = pin.getModel();
                            auto data = pin.getData();
                            pinocchio::forwardKinematics(model, data, latestObs.state);
                            pinocchio::updateFramePlacements(model, data);
                            const auto& info = interfacePtr->getManipulatorModelInfo();
                            // Left
                            const auto left_id = model.getFrameId(info.eeFrame);
                            const auto& left = data.oMf[left_id];
                            lp = left.translation();
                            lq = Eigen::Quaterniond(left.rotation());
                            // Right
                            const auto right_id = model.getFrameId(info.eeFrame1);
                            const auto& right = data.oMf[right_id];
                            rp = right.translation();
                            rq = Eigen::Quaterniond(right.rotation());
                        }
                        else
                        {
                            // Fallback to current marker poses
                            std::tie(lp, lq) = targetPoseCommand.getDualArmPose(ocs2::IMarkerControl::ArmType::LEFT);
                            std::tie(rp, rq) = targetPoseCommand.getDualArmPose(ocs2::IMarkerControl::ArmType::RIGHT);
                        }

                        // Update markers and publish once (manual mode)
                        if (targetPoseCommand.isContinuousMode())
                        {
                            targetPoseCommand.togglePublishMode();
                        }
                        targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::LEFT, lp, lq);
                        targetPoseCommand.updateMarkerDisplay("LeftArmGoal", lp, lq);
                        targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::RIGHT, rp, rq);
                        targetPoseCommand.updateMarkerDisplay("RightArmGoal", rp, rq);
                        targetPoseCommand.sendDualArmTrajectories();
                        res->success = true;
                        res->message = "MPC paused; holding current pose";
                    }
                    catch (const std::exception& e)
                    {
                        res->success = false;
                        res->message = std::string("Hold failed: ") + e.what();
                    }
                }
                else
                {
                    // Start: switch to continuous mode
                    if (!targetPoseCommand.isContinuousMode())
                    {
                        targetPoseCommand.togglePublishMode();
                    }
                    res->success = true;
                    res->message = "MPC started (continuous mode)";
                }
            });

        if (enableJoystick)
        {
            RCLCPP_INFO(node->get_logger(), "Joystick marker wrapper enabled");
            joystickControl = std::make_unique<JoystickMarkerWrapper>(node, &targetPoseCommand);
        }

        if (enableAutoPosition)
        {
            RCLCPP_INFO(node->get_logger(), "Marker auto position wrapper enabled");
            autoPositionWrapper = std::make_unique<MarkerAutoPositionWrapper>(
                node, robotName, &targetPoseCommand,
                MarkerAutoPositionWrapper::UpdateMode::CONTINUOUS,
                dualArmMode); // dualArmMode
        }

        spin(node);
        return 0;
    }

    // Single arm mode
    RCLCPP_INFO(node->get_logger(), "Single arm mode enabled");
    UnifiedTargetTrajectoriesInteractiveMarker targetPoseCommand(node, robotName, &goalPoseToTargetTrajectories, 10.0, markerFrame);

    // Initialize marker to current EE pose (FK of initial state)
    if (interfacePtr)
    {
        try
        {
            const auto& pin = interfacePtr->getPinocchioInterface();
            const auto& model = pin.getModel();
            auto data = pin.getData();
            const auto q0 = interfacePtr->getInitialState();
            pinocchio::forwardKinematics(model, data, q0);
            pinocchio::updateFramePlacements(model, data);

            const auto& info = interfacePtr->getManipulatorModelInfo();
            const auto ee_id = model.getFrameId(info.eeFrame);
            const auto& ee = data.oMf[ee_id];
            Eigen::Vector3d p = ee.translation();
            Eigen::Quaterniond q(ee.rotation());
            targetPoseCommand.setSingleArmPose(p, q);
            targetPoseCommand.updateMarkerDisplay("Goal", p, q);
            RCLCPP_INFO(node->get_logger(), "Initialized marker from initial EE pose.");
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node->get_logger(), "FK init for marker failed: %s", e.what());
        }
    }

    // Service to start/stop tracking for single arm
    auto srv = node->create_service<std_srvs::srv::SetBool>(
        "toggle_mpc",
        [&](const std::shared_ptr<rmw_request_id_t> /*req_header*/, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
        {
            if (!req->data)
            {
                try
                {
                    Eigen::Vector3d p;
                    Eigen::Quaterniond q;
                    if (interfacePtr && haveObs)
                    {
                        const auto& pin = interfacePtr->getPinocchioInterface();
                        const auto& model = pin.getModel();
                        auto data = pin.getData();
                        pinocchio::forwardKinematics(model, data, latestObs.state);
                        pinocchio::updateFramePlacements(model, data);
                        const auto& info = interfacePtr->getManipulatorModelInfo();
                        const auto ee_id = model.getFrameId(info.eeFrame);
                        const auto& ee = data.oMf[ee_id];
                        p = ee.translation();
                        q = Eigen::Quaterniond(ee.rotation());
                    }
                    else
                    {
                        std::tie(p, q) = targetPoseCommand.getSingleArmPose();
                    }

                    if (targetPoseCommand.isContinuousMode())
                    {
                        targetPoseCommand.togglePublishMode();
                    }
                    targetPoseCommand.setSingleArmPose(p, q);
                    targetPoseCommand.updateMarkerDisplay("Goal", p, q);
                    targetPoseCommand.sendSingleArmTrajectories();
                    res->success = true;
                    res->message = "MPC paused; holding current pose";
                }
                catch (const std::exception& e)
                {
                    res->success = false;
                    res->message = std::string("Hold failed: ") + e.what();
                }
            }
            else
            {
                if (!targetPoseCommand.isContinuousMode())
                {
                    targetPoseCommand.togglePublishMode();
                }
                res->success = true;
                res->message = "MPC started (continuous mode)";
            }
        });

    if (enableJoystick)
    {
        RCLCPP_INFO(node->get_logger(), "Joystick marker wrapper enabled");
        joystickControl = std::make_unique<JoystickMarkerWrapper>(node, &targetPoseCommand);
    }

    if (enableAutoPosition)
    {
        RCLCPP_INFO(node->get_logger(), "Marker auto position wrapper enabled");
        autoPositionWrapper = std::make_unique<MarkerAutoPositionWrapper>(
            node, robotName, &targetPoseCommand,
            MarkerAutoPositionWrapper::UpdateMode::CONTINUOUS,  // updateMode
            dualArmMode,  // dualArmMode (false for single arm)
            3.0,          // cooldownDuration
            1.0           // maxUpdateFrequency
        );
    }

    spin(node);
    return 0;
}

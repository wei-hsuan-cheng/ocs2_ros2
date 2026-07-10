cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MpcState.msg"
        "msg/MpcInput.msg"
        "msg/ModeSchedule.msg"
        "msg/MpcObservation.msg"
        "msg/MpcPerformanceIndices.msg"
        "msg/MpcTargetTrajectories.msg"
        "msg/IndividualBenchmarks.msg"
        "msg/Benchmarks.msg"
        "msg/IndividualGains.msg"
        "msg/Gains.msg"
        "msg/BezierSegment.msg"
        "msg/BezierTrajectory.msg"
        "msg/ThreeDofPointTS.msg"
        "msg/ThreeDofPath.msg"
        "msg/WalkingVelocityCommand.msg"
        "msg/MpcFrameRelations.msg"
        "msg/MpcJoints.msg"
        "msg/MpcTwists.msg"
        "msg/MpcTargets.msg"
        "msg/DynamicsMpcTargets.msg"
        "msg/JointPositionTarget.msg"
        "msg/ControllerData.msg"
        "msg/MpcFlattenedController.msg"
        "msg/LagrangianMetrics.msg"
        "msg/Multiplier.msg"
        "msg/Constraint.msg"
        "msg/FrameRelationTarget.msg"
        "msg/FrameRelationTargetArray.msg"
        "msg/SingularityConstraintSettings.msg"
        "msg/SingularityConstraintSettingsArray.msg"
        "srv/Reset.srv"
        DEPENDENCIES geometry_msgs std_msgs trajectory_msgs nav_msgs
)

ament_package()

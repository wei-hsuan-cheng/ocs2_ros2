cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_switched_model_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Clean up any previous ament_cmake_python artifacts that clash with symlink-install.
set(_python_build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}")
if(EXISTS "${_python_build_dir}" AND NOT IS_SYMLINK "${_python_build_dir}")
    file(REMOVE_RECURSE "${_python_build_dir}")
endif()

# Declare the message files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Gait.msg"
        "msg/GaitSequence.msg"
        "msg/ScheduledGaitSequence.msg"
        "srv/TrajectoryRequest.srv"
        DEPENDENCIES std_msgs ocs2_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()

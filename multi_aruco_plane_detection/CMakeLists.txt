cmake_minimum_required(VERSION 3.8)
project(multi_aruco_plane_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(aruco_interfaces REQUIRED)

# multi_aruco_plane_detection: given multiple aruco markers, find the best fitting plane the markers lie on
add_executable(${PROJECT_NAME}
    src/multi_aruco_plane_detection.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp aruco_interfaces visualization_msgs Eigen3
)

# required to build cpp code using Eigen library with ROS2
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

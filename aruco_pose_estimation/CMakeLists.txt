cmake_minimum_required(VERSION 3.8)
project(aruco_pose_estimation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
#find_package(open3d REQUIRED)
#find_package(numpy REQUIRED)
#find_package(cv2 REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(tf_transformations REQUIRED)
find_package(aruco_interfaces REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
    scripts/aruco_node.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
    launch rviz config
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
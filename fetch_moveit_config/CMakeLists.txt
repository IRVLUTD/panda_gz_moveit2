cmake_minimum_required(VERSION 3.5)
project(fetch_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(fetch_description REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
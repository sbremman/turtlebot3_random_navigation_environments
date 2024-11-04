cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_random_navigation_environments)

# Uncomment the following line if you use any C++ files and want to enforce C++11 standard
# add_compile_options(-std=c++11)

# Find necessary catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_ros
  turtlebot3_gazebo
  message_generation
)

# Define custom service files
add_service_files(
  FILES
  RandomizeWorld.srv  # Service for randomizing the Gazebo environment
)

# Generate message and service files with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare this project as a catkin package and specify its dependencies
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

# Include directories for header files (if applicable)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install Python scripts into the package's bin destination
catkin_install_python(PROGRAMS
  scripts/random_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

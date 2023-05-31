cmake_minimum_required(VERSION 3.8)
project(vslam2)

set(ORB_SLAM3_DIR
   /home/sundog/catkin_ws/Dev/ORB_SLAM3
)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
MESSAGE("Eigen3 VERSION:")
MESSAGE(${Eigen3_VERSION})
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Pangolin REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
   ${ORB_SLAM3_DIR}
   ${ORB_SLAM3_DIR}/include
   ${ORB_SLAM3_DIR}/include/CameraModels
   ${ORB_SLAM3_DIR}/Thirdparty/Sophus
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${Pangolin_INCLUDE_DIRS}
   /opt/ros/humble/include
)

# Build ROS nodes
set(LIBS
   ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
)

# mono node
add_executable (${PROJECT_NAME}_mono
   src/mono_node.cc
   src/common.cc
)

ament_target_dependencies(${PROJECT_NAME}_mono
  rclcpp
  std_msgs 
  sensor_msgs 
  image_transport 
  cv_bridge
  )

target_link_libraries(${PROJECT_NAME}_mono
rclcpp
std_msgs 
sensor_msgs 
image_transport 
cv_bridge
${LIBS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

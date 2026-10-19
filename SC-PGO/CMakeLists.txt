cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(rosbag REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#   DEPENDS EIGEN3 PCL 
#   INCLUDE_DIRS include
# )

add_executable(ascanRegistration src/scanRegistration.cpp)
ament_target_dependencies(ascanRegistration rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs pcl_conversions tf2_ros)
target_link_libraries(ascanRegistration ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
ament_target_dependencies(alaserOdometry rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs pcl_conversions tf2_ros)
target_link_libraries(alaserOdometry ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
ament_target_dependencies(alaserMapping rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs pcl_conversions tf2 tf2_ros)
target_link_libraries(alaserMapping ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
ament_target_dependencies(alaserPGO rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs pcl_conversions tf2 tf2_ros tf2_geometry_msgs)
target_compile_options(alaserPGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO 
  ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_measurement_utils test/test_measurement_utils.cpp)
  target_include_directories(test_measurement_utils PRIVATE include)
  ament_add_gtest(test_pgo_factors test/test_pgo_factors.cpp)
  target_include_directories(test_pgo_factors PRIVATE include)
  target_link_libraries(test_pgo_factors gtsam)
  ament_add_gtest(test_pgo_transforms test/test_pgo_transforms.cpp)
  target_include_directories(test_pgo_transforms PRIVATE include)
  target_link_libraries(test_pgo_transforms gtsam)
  find_program(BASH_EXECUTABLE bash REQUIRED)
  add_test(
    NAME test_pgo_shutdown
    COMMAND ${BASH_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/run_shutdown_smoke.sh
      $<TARGET_FILE:alaserPGO>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/pgo_ros2.yaml
  )
  set_tests_properties(test_pgo_shutdown PROPERTIES TIMEOUT 20)
endif()

install(TARGETS
  ascanRegistration
  alaserOdometry
  alaserMapping
  alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

# add_executable(kittiHelper src/kittiHelper.cpp)
# ament_target_dependencies(kittiHelper rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs)
# target_link_libraries(kittiHelper ${PCL_LIBRARIES} ${OpenCV_LIBS})


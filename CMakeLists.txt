cmake_minimum_required(VERSION 3.0.2)
project(lidar_calibration)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization)
find_package(OpenCV REQUIRED)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(select_pointcloud 
  src/select_pointcloud.cpp
)

add_executable(ransac_ground 
  src/ransac_ground.cpp
)

add_executable(calibration 
  src/calibration.cpp
)

target_link_libraries(select_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(ransac_ground
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
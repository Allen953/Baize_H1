cmake_minimum_required(VERSION 2.8.3)

project(LIUZU)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_state_publisher
  robot_state_publisher
  roscpp
  rospy
  rviz
  xacro
  kdl_parser
  tf
  sensor_msgs
  std_msgs
  trac_ik_lib
  serial
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

find_package(roslaunch)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadruped_9g
   CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control joint_state_publisher robot_state_publisher roscpp rospy rviz xacro trac_ik_lib
   DEPENDS Boost orocos_kdl
)

catkin_install_python(PROGRAMS
  scripts/talker_p.py
  scripts/listener_p.py
  scripts/serial_demo.py
  scripts/teleop_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(liuzu src/liuzu.cpp)
target_link_libraries(liuzu ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(control_pig src/control_pig.cpp)
target_link_libraries(control_pig ${catkin_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

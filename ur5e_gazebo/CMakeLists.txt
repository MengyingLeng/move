cmake_minimum_required(VERSION 2.8.3)

project(ur5e_gazebo)

add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy 
  std_msgs 
  actionlib 
  control_msgs 
  trajectory_msgs
)

catkin_package()
catkin_install_python(PROGRAMS ikfastpy/action_lib_simple_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(controller_test_node src/controller_test.cpp)

target_link_libraries( 
  controller_test_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  controller_test_node
  ${catkin_EXPORTED_TARGETS}
)





install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

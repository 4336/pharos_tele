cmake_minimum_required(VERSION 3.0.2)
project(pharos_path_prediction)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  rosgraph_msgs
  rospy
  std_msgs
  nav_msgs
  pharos_msgs
)

# find_package(OpenCV REQUIRED)

# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   nav_msgs
# )

# generate_dynamic_reconfigure_options(
#   cfg/pharosImageTransport.cfg
# )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rosgraph_msgs rospy std_msgs pharos_msgs
)
# dynamic_reconfigure

include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(path_prediction src/nodes/path_predict_node.cpp src/path_predict.cpp)
target_link_libraries(path_prediction ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(path_prediction pharos_msgs_generate_messages_cpp)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   add_rostest_gtest(test_node_example_talker test/test_node_example_talker.test test/test_node_example_talker.cpp)
#   target_link_libraries(test_node_example_talker ${catkin_LIBRARIES})
#   add_dependencies(test_node_example_talker node_example_generate_messages_cpp)

# endif()


# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# # catkin_install_python(PROGRAMS src/pylistener.py src/pytalker.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS path_prediction
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

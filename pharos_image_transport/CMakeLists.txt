cmake_minimum_required(VERSION 3.0.2)
project(pharos_image_transport)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  rosgraph_msgs
  rospy
  std_msgs

  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  PharosImageTransportData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/pharosImageTransport.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(pretreat src/nodes/pretreat_node.cpp src/pretreat.cpp)
target_link_libraries(pretreat ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pretreat pharos_image_transport_gencfg pharos_image_transport_generate_messages_cpp)

add_executable(crop src/nodes/crop_node.cpp src/crop.cpp)
target_link_libraries(crop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(crop pharos_image_transport_gencfg pharos_image_transport_generate_messages_cpp)

add_executable(view src/nodes/view_node.cpp src/view.cpp)
target_link_libraries(view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(view pharos_image_transport_gencfg pharos_image_transport_generate_messages_cpp)

add_executable(stamped_view src/nodes/stamped_view_node.cpp src/stamped_view.cpp)
target_link_libraries(stamped_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(stamped_view pharos_image_transport_gencfg pharos_image_transport_generate_messages_cpp)

add_executable(cam_info_pub src/nodes/cam_info_pub_node.cpp src/cam_info_pub.cpp)
target_link_libraries(cam_info_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(cam_info_pub pharos_image_transport_gencfg pharos_image_transport_generate_messages_cpp)
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

install(TARGETS pretreat
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

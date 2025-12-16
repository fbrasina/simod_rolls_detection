cmake_minimum_required(VERSION 3.0.2)
project(simod_rolls_detection)

add_subdirectory(pallet_detection)

find_package(catkin REQUIRED COMPONENTS
roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  tf
  tf_conversions
  tf2_ros
  tf2_eigen
  eigen_conversions
  actionlib
  actionlib_msgs
  message_generation
  controller_manager_msgs
  ur_msgs
  ima_planner
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
add_message_files(
  DIRECTORY msg
  FILES ReferenceImage.msg
)

add_action_files(
  DIRECTORY action
  FILES DetectPallet.action
        DetectCloseLine.action
        DetectPacks.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES simod_rolls_detection
# #  CATKIN_DEPENDS other_catkin_pkg
# #  DEPENDS system_lib
# )


catkin_package(
  CATKIN_DEPENDS
    roscpp roslib sensor_msgs std_msgs std_srvs geometry_msgs
    visualization_msgs tf tf_conversions tf2_ros tf2_eigen eigen_conversions
    actionlib actionlib_msgs message_runtime
    controller_manager_msgs ur_msgs
    ima_planner
    cv_bridge
  # PCL/OpenCV/Ceres sono system libs -> non in CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  pallet_detection/src
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/simod_rolls_detection
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(pallet_detection_node 
               src/pallet_detection_node.cpp
               )

add_dependencies(pallet_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pallet_detection_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  pallet_detection_lib
  jsoncpp
)

add_executable(close_line_detection_node 
               src/close_line_detection_node.cpp
               )

add_dependencies(close_line_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(close_line_detection_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  pallet_detection_lib
  jsoncpp
)

add_executable(pallet_detection_test 
               src/pallet_detection_test.cpp
               )

add_dependencies(pallet_detection_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pallet_detection_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  jsoncpp
)

add_executable(close_line_detection_test 
               src/close_line_detection_test.cpp
               )

add_dependencies(close_line_detection_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(close_line_detection_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  jsoncpp
)

add_executable(roll_pack_detection_node
               src/roll_pack_detection_node.cpp
               )

add_dependencies(roll_pack_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pack_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pallet_detection_lib
)

add_executable(roll_pack_detection_test
               src/roll_pack_detection_test.cpp
               )

add_dependencies(roll_pack_detection_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pack_detection_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(save_images 
               src/save_images.cpp
               )

add_dependencies(save_images ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(save_images
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



add_executable(simod_vision_ctrl_node
  src/simod_vision_ctrl_node.cpp
  src/simod_vision_ctrl.cpp
  src/ur_cobot.cpp
  src/ur_cobot_utils.cpp
  src/utils_rot.cpp
  src/filters.cpp
)
add_dependencies(simod_vision_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simod_vision_ctrl_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  jsoncpp
)
#############
## Install ##
#############

# install(TARGETS pallet_detection_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS save_images
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS
  pallet_detection_node
  close_line_detection_node
  simod_vision_ctrl_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  save_images
  pallet_detection_test
  close_line_detection_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
cmake_minimum_required(VERSION 3.8)
project(simod_pallet_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set(EXTRA_OPENCV_FILES "")
if("${OpenCV_VERSION}" VERSION_LESS "4.5.0")
  set(EXTRA_OPENCV_FILES "sift/sift.cpp")
  add_definitions("-DEXTRA_SIFT_FILES")
endif()

include_directories(SYSTEM ${PCL_INCLUDE_DIRS}) # SYSTEM disables warnings
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CERES_INCLUDE_DIRS})

add_definitions("-Wno-unused-local-typedefs")

add_library(pallet_detection_lib SHARED
  src/pallet_detection.cpp
  src/pallet_ransac.cpp
  src/pallet_from_image.cpp
  src/pallet_detection_solver.cpp
  src/boxes_to_pallet_description.cpp
  src/close_line_detection.cpp
  
  src/roll_pack_detection.cpp
  src/roll_pack_detection_model.cpp
  src/roll_pack_pose_model.cpp

  ${EXTRA_OPENCV_FILES}
)
target_link_libraries(pallet_detection_lib
  opencv_highgui
  opencv_imgproc
  opencv_video
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

install(
  TARGETS pallet_detection_lib
  RUNTIME DESTINATION lib/simod_rolls_detection
)


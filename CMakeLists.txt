cmake_minimum_required(VERSION 3.8)
project(seekandcapture-cibernots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  vision_msgs
  darknet_ros_msgs
  image_transport
  cv_bridge
  OpenCV
  message_filters
  pcl_conversions
  pcl_ros
  image_geometry
  depth_image_proc
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/perception_asr/DarknetDetectionNode.cpp
  src/perception_asr/DetectionTo3DfromDepthNode.cpp
  src/tf2_detector/PersonDetectorImprovedNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(darknet_detection_tf src/darknet_detection_main.cpp)
ament_target_dependencies(darknet_detection_tf ${dependencies})
target_link_libraries(darknet_detection_tf ${PROJECT_NAME})

add_executable(detection_2d_to_3d_depth_tf src/detection_2d_to_3d_depth_main.cpp)
ament_target_dependencies(detection_2d_to_3d_depth_tf ${dependencies})
target_link_libraries(detection_2d_to_3d_depth_tf ${PROJECT_NAME})

add_executable(imageperson_tf src/detector_improved_main.cpp)
ament_target_dependencies(imageperson_tf ${dependencies})
target_link_libraries(imageperson_tf ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  darknet_detection_tf
  detection_2d_to_3d_depth_tf
  imageperson_tf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
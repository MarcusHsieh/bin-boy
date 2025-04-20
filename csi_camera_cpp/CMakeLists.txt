cmake_minimum_required(VERSION 3.8)
project(csi_camera_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn highgui)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${rclcpp_components_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_nodes SHARED
  src/csi_camera_node.cpp
  src/image_viewer_node.cpp
  src/person_detector_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  vision_msgs
  ament_index_cpp
  OpenCV
)
target_link_libraries(${PROJECT_NAME}_nodes
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_nodes(${PROJECT_NAME}_nodes
  "csi_camera_cpp::CSICameraNode"
  "csi_camera_cpp::ImageViewerNode"
  "csi_camera_cpp::PersonDetectorNode"
)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

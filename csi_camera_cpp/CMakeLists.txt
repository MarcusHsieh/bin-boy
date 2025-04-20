cmake_minimum_required(VERSION 3.8)
project(csi_camera_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED) # Find OpenCV

# Add include directories
include_directories(include)
# Add OpenCV include directory
include_directories(${OpenCV_INCLUDE_DIRS})
# Add rclcpp_components include directory
include_directories(${rclcpp_components_INCLUDE_DIRS})

# Define the node library (needed for composable nodes) - must be SHARED
add_library(${PROJECT_NAME}_nodes SHARED
  src/csi_camera_node.cpp
  src/image_viewer_node.cpp # Add viewer source file
)
target_link_libraries(${PROJECT_NAME}_nodes
  rclcpp::rclcpp # Use modern CMake target
  # rclcpp_components::rclcpp_components # Removed - registration handled by macro
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${sensor_msgs_LIBRARIES} # Use library variable instead of target
  ${OpenCV_LIBRARIES} # Link OpenCV libraries
)

# Register the node components
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes
  "csi_camera_cpp::CSICameraNode"
  "csi_camera_cpp::ImageViewerNode" # Register viewer node
)

# Install the library
install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Add linters etc.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

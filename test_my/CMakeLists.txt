cmake_minimum_required(VERSION 3.8)
project(test_my)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(protocol REQUIRED)

# add_executable(camera_subscriber_depth_node src/camera_subscriber_color_depth.cpp)
# add_executable(camera_subscriber_node src/camera_subscriber_node.cpp)
add_executable(client_node_cpp src/test_client.cpp)
# target_include_directories(camera_subscriber_depth_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_include_directories(camera_subscriber_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_include_directories(client_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_compile_features(camera_subscriber_depth_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_compile_features(camera_subscriber_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(client_node_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   camera_subscriber_depth_node
#   "rclcpp"
#   "cv_bridge"
# )
# ament_target_dependencies(
#   camera_subscriber_node
#   "rclcpp"
#   "cv_bridge"
# )

ament_target_dependencies(
  client_node_cpp
  "rclcpp"
  "cv_bridge"
  protocol
)

# install(TARGETS camera_subscriber_depth_node
#   DESTINATION lib/${PROJECT_NAME})
# install(TARGETS camera_subscriber_node
#   DESTINATION lib/${PROJECT_NAME})
install(TARGETS client_node_cpp
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

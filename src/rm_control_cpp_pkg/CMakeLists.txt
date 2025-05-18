cmake_minimum_required(VERSION 3.8)
project(rm_control_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

################## 自行添加 #####################
# 添加依赖
find_package(rclcpp REQUIRED)

# 添加源文件      可执行文件名
add_executable(USB2CAN_node src/USB2CAN_node.cpp)

# 添加源文件依赖项           可执行文件名
ament_target_dependencies(USB2CAN_node
  rclcpp
)

# 安装可执行文件
install(TARGETS
  USB2CAN_node
  DESTINATION lib/${PROJECT_NAME}
)

################## 自行添加 #####################

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

ament_package()

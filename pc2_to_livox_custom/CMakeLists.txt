cmake_minimum_required(VERSION 3.8)
project(pc2_to_livox_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找核心依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# 2. 编译 C++ 节点可执行文件
add_executable(pc2_to_custom_node src/pc2_to_custom_node.cpp)

# 3. 链接目标依赖
ament_target_dependencies(pc2_to_custom_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

# 4. 安装规则：将可执行文件安装到 lib 目录
install(TARGETS
  pc2_to_custom_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5. 安装规则：将 launch 文件夹拷贝到 share 目录，确保 ros2 launch 能找到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

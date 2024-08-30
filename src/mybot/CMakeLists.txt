cmake_minimum_required(VERSION 3.22)
project(mybot)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)  # 添加 moveit_msgs 依赖
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# 手动设置 serial 库的路径
include_directories(/home/fins/ws_moveit2/src/serial/include)
link_directories(/home/fins/ws_moveit2/src/serial/build/devel/lib)

# 添加可执行文件
add_executable(move_group_interface src/move_group_interface.cpp)
add_executable(trajectory_subscriber_node src/trajectory_subscriber_node.cpp)

# 链接依赖库
ament_target_dependencies(move_group_interface 
  rclcpp 
  moveit_core 
  moveit_ros_planning_interface 
  geometry_msgs 
  Eigen3
  moveit_visual_tools
  tf2
  tf2_eigen
  tf2_ros
)

ament_target_dependencies(trajectory_subscriber_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  moveit_msgs  # 添加 moveit_msgs 依赖
  tf2
  tf2_eigen
  tf2_ros
)

# 手动链接 serial 库
target_link_libraries(trajectory_subscriber_node
  serial  # 直接链接 serial 库
)

# 安装可执行文件
install(TARGETS move_group_interface trajectory_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

# 安装src目录中的所有文件
install(DIRECTORY src/ DESTINATION share/${PROJECT_NAME}/src
  FILES_MATCHING PATTERN "*.cpp" PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.c"
)

# 安装config目录中的所有文件
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 安装launch目录和其他文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()


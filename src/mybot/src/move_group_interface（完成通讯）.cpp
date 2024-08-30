#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.hpp>  // 更新为推荐的头文件
#include <memory>
#include <limits> // 用于检测NaN值

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

bool is_valid_pose(const geometry_msgs::msg::Pose::SharedPtr msg) {
  // 检查x, y, z是否为NaN
  if (std::isnan(msg->position.x) || std::isnan(msg->position.y) || std::isnan(msg->position.z)) {
    RCLCPP_ERROR(LOGGER, "Invalid pose: NaN value detected in x, y, or z.");
    return false;
  }
  
  // 可以添加更多的检测逻辑，例如检查值是否在合理范围内
  return true;
}

Eigen::Vector3d calculate_displacement_vector(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& target_pose) {
  Eigen::Vector3d start_position(start_pose.position.x, start_pose.position.y, start_pose.position.z);
  Eigen::Vector3d target_position(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  return target_position - start_position;
}

int main(int argc, char** argv){
  rclcpp::init(argc,argv);// 初始化ROS2节点系统，使系统可以处理ROS2的参数和通信功能
  rclcpp::NodeOptions node_options;// 创建节点选项的对象，配置节点的行为
  // 设置节点自动从命令行参数中声明参数，便于动态调整参数。
  node_options.automatically_declare_parameters_from_overrides(true);

  // 创建节点 并调用前面的节点选项 move_group_interface是名称
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  // 创建一个单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;

  // 添加节点到执行器中，以便使其可以处理回调
  executor.add_node(move_group_node);

  // 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
  std::thread([&]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "my_group"; // 定义机器人规划组的名称

  // 创建MoveGroupInterface实例，用于控制和规划指定的"robot_arm"规划组。
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // 创建PlanningSceneInterface实例，用于管理和更新规划场景的碰撞对象。
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 获取当前机器人状态，并从中获取关于"robot_arm"规划组的信息。
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;

  bool received_valid_pose = false; // 标志位，表示是否接收到有效的目标位置

  // 创建一个订阅者来接收x, y, z，并更新目标位置
  auto pose_subscriber = move_group_node->create_subscription<geometry_msgs::msg::Pose>(
      "output_topic", 10,
      [&target_pose1, &received_valid_pose](const geometry_msgs::msg::Pose::SharedPtr msg) {
        // 检查消息是否为有效的格式
        if (!is_valid_pose(msg)) {
          return; // 如果格式无效，不做任何处理
        }

        // 更新target_pose1的位置
        target_pose1.position.x = msg->position.x;
        target_pose1.position.y = msg->position.y;
        target_pose1.position.z = msg->position.z;

        RCLCPP_INFO(LOGGER, "Updated target_pose1: x=%f, y=%f, z=%f",
                    target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);

        received_valid_pose = true; // 设置标志位表示接收到有效的目标位置
      });

  // 等待接收有效的目标位置
  while (!received_valid_pose) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(LOGGER, "等待发布位置");
  }

  // 尝试的容差值列表
  std::vector<double> tolerances = {0.0, 0.1, 1.0, 10.0};
  bool success = false;

  // 逐步增加容差并尝试规划
  for (double tolerance : tolerances) {
    // 设置允许的误差
    if(tolerance != 0.0)
      move_group.setGoalTolerance(tolerance);

    // 将目标姿态设置为MoveGroup的目标，MoveGroup会计划如何达到这一姿态。
    move_group.setPoseTarget(target_pose1);

    // 计划从当前位置到目标姿态的移动。
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);// 判断是否成功

    if(success){
      RCLCPP_INFO(move_group_node->get_logger(),"规划成功，误差容忍度: %f", tolerance);

      // 获取假设执行后的终点位姿
      moveit::core::RobotStatePtr executed_state = move_group.getCurrentState();
      executed_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
      const Eigen::Isometry3d& executed_transform = executed_state->getGlobalLinkTransform(move_group.getEndEffectorLink());
      geometry_msgs::msg::Pose executed_pose = tf2::toMsg(executed_transform);

      // 计算位移向量（从假设执行后的位姿到目标位置的位移向量）
      Eigen::Vector3d displacement = calculate_displacement_vector(executed_pose, target_pose1);
      RCLCPP_INFO(move_group_node->get_logger(), "假设执行后到目标位置的位移向量: x=%f, y=%f, z=%f",
                  displacement.x(), displacement.y(), displacement.z());

      // 只有当容忍度为0.0时，才执行计划
      if (tolerance == 0.0) {
        move_group.execute(my_plan);
        RCLCPP_INFO(move_group_node->get_logger(),"计划执行成功，误差容忍度: %f", tolerance);
      } else {
        RCLCPP_INFO(move_group_node->get_logger(),"计划未执行，仅进行了规划，误差容忍度: %f", tolerance);
      }

      break; // 如果规划成功，不再尝试其他容差
    } else {
      RCLCPP_WARN(move_group_node->get_logger(),"规划失败，尝试下一步误差容忍度: %f", tolerance);
    }
  }

  if (!success) {
    RCLCPP_ERROR(move_group_node->get_logger(),"规划失败，即使允许最大误差10.0");
  }

  rclcpp::shutdown();
  return 0;
}


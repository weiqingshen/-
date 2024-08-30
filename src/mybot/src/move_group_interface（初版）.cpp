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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

int main(int argc, char** argv){
  rclcpp::init(argc,argv);//初始化ROS2节点系统，使系统可以处理ROS2的参数和通信功能
  rclcpp::NodeOptions node_options;//创建节点选项的对象，配置节点的行为
  // 设置节点自动从命令行参数中声明参数，便于动态调整参数。
node_options.automatically_declare_parameters_from_overrides(true);

//创建节点 并调用前面的节点选项 move_group_interface是名称
auto move_group_node=rclcpp::Node::make_shared("move_group_interface",node_options);

//创建一个单线程执行器
rclcpp::executors::SingleThreadedExecutor executor;

//添加节点到执行器中，以便使其可以处理回调
executor.add_node(move_group_node);

// 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
std::thread([&]() { executor.spin(); }).detach();

static const std::string PLANNING_GROUP="my_group";//定义机器人规划组的名称

// 创建MoveGroupInterface实例，用于控制和规划指定的"robot_arm"规划组。
moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

// 创建PlanningSceneInterface实例，用于管理和更新规划场景的碰撞对象。
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// 获取当前机器人状态，并从中获取关于"robot_arm"规划组的信息。
const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x =  0.015202;
  target_pose1.position.y = 0.02949;
  target_pose1.position.z = 0.262147; 

// 将目标姿态设置为MoveGroup的目标，MoveGroup会计划如何达到这一姿态。
move_group.setPoseTarget(target_pose1);

// 计划从当前位置到目标姿态的移动。
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);//判断是否成功

  if(success){
  move_group.execute(my_plan);
  RCLCPP_INFO(move_group_node->get_logger(),"规划成功");
  }
  else{
    RCLCPP_ERROR(move_group_node->get_logger(),"规划失败");
  }

  rclcpp::shutdown();
  return 0;
}
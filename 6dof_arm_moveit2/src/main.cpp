#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

auto const logger = rclcpp::get_logger("hello_moveit");


int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "1. ******");
  rclcpp::init(argc, argv);


  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hello_moveit", node_options);

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");

  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");

  // Create the MoveIt MoveGroup Interface
  //using moveit::planning_interface::MoveGroupInterface;

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");
  //auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose Goal
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.3;  //Limit > 0.28
    msg.position.y = -0.0; //limit > -0.2
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
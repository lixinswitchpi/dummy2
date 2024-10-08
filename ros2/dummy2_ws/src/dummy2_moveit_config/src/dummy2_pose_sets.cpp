#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "dummy_arm");


  // geometry_msgs::msg::PoseStamped target_pose1;
  // target_pose1.header.frame_id = "base_link";
  // target_pose1.pose.orientation.w = 0.17523;
  // target_pose1.pose.position.x = -0.18585;
  // target_pose1.pose.position.y = 0.70344;
  // target_pose1.pose.position.z = -0.66327;
  // planning_components->setGoal(target_pose1, "link6_1_1");

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = -0.18585;
  msg.orientation.y = 0.70344;
  msg.orientation.z = -0.66327;
  msg.orientation.w = 0.17523;
  msg.position.x = 0.11973;
  msg.position.y = -0.17823;
  msg.position.z = 0.14591;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
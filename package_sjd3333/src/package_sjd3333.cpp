#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "package_sjd3333",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("package_sjd3333");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_arm = MoveGroupInterface(node, "panda_arm");
  auto move_group_hand = MoveGroupInterface(node, "hand");

  std::map<std::string, double> joint_values;
  joint_values["panda_finger_joint1"] = 0.0124; 

  move_group_hand.setJointValueTarget(joint_values);


  if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to move the gripper to the specified position!");
        return 1; 
    }

// Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.05307640880346298;
    msg.orientation.y = 0.9971625804901123;
    msg.orientation.z = -0.04374096170067787;
    msg.orientation.w = 0.030601151287555695;
    msg.position.x = -0.6941039562225342;
    msg.position.y = 0.08451995998620987;
    msg.position.z = 0.3288669288158417;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    if (move_group_arm.execute(plan)) {
      // Set custom joint values to close the gripper
      std::map<std::string, double> joint_values;
      joint_values["panda_finger_joint1"] = 0.00;  
      move_group_hand.setJointValueTarget(joint_values);

      if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to close the gripper!");
      }
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for arm failed!");
  };

// Set a target Pose
  auto const target_pose2 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.9789465069770813;
    msg.orientation.y = 0.176754891872406;
    msg.orientation.z = 0.09022073447704315;
    msg.orientation.w = -0.04776712879538536;
    msg.position.x = 0.7001023888587952;
    msg.position.y = 0.2793777585029602;
    msg.position.z = 0.5231913328170776;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose2);

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    if (move_group_arm.execute(plan2)) {
      if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to close the gripper!");
      }
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for arm failed!");
  };

// Set a target Pose
  auto const target_pose3 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.923910915851593;
    msg.orientation.y = 0.3826076090335846;
    msg.orientation.z = -6.014058817527257e-05;
    msg.orientation.w = -2.4912691515055485e-05;
    msg.position.x = 0.0880732536315918;
    msg.position.y = -1.974851511477027e-05;
    msg.position.z = 0.8675949573516846;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose3);

  // Create a plan to that target pose
  auto const [success3, plan3] = [&move_group_arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    if (move_group_arm.execute(plan3)) {
      // Set custom joint values to close the gripper
      std::map<std::string, double> joint_values;
      joint_values["panda_finger_joint1"] = 0.024;  
      move_group_hand.setJointValueTarget(joint_values);

      if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to close the gripper!");
      }
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for arm failed!");
  };

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
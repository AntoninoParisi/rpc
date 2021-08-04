#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GlueSealing");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP_ARM = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);




  

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "GlueSealing", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  ROS_INFO_NAMED("GlueSealing", "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("GlueSealing", "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("GlueSealing", "Available Planning Groups:");
  std::copy(move_group_arm.getJointModelGroupNames().begin(),
            move_group_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


 
 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose target_pose;
  geometry_msgs::Pose current_pose;
  std::vector<double> joint_group_positions;
  moveit::core::RobotStatePtr current_state = move_group_arm.getCurrentState();
  



  /*
    close gripper
  */

  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group_arm_gripper(PLANNING_GROUP_GRIPPER);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_gripper;
  const moveit::core::JointModelGroup* joint_model_group_gripper =
  move_group_arm_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  moveit::planning_interface::MoveGroupInterface::Plan gripper_close;

  std::vector<double> joint_group_positions_gripper;
  moveit::core::RobotStatePtr current_state_gripper = move_group_arm_gripper.getCurrentState();
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);

  joint_group_positions_gripper[0] = 0.032;
  joint_group_positions_gripper[1] = 0.032;
  move_group_arm_gripper.setJointValueTarget(joint_group_positions_gripper);
  auto success = (move_group_arm_gripper.plan(gripper_close) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("GlueSealing", "Closing the gripper: %s", success ? "SUCCESS" : "FAILED");
  move_group_arm_gripper.execute(gripper_close);

  visual_tools.prompt("Press 'next' to start the homework");

 
  std::vector<geometry_msgs::Pose> waypoints;

  

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  

  current_pose = move_group_arm.getCurrentPose().pose;

  tf2::Quaternion q;

  q.setRPY(M_PI,0,M_PI);


  geometry_msgs::Pose a_vertex = current_pose;
  a_vertex.position.x = 0.2;
  a_vertex.position.y = -0.4;
  a_vertex.position.z = 0.15;
  a_vertex.orientation.x = q.x();
  a_vertex.orientation.y = q.y();
  a_vertex.orientation.z = q.z();
  a_vertex.orientation.w = q.w();



  geometry_msgs::Pose b_vertex = current_pose;
  b_vertex.position.x = 0.3;
  b_vertex.position.y = -0.4;
  b_vertex.position.z = 0.15;
  b_vertex.orientation.x = q.x();
  b_vertex.orientation.y = q.y();
  b_vertex.orientation.z = q.z();
  b_vertex.orientation.w = q.w();


  geometry_msgs::Pose c_vertex = current_pose;
  c_vertex.position.x = 0.3;
  c_vertex.position.y = -0.55;
  c_vertex.position.z = 0.15;
  c_vertex.orientation.x = q.x();
  c_vertex.orientation.y = q.y();
  c_vertex.orientation.z = q.z();
  c_vertex.orientation.w = q.w();


  geometry_msgs::Pose x_vertex = current_pose;
  x_vertex.position.x = 0.25;
  x_vertex.position.y = -0.45;
  x_vertex.position.z = 0.15;
  x_vertex.orientation.x = q.x();
  x_vertex.orientation.y = q.y();
  x_vertex.orientation.z = q.z();
  x_vertex.orientation.w = q.w();



  geometry_msgs::Pose d_vertex = current_pose;
  d_vertex.position.x = 0.2;
  d_vertex.position.y = -0.55;
  d_vertex.position.z = 0.15;
  d_vertex.orientation.x = q.x();
  d_vertex.orientation.y = q.y();
  d_vertex.orientation.z = q.z();
  d_vertex.orientation.w = q.w();


  move_group_arm.setPoseTarget(a_vertex);
  move_group_arm.move();

  waypoints.clear();
  waypoints.push_back(a_vertex);
  waypoints.push_back(b_vertex);
  waypoints.push_back(x_vertex);
  waypoints.push_back(c_vertex);
  waypoints.push_back(d_vertex);
  waypoints.push_back(a_vertex);


  fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  move_group_arm.execute(trajectory);

  

  ros::shutdown();
  return 0;
}

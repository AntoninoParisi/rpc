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
  ros::init(argc, argv, "move_group_arm_node");
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
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_arm.getJointModelGroupNames().begin(),
            move_group_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // visual_tools.prompt("Press 'next' to start the homework");

  /*
    RESET TO HOME
  */
 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose target_pose;
  geometry_msgs::Pose current_pose;
  std::vector<double> joint_group_positions;
  moveit::core::RobotStatePtr current_state = move_group_arm.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 0;
  move_group_arm.setJointValueTarget(joint_group_positions);
  auto success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("HomeworkB2", "Returning to home position: %s", success ? "SUCCESS" : "FAILED");
  move_group_arm.execute(my_plan);



  /*
    close gripper
  */

  // static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  // moveit::planning_interface::MoveGroupInterface move_group_arm_gripper(PLANNING_GROUP_GRIPPER);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_gripper;
  // const moveit::core::JointModelGroup* joint_model_group_gripper =
  // move_group_arm_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  // moveit::planning_interface::MoveGroupInterface::Plan gripper_close;

  // std::vector<double> joint_group_positions_gripper;
  // moveit::core::RobotStatePtr current_state_gripper = move_group_arm_gripper.getCurrentState();
  // current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);

  // joint_group_positions_gripper[0] = 0.05;
  // joint_group_positions_gripper[1] = 0.05;
  // move_group_arm_gripper.setJointValueTarget(joint_group_positions_gripper);
  // success = (move_group_arm_gripper.plan(gripper_close) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("HomeworkB2", "Closing the gripper: %s", success ? "SUCCESS" : "FAILED");
  // move_group_arm_gripper.execute(gripper_close);

  visual_tools.prompt("Press 'next' to start the homework");

  // /*
  //   C: Command (plan and optionally execute) the robot to move to a specific pose
  // */

  // target_pose.position.x = 0.7;
  // target_pose.position.y = 0.1;
  // target_pose.position.z = 0.7;
  // move_group_arm.setPoseTarget(target_pose);

  // success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // visual_tools.publishAxisLabeled(target_pose, "Pose 1"); // Visualize it
  // visual_tools.publishText(text_pose, "Trajectory in cartesian space", rvt::WHITE, rvt::XXXLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' to move to the shown pose");
  // move_group_arm.execute(my_plan);
  // visual_tools.prompt("Press 'next' to proceed");

  // /*
  //   D: Command (plan and optionally execute) the robot to move to a specific joint state
  // */
  // current_state = move_group_arm.getCurrentState();
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = 0;
  // joint_group_positions[1] = -M_PI/2;
  // joint_group_positions[2] = 0;
  // joint_group_positions[3] = -M_PI/2;
  // joint_group_positions[4] = 0;
  // joint_group_positions[5] = 0;
  // move_group_arm.setJointValueTarget(joint_group_positions);
  // success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Goal in joint space", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // move_group_arm.execute(my_plan);
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /*
    E: Command (plan and optionally execute) the robot to follow a linear path between two Cartesian poses
  */

  // current_pose = move_group_arm.getCurrentPose().pose;

  // moveit_msgs::OrientationConstraint ocm;
  // ocm.link_name = "robot_tool0";
  // ocm.header.frame_id = "robot_base_link";
  // ocm.orientation = current_pose.orientation;
  // ocm.absolute_x_axis_tolerance = 0.01;
  // ocm.absolute_y_axis_tolerance = 0.01;
  // ocm.absolute_z_axis_tolerance = 0.01;
  // ocm.weight = 1.0;

  // moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group_arm.setPathConstraints(test_constraints);


  // target_pose = current_pose;
  // target_pose.position.z -= 0.2;
  // target_pose.position.y += 0.4;
  // move_group_arm.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  // move_group_arm.setPlanningTime(30.0);


  // success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing constraints %s", success ? "SUCCESS" : "FAILED");

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishAxisLabeled(current_pose, "Start");
  // visual_tools.publishAxisLabeled(target_pose, "Goal");
  // visual_tools.publishText(text_pose, "Constrained goal", rvt::WHITE, rvt::XXLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("next");  
  // move_group_arm.execute(my_plan);
  // visual_tools.prompt("next");
  // move_group_arm.clearPathConstraints();


  /*
    F: Command (plan and optionally execute) the robot to move along a trajectory defined by a set of waypoints
  */

  std::vector<geometry_msgs::Pose> waypoints;

  // current_pose = move_group_arm.getCurrentPose().pose;
  // waypoints.push_back(current_pose);

  // target_pose = current_pose;
  // target_pose.position.z -= 0.1;
  // waypoints.push_back(target_pose);

  // target_pose.position.y -= 0.1;
  // waypoints.push_back(target_pose);

  // target_pose.position.z += 0.1;
  // waypoints.push_back(target_pose);

  // target_pose.position.y += 0.1;
  // waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // move_group_arm.execute(trajectory);
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  /*
    G: Command (plan and optionally execute) the robot to move along a trajectory (side length 0.2 m) parallel to
    X-Y world plane and with Z coordinate 0.3
  */

  // moveit_msgs::OrientationConstraint ocm;
  // ocm.header.frame_id = move_group_arm.getPlanningFrame().c_str();
  // ocm.link_name = move_group_arm.getEndEffectorLink().c_str();
  // ocm.orientation.x = 0.0;
  // ocm.orientation.x = 0.2;
  // ocm.orientation.w = 0.2;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group_arm.setPathConstraints(test_constraints);

  current_pose = move_group_arm.getCurrentPose().pose;

  tf2::Quaternion q;

  q.setRPY(M_PI,0,M_PI);


  geometry_msgs::Pose a_vertex = current_pose;
  a_vertex.position.x = 0.05;
  a_vertex.position.y = -0.4;
  a_vertex.position.z = 0.15;
  a_vertex.orientation.x = q.x();
  a_vertex.orientation.y = q.y();
  a_vertex.orientation.z = q.z();
  a_vertex.orientation.w = q.w();



  geometry_msgs::Pose b_vertex = current_pose;
  b_vertex.position.x = 0.1;
  b_vertex.position.y = -0.4;
  b_vertex.position.z = 0.15;
  b_vertex.orientation.x = q.x();
  b_vertex.orientation.y = q.y();
  b_vertex.orientation.z = q.z();
  b_vertex.orientation.w = q.w();


  geometry_msgs::Pose c_vertex = current_pose;
  c_vertex.position.x = 0.1;
  c_vertex.position.y = -0.45;
  c_vertex.position.z = 0.15;
  c_vertex.orientation.x = q.x();
  c_vertex.orientation.y = q.y();
  c_vertex.orientation.z = q.z();
  c_vertex.orientation.w = q.w();



  geometry_msgs::Pose d_vertex = current_pose;
  d_vertex.position.x = 0.05;
  d_vertex.position.y = -0.45;
  d_vertex.position.z = 0.15;
  d_vertex.orientation.x = q.x();
  d_vertex.orientation.y = q.y();
  d_vertex.orientation.z = q.z();
  d_vertex.orientation.w = q.w();


  move_group_arm.setPoseTarget(a_vertex);
  move_group_arm.move();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  waypoints.clear();
  waypoints.push_back(a_vertex);
  waypoints.push_back(b_vertex);
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
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_arm.execute(trajectory);
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /*
    H: Add a box object into the workspace of the UR5 manipulator,
    you could choose the dimensions and position , but you should
    command the robot to move in a goal pose or joint state that is
    showing that the trajectory is avoiding the collision with the box
  */

  //  move_group_arm.setStartState(*move_group_arm.getCurrentState());

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();

  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group_arm.getPlanningFrame();

  // collision_object.id = "box1";

  // // Define a box to add to the world.
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 1.0;
  // primitive.dimensions[primitive.BOX_Y] = 0.1;
  // primitive.dimensions[primitive.BOX_Z] = 0.4;

  // // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.0;
  // box_pose.position.y = 0.3;
  // box_pose.position.z = 0.0;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);

  // // Now, let's add the collision object into the world
  // // (using a vector that could contain additional objects)
  // ROS_INFO_NAMED("tutorial", "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // geometry_msgs::Pose another_pose;
  // another_pose.orientation.x = 1.0;
  // another_pose.position.x = 0.0;
  // another_pose.position.y = 0.6;
  // another_pose.position.z = 0.1;
  // move_group_arm.setPoseTarget(another_pose);

  // success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // // Attaching objects to the robot
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // You can attach objects to the robot, so that it moves with the robot geometry.
  // // This simulates picking up the object for the purpose of manipulating it.
  // // The motion planning should avoid collisions between the two objects as well.
  // moveit_msgs::CollisionObject object_to_attach;
  // object_to_attach.id = "cylinder1";

  // shape_msgs::SolidPrimitive cylinder_primitive;
  // cylinder_primitive.type = primitive.CYLINDER;
  // cylinder_primitive.dimensions.resize(2);
  // cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.3;
  // cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.05;

  // // We define the frame/pose for this cylinder so that it appears in the gripper
  // object_to_attach.header.frame_id = move_group_arm.getEndEffectorLink();
  // geometry_msgs::Pose grab_pose;
  // grab_pose.orientation.x = 0.7317;
  // grab_pose.orientation.w = 0.6816;
  // grab_pose.position.z = 0.15;

  // // First, we add the object to the world (without using a vector)
  // object_to_attach.primitives.push_back(cylinder_primitive);
  // object_to_attach.primitive_poses.push_back(grab_pose);
  // object_to_attach.operation = object_to_attach.ADD;
  // planning_scene_interface.applyCollisionObject(object_to_attach);

  // // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  // ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  // move_group_arm.attachObject(object_to_attach.id, "robot_tool0");

  // visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to receive and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // // Replan, but now with the object in hand.
  // move_group_arm.setStartStateToCurrentState();
  // success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // // The result may look something like this:
  // //
  // // .. image:: ./move_group_arm_tutorial_attached_object.gif
  // //    :alt: animation showing the arm moving differently once the object is attached
  // //
  // // Detaching and Removing Objects
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Now, let's detach the cylinder from the robot's gripper.
  // ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  // move_group_arm.detachObject(object_to_attach.id);

  // // Show text in RViz of status
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to receive and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  // // Now, let's remove the objects from the world.
  // ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  // std::vector<std::string> object_ids;
  // object_ids.push_back(collision_object.id);
  // object_ids.push_back(object_to_attach.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);

  // // Show text in RViz of status
  // visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to receive and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

  ros::shutdown();
  return 0;
}

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


float x_pos_obj = 0.25,y_pos_obj = -0.4,z_pos_obj=0.05;
float x_pos_final = 0.3,y_pos_final = -0.4 ,z_pos_final=0.25;


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_top_to_finger1_joint";
  posture.joint_names[1] = "gripper_top_to_finger2_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_top_to_finger1_joint";
  posture.joint_names[1] = "gripper_top_to_finger2_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.01;
  posture.points[0].positions[1] = 0.01;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // pick1
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "robot_base_link";
  tf2::Quaternion orientation;
  //tf2::Quaternion orientation(0,0,0,1);

  orientation.setEuler(-M_PI,0,M_PI/2);
  
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = x_pos_obj;
  grasps[0].grasp_pose.pose.position.y = y_pos_obj;
  grasps[0].grasp_pose.pose.position.z = z_pos_obj+0.095;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "robot_base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "robot_base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.095;
  grasps[0].post_grasp_retreat.desired_distance = 0.115;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("env_conveyor_to_visual_inspection");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "robot_base_link";
  tf2::Quaternion orientation;

  orientation.setEuler(0,0,M_PI/2);
  
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = x_pos_obj;
  place_location[0].place_pose.pose.position.y = y_pos_obj;
  place_location[0].place_pose.pose.position.z = z_pos_obj+0.08;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "robot_base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "robot_base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.x = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.095;
  place_location[0].post_place_retreat.desired_distance = 0.115;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("env_conveyor_to_visual_inspection");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);
  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = "robot_base_link";
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.05;
  collision_objects[0].primitives[0].dimensions[1] = 0.025;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = x_pos_obj;
  collision_objects[0].primitive_poses[0].position.y = y_pos_obj;
  collision_objects[0].primitive_poses[0].position.z = z_pos_obj;
  // collision_objects[0].primitive_poses[0].position.x = x_pos_final;
  // collision_objects[0].primitive_poses[0].position.y = y_pos_final;
  // collision_objects[0].primitive_poses[0].position.z = z_pos_final;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].APPEND;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}


void inspection(){

    namespace rvt = rviz_visual_tools;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    static const std::string PLANNING_GROUP_ARM = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);
    const moveit::core::JointModelGroup* joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> waypoints; 
    geometry_msgs::Pose current_pose;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = move_group_arm.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group_arm.setJointValueTarget(joint_group_positions);
    auto success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("HomeworkB2", "Returning to home position: %s", success ? "SUCCESS" : "FAILED");
    move_group_arm.execute(my_plan);
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();



    current_pose = move_group_arm.getCurrentPose().pose;

    tf2::Quaternion q;

    q.setEuler(-M_PI,0,M_PI/2);


    geometry_msgs::Pose a_vertex = current_pose;
    a_vertex.position.x = x_pos_final;
    a_vertex.position.y = y_pos_final;
    a_vertex.position.z = z_pos_final;
    a_vertex.orientation.x = q.x();
    a_vertex.orientation.y = q.y();
    a_vertex.orientation.z = q.z();
    a_vertex.orientation.w = q.w();

    q.setRPY(M_PI/2,0,M_PI/2);


    geometry_msgs::Pose b_vertex = current_pose;
    b_vertex.position.x = x_pos_final;
    b_vertex.position.y = y_pos_final;
    b_vertex.position.z = z_pos_final;
    b_vertex.orientation.x = q.x();
    b_vertex.orientation.y = q.y();
    b_vertex.orientation.z = q.z();
    b_vertex.orientation.w = q.w();

    


    move_group_arm.setPoseTarget(a_vertex);
    move_group_arm.move();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    waypoints.clear();
    waypoints.push_back(a_vertex);
    waypoints.push_back(b_vertex);

    // waypoints.push_back(e_vertex);




    double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group_arm.execute(trajectory);


    



    current_state = move_group_arm.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    for(int i=0;i<6;i++)
    {
        //joint_group_positions[i] = real_joint_state_vect[i];
        ROS_INFO("Joint %i : %.3f\n",i,joint_group_positions[i]);

    }   
    
    joint_group_positions[5] = M_PI;
    move_group_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("HomeworkB2", "Upper inspection: %s", success ? "SUCCESS" : "FAILED");
    move_group_arm.execute(my_plan);


    ros::WallDuration(1.0).sleep();

    
    joint_group_positions[5] = -M_PI;
    move_group_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("HomeworkB2", "Lower inspection: %s", success ? "SUCCESS" : "FAILED");
    move_group_arm.execute(my_plan);


    ros::WallDuration(1.0).sleep();
    
    joint_group_positions[2] = 1.3109108405586278;
    joint_group_positions[1] = -1.6298729451988205;
    joint_group_positions[0] = -2.881820193359532;
    joint_group_positions[3] =  0.07078617730403819;
    joint_group_positions[4] =  1.7008332839692943;
    joint_group_positions[5] =  1.4926065732611808 ;

 
    move_group_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("HomeworkB2", "Back to position before-place: %s", success ? "SUCCESS" : "FAILED");
    move_group_arm.execute(my_plan);


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setPlanningTime(30.0);
    std::cout << "stato : " << group.getPlannerId();
    addCollisionObjects(planning_scene_interface);
    ROS_INFO("Before pick");
    // Wait a bit for ROS things to initialize




    ros::WallDuration(1.0).sleep();


    ROS_INFO("Picking...");
    pick(group);
    ROS_INFO("After pick");

    ros::WallDuration(1.0).sleep();

    ROS_INFO("inspection...");

        inspection();

    ros::WallDuration(1.0).sleep();
    ROS_INFO("Placing...");

    place(group);

    ROS_INFO("Shutting down the whole program...");



    return 0;
}
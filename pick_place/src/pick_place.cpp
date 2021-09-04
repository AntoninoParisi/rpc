#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


float x_pos_obj = 0.2075,y_pos_obj = -0.3575,z_pos_obj=0.05;
float x_pos_final = -0.5,y_pos_final = 0 ,z_pos_final=0.06;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  //  open_gripper
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
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // closed_gripper
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
}



  void go_to_pick_place(moveit::planning_interface::MoveGroupInterface &group){



    group.setPlannerId("RRTConnect");


    tf2::Quaternion q;

    q.setRPY(M_PI,0,M_PI);

    geometry_msgs::Pose current_pose;

    current_pose = group.getCurrentPose().pose;

    // std::cout << current_pose << std::endl;



    geometry_msgs::Pose a_vertex = current_pose;


    a_vertex.position.x = -0.266252;
    a_vertex.position.y =  -0.473848;
    a_vertex.position.z =  0.247984;
    a_vertex.orientation.x =  0.699999;
    a_vertex.orientation.y =-0.71396;
    a_vertex.orientation.z =  0.0161121;
    a_vertex.orientation.w = 0.00200029;


    group.setPoseTarget(a_vertex);
    group.move();





   
     a_vertex.position.x =0.255531;
    a_vertex.position.y = -0.509722;
    a_vertex.position.z =  0.155057;
    a_vertex.orientation.x =   -0.705224;
    a_vertex.orientation.y =-0.708933;
    a_vertex.orientation.z = -0.00825431;
    a_vertex.orientation.w = 0.00211024;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(3.0).sleep();





  }


  void go_to_release_place(moveit::planning_interface::MoveGroupInterface &group){



    group.setPlannerId("RRTConnect");


    tf2::Quaternion q;

    q.setRPY(M_PI,0,M_PI);

    geometry_msgs::Pose current_pose;

    current_pose = group.getCurrentPose().pose;

    std::cout << current_pose << std::endl;



    geometry_msgs::Pose a_vertex = current_pose;

    a_vertex.position.x = -0.266252;
    a_vertex.position.y =  -0.473848;
    a_vertex.position.z =  0.247984;
    a_vertex.orientation.x =  0.699999;
    a_vertex.orientation.y =-0.71396;
    a_vertex.orientation.z =  0.0161121;
    a_vertex.orientation.w = 0.00200029;


    group.setPoseTarget(a_vertex);
    group.move();





    a_vertex.position.x = -0.435426;
    a_vertex.position.y =  -0.109913;
    a_vertex.position.z =  0.153226;
    a_vertex.orientation.x =  0.676498;
    a_vertex.orientation.y =0.736243;
    a_vertex.orientation.z = -0.00119716;
    a_vertex.orientation.w =0.0171676;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(3.0).sleep();





  }


void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // pick1
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "robot_base_link";
  tf2::Quaternion orientation(-0.705224,-0.708933,-0.00825431,0.00211024);

  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = x_pos_obj;
  grasps[0].grasp_pose.pose.position.y = y_pos_obj;
  grasps[0].grasp_pose.pose.position.z = z_pos_obj+0.095;


  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "robot_base_link";
  /* Direction is set as negative x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;


  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "robot_base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.095;
  grasps[0].post_grasp_retreat.desired_distance = 0.115;


  openGripper(grasps[0].pre_grasp_posture);



  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("env_conveyor_to_visual_inspection");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
 
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

 
  place_location[0].place_pose.header.frame_id = "robot_base_link";
  tf2::Quaternion orientation(0.676498,0.736243,-0.00119716,0.0171676);



  
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = x_pos_final;
  place_location[0].place_pose.pose.position.y = y_pos_final;
  place_location[0].place_pose.pose.position.z = z_pos_final+0.095;

  
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "robot_base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;


  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "robot_base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.095;
  place_location[0].post_place_retreat.desired_distance = 0.115;


  openGripper(place_location[0].post_place_posture);

  group.setSupportSurfaceName("env_conveyor_base");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

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


  collision_objects[0].operation = collision_objects[0].APPEND;

  planning_scene_interface.applyCollisionObjects(collision_objects);
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
  group.setPlanningTime(5.0);
  group.setPlannerId("RRTConnect");

  std::cout << "stato : " << group.getPlannerId();
  addCollisionObjects(planning_scene_interface);

  // go to position A near the end of the small conveyor belt
  




  ROS_INFO("Before pick");

  go_to_pick_place(group);


  ROS_INFO_NAMED("GlueSealing", "Available Planning Groups:");
  std::copy(group.getJointModelGroupNames().begin(),
            group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  ROS_INFO("Picking...");
  pick(group);
  ROS_INFO("After pick");

    go_to_release_place(group);


  ros::WallDuration(1.0).sleep();
  ROS_INFO("Placing...");

  place(group);

  ROS_INFO("Shutting down the whole program...");


  
  return 0;
}
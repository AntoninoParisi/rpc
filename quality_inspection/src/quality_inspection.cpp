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
#include <ur_msgs/SetIO.h>



float x_pos_obj = 0.2075,y_pos_obj = -0.3575,z_pos_obj=0.05;
float x_pos_final = -0.5,y_pos_final = 0 ,z_pos_final=0.06;
ur_msgs::SetIO msgs_service;

void openGripper()
{
  // //  open_gripper
  // /* Add both finger joints of panda robot. */
  // posture.joint_names.resize(2);
  // posture.joint_names[0] = "gripper_top_to_finger1_joint";
  // posture.joint_names[1] = "gripper_top_to_finger2_joint";

  // /* Set them as open, wide enough for the object to fit. */
  // posture.points.resize(1);
  // posture.points[0].positions.resize(2);
  // posture.points[0].positions[0] = 0.00;
  // posture.points[0].positions[1] = 0.00;
  // posture.points[0].time_from_start = ros::Duration(0.5);
  ros::NodeHandle nh;
  ros::ServiceClient io_client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");


  msgs_service.request.fun = 1;
  msgs_service.request.pin = 16;
  msgs_service.request.state = 1;
  io_client.call(msgs_service);
}

void closedGripper()
{
  // closed_gripper
  /* Add both finger joints of panda robot. */
  // posture.joint_names.resize(2);
  // posture.joint_names[0] = "gripper_top_to_finger1_joint";
  // posture.joint_names[1] = "gripper_top_to_finger2_joint";

  // /* Set them as closed. */
  // posture.points.resize(1);
  // posture.points[0].positions.resize(2);
  // posture.points[0].positions[0] = 0.01;
  // posture.points[0].positions[1] = 0.01;
  // posture.points[0].time_from_start = ros::Duration(0.5);


  ros::NodeHandle nh;
  ros::ServiceClient io_client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  msgs_service.request.fun = 1;
  msgs_service.request.pin = 16;
  msgs_service.request.state = 0;
  io_client.call(msgs_service);
}


void go_to_pick_place(moveit::planning_interface::MoveGroupInterface &group){



    group.setPlannerId("RRTConnect");

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



    ros::WallDuration(10.0).sleep();


   // posizione corretta pick
     a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.246657;
    a_vertex.orientation.x =   -0.705224;
    a_vertex.orientation.y = -0.708933;
    a_vertex.orientation.z = -0.00825431;
    a_vertex.orientation.w = 0.00211024;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(10.0).sleep();





  }

void pick(moveit::planning_interface::MoveGroupInterface& group)
{
  group.setPlannerId("RRTConnect");

    geometry_msgs::Pose current_pose;

    current_pose = group.getCurrentPose().pose;

    std::cout << current_pose << std::endl;


    // opening gripper

    openGripper();

    ros::WallDuration(2.0).sleep();



    geometry_msgs::Pose a_vertex = current_pose;

    a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.186657;
    a_vertex.orientation.x =   -0.705224;
    a_vertex.orientation.y = -0.708933;
    a_vertex.orientation.z = -0.00825431;
    a_vertex.orientation.w = 0.00211024;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(10.0).sleep();

  

    closedGripper();
    ros::WallDuration(2.0).sleep();

   a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.246657;
    a_vertex.orientation.x =   -0.705224;
    a_vertex.orientation.y = -0.708933;
    a_vertex.orientation.z = -0.00825431;
    a_vertex.orientation.w = 0.00211024;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(10.0).sleep();

}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  group.setPlannerId("RRTConnect");

    geometry_msgs::Pose current_pose;

    current_pose = group.getCurrentPose().pose;

    std::cout << current_pose << std::endl;





    geometry_msgs::Pose a_vertex = current_pose;

    a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.246657;
    a_vertex.orientation.x =   -0.705224;
    a_vertex.orientation.y = -0.708933;
    a_vertex.orientation.z = -0.00825431;
    a_vertex.orientation.w = 0.00211024;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(10.0).sleep();

  

    closedGripper();
    ros::WallDuration(2.0).sleep();

   a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.18667;
    a_vertex.orientation.x =   -0.705224;
    a_vertex.orientation.y = -0.708933;
    a_vertex.orientation.z = -0.00825431;
    a_vertex.orientation.w = 0.00211024;


    group.setPoseTarget(a_vertex);
    group.move();

    ros::WallDuration(10.0).sleep();


    // opening gripper

    openGripper();

    ros::WallDuration(2.0).sleep();
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


void inspection(moveit::planning_interface::MoveGroupInterface &group){

    
    group.setPlannerId("RRTConnect");


    tf2::Quaternion q;

    q.setRPY(M_PI,0,M_PI);

    geometry_msgs::Pose current_pose;

    current_pose = group.getCurrentPose().pose;

    std::cout << current_pose << std::endl;



    

    //rotation of wrist 1 & 2 

    geometry_msgs::Pose a_vertex = current_pose;

    a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.246657;
    a_vertex.orientation.x =  0.486816;
    a_vertex.orientation.y =0.510752;
    a_vertex.orientation.z =  0.50263;
    a_vertex.orientation.w = 0.499505;


    group.setPoseTarget(a_vertex);
    group.move();




    // rotation of wrist 3 in position B




    a_vertex.position.x = 0.259531;
    a_vertex.position.y = -0.519722;
    a_vertex.position.z =  0.246657;
    a_vertex.orientation.x =  -0.522272;
    a_vertex.orientation.y = 0.473829;
    a_vertex.orientation.z =  -0.511977;
    a_vertex.orientation.w = 0.490508;


    group.setPoseTarget(a_vertex);
    group.move();


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
    //addCollisionObjects(planning_scene_interface);
    ROS_INFO("Before pick");
    // Wait a bit for ROS things to initialize



    go_to_pick_place(group);


    ros::WallDuration(1.0).sleep();


    ROS_INFO("Picking...");
    pick(group);
    ROS_INFO("After pick");

    ros::WallDuration(1.0).sleep();

    ROS_INFO("inspection...");

    inspection(group);

    // ros::WallDuration(1.0).sleep();
    // ROS_INFO("Placing...");

    place(group);

    // ROS_INFO("Shutting down the whole program...");



    return 0;
}
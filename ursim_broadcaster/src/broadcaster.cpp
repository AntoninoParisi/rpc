#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sstream>

ros::Publisher pub;

void chatterCallback(const sensor_msgs::JointState & msg)
{




for(int i = 0; i<6 ; i++)
    std::cout <<  msg.position[i] << ' ';
std::cout << std::endl;

    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);
    traj.points.resize(1);

traj.joint_names = {"robot_elbow_joint", "robot_shoulder_lift_joint", "robot_shoulder_pan_joint", "robot_wrist_1_joint",
"robot_wrist_2_joint", "robot_wrist_3_joint"};

  for(int i = 0; i<6 ; i++)
    point.positions[i] = msg.position[i];

    point.time_from_start = ros::Duration(3);

    traj.points[0] = point;

    pub.publish(traj);
    std::cout << "pubblicato" << std::endl;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ursim_broadcaster");
  ros::NodeHandle nh;
  ros::Rate lr(10);
  ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states", 1000,chatterCallback);
  pub = nh.advertise<trajectory_msgs::JointTrajectory>("scaled_pos_joint_traj_controller/command",1000);
  while(ros::ok()){
    ros::spinOnce();
    lr.sleep();
};
}
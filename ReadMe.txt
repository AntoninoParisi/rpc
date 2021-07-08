#################################

ROS DISTRO -> MELODIC
UBUNTU DISTRO -> 18.04


AUTHORS : Edoardo Fiorini, Antonino Parisi
Date : 8-7-2021
#################################




#################################

commands:

    # open the rviz simulation : (visualizer and planning group)
        roslaunch ur5e_2f_moveit rviz_complete_bringup.launch
    # launch pick and place
        rosrun pick_place pick_place_node
    # launch planner
        rosrun planner_rpc planner_rpc_node 

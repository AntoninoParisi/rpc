# RPC PROJECT
## Implementation of ur5e through ursim and moveit


## Features

- Comunication with ursim and moveit
- Glue sealing, pick and place, quality inspection
- Ice-Lab environment


## Dependencies


- ROS - Melodic!
- Ubuntu - version : 18.4

## Installation



To install the dependencies check out the package.xml.

```sh
mkdir your_ros_ws
cd your_ros_ws
git clone https://github.com/AntoninoParisi/rpc.git
# rename rpc to src (do it with gui interface)

cd src
catkin_init_workspace
cd ..
# remember to source the main setup of your ros setup
catkin_make

# open the rviz simulation : (visualizer and planning group)
roslaunch ur5e_2f_moveit rviz_complete_bringup.launch
# launch pick and place
rosrun pick_place pick_place_node
# launch planner
rosrun planner_rpc planner_rpc_node




# calibration command
roslaunch ur_calibration calibration_correction.launch robot_ip:=127.0.0.1 target_filename:="$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml"
# launch ursim with calibration
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=127.0.0.1 kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml


# to enable the comunication with ursim must start ursim_broadcaster
rosrun ursim_broadcaster ursim_broadcaster

```

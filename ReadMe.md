<h1 class="code-line" data-line-start=0 data-line-end=1 ><a id="ROBOT_PROGRAMMING_AND_CONTROL_FINAL_PROJECT__MODULE_B_0"></a>ROBOT PROGRAMMING AND CONTROL FINAL PROJECT - MODULE B</h1>
<h2 class="code-line" data-line-start=2 data-line-end=3 ><a id="_High_level_control_of_UR5e_with_ROS_and_MoveIt_both_in_simulation_and_in_reality__2"></a><em>High level control of UR5e with ROS and MoveIt both in simulation and in reality</em></h2>
<p class="has-line-data" data-line-start="4" data-line-end="5">The purpose of the project is to programming a flexible industrial cell based on ROS. In particular, it refers to the UR5e robot mounted in the production line of the <a href="https://www.icelab.di.univr.it/">ICELab</a> at the University of Verona. This workspace allows to control the robot both in simulation and in reality. The trajectory are planned with <a href="https://moveit.ros.org/">MoveIt</a>, instead the comunication with the real robot is given by the <a href="https://github.com/UniversalRobots/Universal_Robots_ROS_Driver">Universal_Robot_ROS_Driver</a>. The key points to address are written in following:</p>
<ul>
<li class="has-line-data" data-line-start="6" data-line-end="7">Geometric modeling of a UR5e in a manufacturing cell</li>
<li class="has-line-data" data-line-start="7" data-line-end="8">Object Pick and place</li>
<li class="has-line-data" data-line-start="8" data-line-end="9">Glue sealing (following a cartesian path with trajectory constraints in orientation and speed)</li>
<li class="has-line-data" data-line-start="9" data-line-end="10">Quality inspection(pick an object and show it with two different angles to a camera)</li>
<li class="has-line-data" data-line-start="10" data-line-end="12">Communication with the simulator(<a href="https://www.universal-robots.com/download/?query=">URsim</a>) and the real robot</li>
</ul>
<h2 class="code-line" data-line-start=12 data-line-end=13 ><a id="Features_12"></a>Features</h2>
<p class="has-line-data" data-line-start="14" data-line-end="15">In following the most important features of the proposed work:</p>
<ul>
<li class="has-line-data" data-line-start="16" data-line-end="17">Design the model of gripper_mlp3240 attached on the robot starting from CAD to guarantee more reliability during the task. Including the flange between the wrist 3 and the gripper.</li>
<li class="has-line-data" data-line-start="17" data-line-end="18">ICELab environmnet reconstruction(mesh and parameters)</li>
<li class="has-line-data" data-line-start="18" data-line-end="19">URSim/Robot and MoveIt comunication through a broadcast node</li>
<li class="has-line-data" data-line-start="19" data-line-end="21">The tasks parameters are defined  depending on real world measure</li>
</ul>
<h2 class="code-line" data-line-start=21 data-line-end=22 ><a id="Acknowledgment_21"></a>Acknowledgment</h2>
<p class="has-line-data" data-line-start="22" data-line-end="23">This work has been devoleped by Antonino Parisi and Edoardo Fiorini at the University of Verona.</p>
<h2 class="code-line" data-line-start=24 data-line-end=25 ><a id="UR5e_model_24"></a>UR5e model</h2>
<p class="has-line-data" data-line-start="26" data-line-end="27">To model the UR5e robot mounted in the production line has been used the already existing URDF file from the official <a href="https://github.com/ros-industrial/universal_robot">repository</a>. In this package there are also all the planning group which allows us to design our trajectory with moveIt.</p>
<h2 class="code-line" data-line-start=27 data-line-end=28 ><a id="Gripper_mlp3240_27"></a>Gripper mlp3240</h2>
<p class="has-line-data" data-line-start="29" data-line-end="30">To ensure the best reliability we have developed the gripper model from scratch. We started from the CAD models from which we extracted the meshes through blender. A bit of post production was needed to get a good resolution. After that we extracted the complete meshes and assembled them inside the urdf. In particular, to see these stuff you have to open the fmauch_universal_robot/ur_description folder. Then, in meshes there are all the 3D shape, instead in urdf there are all the xacro file. Of course, it was necessary to define the links between the ur5e + gripper xacro file and environment.</p>
<h2 class="code-line" data-line-start=30 data-line-end=31 ><a id="Environment_30"></a>Environment</h2>
<p class="has-line-data" data-line-start="31" data-line-end="34">The production line of the ICELab has been reconstructed in minimal details to avoid gap between the simulation and the real task. To use URSim it is necessary to have all the real parameters regarding the conveyors belt and the support of UR5e. Only in this way what happens in the simulator is exactly what will expect in reality. In following is shown respectively the prodcuction line plant and the geometric model:<br>
<img src="image/modelloIceLab.png?raw=true" alt="Alt text" title="Production Line"><br> 
<img src="image/photo5861574527685867334.jpg?raw=true" alt="Alt text" title="Geometric Model"></p>
<h2 class="code-line" data-line-start=36 data-line-end=37 ><a id="Requirements_36"></a>Requirements</h2>
<p class="has-line-data" data-line-start="38" data-line-end="39">This workspace requires a system setup with ROS. It is recommended to use:</p>
<ul>
<li class="has-line-data" data-line-start="40" data-line-end="41">Ubuntu 18.04 LTS</li>
<li class="has-line-data" data-line-start="41" data-line-end="43">ROS melodic</li>
</ul>
<p class="has-line-data" data-line-start="43" data-line-end="44">To make sure that robot control is not affected by system latencies, it should be better to use a real-time kernel with the system. See the <a href="https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md">real-time setup</a> guide.</p>
<h2 class="code-line" data-line-start=45 data-line-end=46 ><a id="Installation_45"></a>Installation</h2>
<p class="has-line-data" data-line-start="47" data-line-end="48">First of all you have to install the URSim simulator from the Universal Robot <a href="https://www.universal-robots.com/download/software-e-series/simulator-linux/offline-simulator-e-series-ur-sim-for-linux-5110/">official page</a>. It is recommended to install the simualtor before ROS to avoid problems with dependencies.</p>
<p class="has-line-data" data-line-start="49" data-line-end="50">After that you are ready to install the project in your ROS workspace. Open a terminal and write the following commands:</p>
<pre><code class="has-line-data" data-line-start="52" data-line-end="67">cd $HOME/your_ros_ws

git clone https://github.com/AntoninoParisi/rpc.git

#rename rpc to src (do it with gui interface)
cd $HOME/your_ros_ws/src
catkin_init_workspace
cd ..

#building
catkin_make

#activate this workspace
source $HOME/your_ros_ws/devel/setup.bash
</code></pre>
<p class="has-line-data" data-line-start="68" data-line-end="69">There could be some problems with ROS-packages or dependencies. Check out the package.xml and install all the package that the catkin_make needs.</p>
<h2 class="code-line" data-line-start=70 data-line-end=71 ><a id="Usage_in_simulation_with_URSim_70"></a>Usage in simulation with URSim</h2>
<p class="has-line-data" data-line-start="72" data-line-end="73">To run the project in simulation you have to launch some commands, all of these have to be launch from different terminals and before run the command you have to activate your workspace with:</p>
<pre><code class="has-line-data" data-line-start="75" data-line-end="77">source $HOME/your_ros_ws/devel/setup.bash
</code></pre>
<p class="has-line-data" data-line-start="78" data-line-end="79">The steps are the following one:</p>
<ol>
<li class="has-line-data" data-line-start="80" data-line-end="84">
<p class="has-line-data" data-line-start="80" data-line-end="81">Open the rviz simulation (visualizer and planning group):</p>
<p class="has-line-data" data-line-start="82" data-line-end="83"><code>roslaunch ur5e_mlp3240_env demo.launch</code></p>
</li>
<li class="has-line-data" data-line-start="84" data-line-end="88">
<p class="has-line-data" data-line-start="84" data-line-end="85">Open the URSim simulator:</p>
<p class="has-line-data" data-line-start="86" data-line-end="87"><code>cd $HOME/your_path/ursim-5.X.X.XXXXX/start-ursim.sh</code></p>
</li>
</ol>
<p class="has-line-data" data-line-start="88" data-line-end="89">Now the URCap must be installed in the simulator, using the following <a href="https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md">guide</a></p>
<ol start="3">
<li class="has-line-data" data-line-start="89" data-line-end="92">
<p class="has-line-data" data-line-start="89" data-line-end="90">Extract the calibration of the robot:</p>
<p class="has-line-data" data-line-start="91" data-line-end="92"><code>roslaunch ur_calibration calibration_correction.launch robot_ip:=127.0.0.1 target_filename:=&quot;$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml&quot;</code></p>
</li>
<li class="has-line-data" data-line-start="92" data-line-end="95">
<p class="has-line-data" data-line-start="92" data-line-end="93">Run the ROS node that comunicates with URSim</p>
<p class="has-line-data" data-line-start="94" data-line-end="95"><code>roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=127.0.0.1 kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml</code></p>
</li>
<li class="has-line-data" data-line-start="95" data-line-end="99">
<p class="has-line-data" data-line-start="95" data-line-end="96">Run the broadcaster ROS node that send the MoveIt trajectory to URSim</p>
<p class="has-line-data" data-line-start="97" data-line-end="98"><code>rosrun ursim_broadcaster ursim_broadcaster</code></p>
</li>
<li class="has-line-data" data-line-start="99" data-line-end="100">
<p class="has-line-data" data-line-start="99" data-line-end="100">Run the task ROS node in base on what you want to execute:</p>
</li>
</ol>
<ul>
<li class="has-line-data" data-line-start="100" data-line-end="102">Pick and Place:<br>
<code>rosrun pick_place pick_place_node</code></li>
<li class="has-line-data" data-line-start="102" data-line-end="104">Glue Sealing:<br>
<code>rosrun glue_sealing glue_sealing_node</code></li>
<li class="has-line-data" data-line-start="104" data-line-end="107">Visual Inspection:<br>
<code>rosrun visual_inspection visual_inspection_node</code></li>
</ul>
<h2 class="code-line" data-line-start=107 data-line-end=108 ><a id="Usage_with_the_real_Robot_107"></a>Usage with the real Robot</h2>
<p class="has-line-data" data-line-start="109" data-line-end="110">To run the project using the real robot, first of all you have to switch on the robot from the teach pendant and you have to connect to the computer robot. Then you are ready to launch some commands, all of these have to be launch from different terminals and before run the command you have to activate your workspace with:</p>
<pre><code class="has-line-data" data-line-start="112" data-line-end="114">source $HOME/your_ros_ws/devel/setup.bash
</code></pre>
<p class="has-line-data" data-line-start="115" data-line-end="116">The steps are the following one:</p>
<ol>
<li class="has-line-data" data-line-start="117" data-line-end="121">
<p class="has-line-data" data-line-start="117" data-line-end="118">Open the rviz simulation (visualizer and planning group):</p>
<p class="has-line-data" data-line-start="119" data-line-end="120"><code>roslaunch ur5e_mlp3240_env demo.launch</code></p>
</li>
<li class="has-line-data" data-line-start="121" data-line-end="122">
<p class="has-line-data" data-line-start="121" data-line-end="122">Start the robot and open the program with the URCap on the teach pendant:</p>
</li>
</ol>
<p class="has-line-data" data-line-start="124" data-line-end="125">The URCap must be installed in the simulator, using the following <a href="https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md">guide</a></p>
<ol start="3">
<li class="has-line-data" data-line-start="125" data-line-end="128">
<p class="has-line-data" data-line-start="125" data-line-end="126">Extract the calibration of the robot:</p>
<p class="has-line-data" data-line-start="127" data-line-end="128"><code>roslaunch ur_calibration calibration_correction.launch robot_ip:=YOUR_ROBOT_IP target_filename:=&quot;$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml&quot;</code></p>
</li>
<li class="has-line-data" data-line-start="128" data-line-end="131">
<p class="has-line-data" data-line-start="128" data-line-end="129">Run the ROS node that comunicates with the real robot</p>
<p class="has-line-data" data-line-start="130" data-line-end="131"><code>roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=ROBOT_IP kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml</code></p>
</li>
<li class="has-line-data" data-line-start="131" data-line-end="135">
<p class="has-line-data" data-line-start="131" data-line-end="132">Run the broadcaster ROS node that send the MoveIt trajectory to the real robot</p>
<p class="has-line-data" data-line-start="133" data-line-end="134"><code>rosrun ursim_broadcaster ursim_broadcaster</code></p>
</li>
<li class="has-line-data" data-line-start="135" data-line-end="136">
<p class="has-line-data" data-line-start="135" data-line-end="136">Run the task ROS node in base on what you want to execute:</p>
</li>
</ol>
<ul>
<li class="has-line-data" data-line-start="136" data-line-end="138">Pick and Place:<br>
<code>rosrun pick_place pick_place_node</code></li>
<li class="has-line-data" data-line-start="138" data-line-end="140">Glue Sealing:<br>
<code>rosrun glue_sealing glue_sealing_node</code></li>
<li class="has-line-data" data-line-start="140" data-line-end="142">Visual Inspection:<br>
<code>rosrun visual_inspection visual_inspection_node</code></li>
</ul>

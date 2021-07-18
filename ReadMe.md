<h1 class="code-line" data-line-start=0 data-line-end=1 ><a id="ROBOT_PROGRAMMING_AND_CONTROL_FINAL_PROJECT__MODULE_B_0"></a>ROBOT PROGRAMMING AND CONTROL FINAL PROJECT - MODULE B</h1>
<h2 class="code-line" data-line-start=2 data-line-end=3 ><a id="_High_level_control_of_UR5e_with_ROS_MoveIt_and_URSim__2"></a><em>High level control of UR5e with ROS, MoveIt and URSim</em></h2>
<p class="has-line-data" data-line-start="4" data-line-end="5">The purpose of the project is to programming a flexible industrial cell based on ROS. In particular, it refers to the UR5e robot mounted in the production line of the <a href="https://www.icelab.di.univr.it/">ICELab</a> at the University of Verona. This workspace allows to control the robot both in simulation and in reality. The trajectory are planned with <a href="https://moveit.ros.org/">MoveIt</a>, instead the comunication with the real robot is given by <a href="https://www.universal-robots.com/download/?query=">URSim</a>. The key poiints are written in following:</p>
<ul>
<li class="has-line-data" data-line-start="6" data-line-end="7">Geometric modeling of a UR5e in a manufacturing cell</li>
<li class="has-line-data" data-line-start="7" data-line-end="8">Object Pick and place</li>
<li class="has-line-data" data-line-start="8" data-line-end="9">Glue sealing (following a cartesian path with trajectory constraints in orientation and speed)</li>
<li class="has-line-data" data-line-start="9" data-line-end="10">Quality inspection</li>
<li class="has-line-data" data-line-start="10" data-line-end="12">Comunication with the real robot</li>
</ul>
<h2 class="code-line" data-line-start=12 data-line-end=13 ><a id="Features_12"></a>Features</h2>
<p class="has-line-data" data-line-start="14" data-line-end="15">In following the most important feature to adress:</p>
<ul>
<li class="has-line-data" data-line-start="16" data-line-end="17">Design the model of gripper_mlp3240 attached on the robot starting from CAD to guarantee more reliability during the task.</li>
<li class="has-line-data" data-line-start="17" data-line-end="18">ICELab environmnet reconstruction(mesh and parameters)</li>
<li class="has-line-data" data-line-start="18" data-line-end="19">URSim and moveit comunication through a broadcast node</li>
<li class="has-line-data" data-line-start="19" data-line-end="21">Defined the tasks parameters depending on real world measure</li>
</ul>
<h2 class="code-line" data-line-start=21 data-line-end=22 ><a id="UR5e_model_21"></a>UR5e model</h2>
<p class="has-line-data" data-line-start="23" data-line-end="24">To model the UR5e robot mounted in the production line has been used the already existing URDF file from the official <a href="https://github.com/ros-industrial/universal_robot">repository</a>. In this package there are also all the planning group which allows us to design our trajectory with moveIt.</p>
<h2 class="code-line" data-line-start=24 data-line-end=25 ><a id="Gripper_mlp3240_24"></a>Gripper mlp3240</h2>
<p class="has-line-data" data-line-start="26" data-line-end="27">To ensure the best reliability we have developed the gripper model from scratch.</p>
<h2 class="code-line" data-line-start=27 data-line-end=28 ><a id="Environment_27"></a>Environment</h2>
<p class="has-line-data" data-line-start="28" data-line-end="31">The production line of the ICELab has been reconstructed in minimal details to avoid gap between the simulation and the real task. To use URSim it is necessary to have all the real parameters regarding the conveyors belt and the support of UR5e. Only in this way what happens in the simulator is exactly what will expect in reality. In following is shown respectively the prodcuction line plant and the geometric model:<br>
<img src="image/modelloIceLab.png?raw=true" alt="Alt text" title="Production Line"><br>
<img src="https://www.artemedialab.it/wp-content/uploads/2019/04/immagini-sfondo-1-700x400.jpg?aw=true" alt="Alt text" title="Geometric Model"></p>
<h2 class="code-line" data-line-start=33 data-line-end=34 ><a id="Requirements_33"></a>Requirements</h2>
<p class="has-line-data" data-line-start="35" data-line-end="36">This workspace requires a system setup with ROS. It is recommended to use:</p>
<ul>
<li class="has-line-data" data-line-start="37" data-line-end="38">Ubuntu 18.04 LTS</li>
<li class="has-line-data" data-line-start="38" data-line-end="40">ROS melodic</li>
</ul>
<p class="has-line-data" data-line-start="40" data-line-end="41">To make sure that robot control is not affected by system latencies, it should be better to use a real-time kernel with the system. See the <a href="https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md">real-time setup</a> guide.</p>
<h2 class="code-line" data-line-start=42 data-line-end=43 ><a id="Installation_42"></a>Installation</h2>
<p class="has-line-data" data-line-start="44" data-line-end="45">First of all you have to install the URSim simulator from the Universal Robot <a href="https://www.universal-robots.com/download/software-e-series/simulator-linux/offline-simulator-e-series-ur-sim-for-linux-5110/">official page</a>. It is recommended to install the simualtor before ROS to avoid problems with dependencies.</p>
<p class="has-line-data" data-line-start="46" data-line-end="47">After that you are ready to install the project in your ROS workspace. Open a terminal and write the following commands:</p>
<pre><code class="has-line-data" data-line-start="49" data-line-end="64">cd $HOME/your_ros_ws

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
<p class="has-line-data" data-line-start="65" data-line-end="66">There could be some problems with ROS-packages or dependencies. Check out the package.xml and install all the package that the catkin_make needs.</p>
<h2 class="code-line" data-line-start=67 data-line-end=68 ><a id="Usage_in_simulation_with_URSim_67"></a>Usage in simulation with URSim</h2>
<p class="has-line-data" data-line-start="69" data-line-end="70">To run the project in simulation you have to launch some commands, all of these have to be run from different terminals and before launch the command you have to activate your workspace with:</p>
<pre><code class="has-line-data" data-line-start="72" data-line-end="74">source $HOME/your_ros_ws/devel/setup.bash
</code></pre>
<p class="has-line-data" data-line-start="75" data-line-end="76">The steps are the following one:</p>
<ol>
<li class="has-line-data" data-line-start="77" data-line-end="81">
<p class="has-line-data" data-line-start="77" data-line-end="78">Open the rviz simulation (visualizer and planning group):&lt;br&gt;</p>
<p class="has-line-data" data-line-start="79" data-line-end="80"><code>roslaunch ur5e_2f_moveit rviz_complete_bringup.launch</code></p>
</li>
<li class="has-line-data" data-line-start="81" data-line-end="85">
<p class="has-line-data" data-line-start="81" data-line-end="82">Open the URSim simulator:&lt;br&gt;</p>
<p class="has-line-data" data-line-start="83" data-line-end="84"><code>cd $HOME/your_path/ursim-5.X.X.XXXXX/start-ursim.sh</code>&lt;br&gt;</p>
</li>
</ol>
<p class="has-line-data" data-line-start="85" data-line-end="86">Now the URCap must be installed in the simulator, using the following <a href="https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md">guide</a></p>
<ol start="3">
<li class="has-line-data" data-line-start="86" data-line-end="89">
<p class="has-line-data" data-line-start="86" data-line-end="87">Extract the calibration of the robot:&lt;br&gt;</p>
<p class="has-line-data" data-line-start="88" data-line-end="89"><code>roslaunch ur_calibration calibration_correction.launch robot_ip:=127.0.0.1 target_filename:=&quot;$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml&quot;</code></p>
</li>
<li class="has-line-data" data-line-start="89" data-line-end="92">
<p class="has-line-data" data-line-start="89" data-line-end="90">Run the ROS node that comunicates with URSim&lt;br&gt;</p>
<p class="has-line-data" data-line-start="91" data-line-end="92"><code>roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=127.0.0.1 kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml</code></p>
</li>
<li class="has-line-data" data-line-start="92" data-line-end="95">
<p class="has-line-data" data-line-start="92" data-line-end="93">Run the broadcaster ROS node that send the MoveIt trajectory to URSim&lt;br&gt;</p>
<p class="has-line-data" data-line-start="94" data-line-end="95"><code>rosrun ursim_broadcaster ursim_broadcaster</code></p>
</li>
<li class="has-line-data" data-line-start="95" data-line-end="98">
<p class="has-line-data" data-line-start="95" data-line-end="96">Run the ROS node that comunicates with URSim&lt;br&gt;</p>
<p class="has-line-data" data-line-start="97" data-line-end="98"><code>roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=127.0.0.1 kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml</code></p>
</li>
<li class="has-line-data" data-line-start="98" data-line-end="99">
<p class="has-line-data" data-line-start="98" data-line-end="99">Run the task ROS node in base on what you want to execute:</p>
</li>
</ol>
<ul>
<li class="has-line-data" data-line-start="99" data-line-end="101">Pick and Place:&lt;br&gt;<br>
<code>rosrun pick_place pick_place_node</code></li>
<li class="has-line-data" data-line-start="101" data-line-end="103">Glue Sealing:&lt;br&gt;<br>
<code>rosrun planner_rpc planner_rpc_node</code></li>
<li class="has-line-data" data-line-start="103" data-line-end="106">Visual Inspection:&lt;br&gt;<br>
<code>rosrun pick_place pick_place_node</code></li>
</ul>
<h2 class="code-line" data-line-start=106 data-line-end=107 ><a id="Usage_with_the_real_Robot_106"></a>Usage with the real Robot</h2>
<p class="has-line-data" data-line-start="108" data-line-end="109">To run the project in simulation you have to launch some commands, all of these have to be run from different terminals and before launch the command you have to activate your workspace with:</p>
<pre><code class="has-line-data" data-line-start="111" data-line-end="113">source $HOME/your_ros_ws/devel/setup.bash
</code></pre>
<p class="has-line-data" data-line-start="114" data-line-end="115">The steps are the following one:</p>






</code></pre>

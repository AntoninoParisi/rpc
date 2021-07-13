<h1 class="code-line" data-line-start=0 data-line-end=1 ><a id="RPC_PROJECT_0"></a>RPC PROJECT</h1>
<h2 class="code-line" data-line-start=1 data-line-end=2 ><a id="Implementation_of_ur5e_through_ursim_and_moveit_1"></a>Implementation of ur5e through ursim and moveit</h2>
<h2 class="code-line" data-line-start=4 data-line-end=5 ><a id="Features_4"></a>Features</h2>
<ul>
<li class="has-line-data" data-line-start="6" data-line-end="7">Comunication with ursim and moveit</li>
<li class="has-line-data" data-line-start="7" data-line-end="8">Glue sealing, pick and place, quality inspection</li>
<li class="has-line-data" data-line-start="8" data-line-end="9">Ice-Lab environment</li>
</ul>
<h2 class="code-line" data-line-start=11 data-line-end=12 ><a id="Dependencies_11"></a>Dependencies</h2>
<ul>
<li class="has-line-data" data-line-start="14" data-line-end="15">ROS - Melodic!</li>
<li class="has-line-data" data-line-start="15" data-line-end="17">Ubuntu - version : 18.4</li>
</ul>
<h2 class="code-line" data-line-start=17 data-line-end=18 ><a id="Installation_17"></a>Installation</h2>
<p class="has-line-data" data-line-start="21" data-line-end="22">To install the dependencies check out the package.xml.</p>
<pre><code class="has-line-data" data-line-start="24" data-line-end="55" class="language-sh">mkdir your_ros_ws
<span class="hljs-built_in">cd</span> your_ros_ws
git <span class="hljs-built_in">clone</span> https://github.com/AntoninoParisi/rpc.git
<span class="hljs-comment"># rename rpc to src (do it with gui interface)</span>

<span class="hljs-built_in">cd</span> src
catkin_init_workspace
<span class="hljs-built_in">cd</span> ..
<span class="hljs-comment"># remember to source the main setup of your ros setup</span>
catkin_make

<span class="hljs-comment"># open the rviz simulation : (visualizer and planning group)</span>
roslaunch ur5e_2f_moveit rviz_complete_bringup.launch
<span class="hljs-comment"># launch pick and place</span>
rosrun pick_place pick_place_node
<span class="hljs-comment"># launch planner</span>
rosrun planner_rpc planner_rpc_node




<span class="hljs-comment"># calibration command</span>
roslaunch ur_calibration calibration_correction.launch robot_ip:=<span class="hljs-number">127.0</span>.<span class="hljs-number">0.1</span> target_filename:=<span class="hljs-string">"<span class="hljs-variable">$(rospack find ur_calibration)</span>/etc/ex-ur5e_calibration.yaml"</span>
<span class="hljs-comment"># launch ursim with calibration</span>
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<span class="hljs-number">127.0</span>.<span class="hljs-number">0.1</span> kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml


<span class="hljs-comment"># to enable the comunication with ursim must start ursim_broadcaster</span>
rosrun ursim_broadcaster ursim_broadcaster

</code></pre>
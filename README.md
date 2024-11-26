# Multi-Robot Racing Simulator
This repository offers a multi-robot racing simulation environment built with ROS and Gazebo, designed to be used as testbed for autonomous navigation techniques. 
The environment enables users to deploy and evaluate navigation algorithms, particularly those using Deep Reinforcement Learning (DRL), in dynamic racing scenarios. 
With support for multiple robots, this simulator can be used for research and development in competitive autonomous racing, providing a ready-to-use setup that helps 
with testing and experimentation.


The DRL algorithm used in this repository corresponds to the SARL* modified to support multiple robots in Gazebo. This is done by separating agents with 
namespaces, resulting in independent entities. Also, a testbed for traditional navigation methods using the ROS Navigation Stack is presented.

To keep the ROS workspaces separated, the information about the setup and installation for agents using the traditional ROS Navigation Stack can be found
[here](https://github.com/mateopozor02/traditional-multirobot-env.git). 

## Contents   
1. [Build & Install](#build--install)   
2. [Training](#training)
3. [Multi-Robot Setup](#multi-robot-setup)
5. [Test the SARL* agents in Gazebo](#test-the-sarl-agents-in-gazebo) 
7. [Reference](#reference)

## Build & Install 
To setup the environment, ROS Melodic should be installed on your machine. For this reason, you must be running Ubuntu 18. 
1. Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Create your own ROS workspace and clone this repository.
```bash
mkdir -p ~/racing_ws/src
cd ~/racing_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/mateopozor02/multirobot-racing-env.git
```
3. Install the required dependencies
```bash
sudo apt install libbullet-dev
sudo apt install libsdl-image1.2-dev
sudo apt install libsdl-dev
sudo apt install ros-melodic-bfl ros-melodic-tf2-sensor-msgs
pip install empy
pip install configparser
sudo apt install cmake
```
4. Install [Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2)
```bash
cd ~/racing_ws/src/sarl_dif_env/Python-RVO2/
pip install Cython
python setup.py build
# Note: If the error 'subprocess.CalledProcessError: Command '['cmake', '--build', '.']' returned non-zero exit status 2' is generated, then delete the folder 'build' run 'python setup.py build' the second time
python setup.py install
```
5. Install [CrowdNav](https://github.com/vita-epfl/CrowdNav)
```bash
cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav
pip install -e .
```
Note: if the installation is killed at 99% while 'torch' is being installed, reboot and try again with below command.
```bash
pip install -e . --no-cache-dir
```
6. Install the necessary ROS Packages.
```bash
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
ros-melodic-rosserial-server ros-melodic-rosserial-client \
ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
ros-melodic-compressed-image-transport ros-melodic-rqt* \
ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
7. Finally, build the catkin workspace.
```bash
cd ~/racing_ws/
catkin_make
source devel/setup.bash
```

## Training
Three simulation environments were defined for training, each with its own geometry. Furthermore, the reward function and parameters are customizable.
The training parameters can be found in "env.config", "policy.config", "train.config" under "~/racing_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/".
![scenarios](https://github.com/user-attachments/assets/c072c91c-dc8c-41ad-a615-c548a5748789)
The different scenarios are defined in its own CrowdNav directory. In this project: *CrowdNav* (curve path), *CrowdNav-crosspath*, and *CrowdNav-nowall* are defined. To 
change the working scenario, change the name of the desired folder to *CrowdNav* and follow the normal workflow. 

1. Training the value network
```bash
cd ~/racing_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/
```
```bash
python train.py --policy sarl --output_dir data/your_model_name
```
This will train the value network with the parameters defined in `~/racing_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/`. After this, your 
model will be saved with the name given in the command line argument `--output_dir`.

2. Visualize a test case

To visualize a test case from the trained model, you can run the following commands:
```bash
cd ~/racing_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav
```
```bash
python test.py --policy sarl --model_dir data/your_model_name --phase test --visualize --test_case 0
```

This will generate a GIF with a testcase of your model in the training scenario.
<div align="center">
  <img src="https://github.com/user-attachments/assets/27c91a9f-6b67-48c9-a321-2456846ac8ab" width="400"/>
</div>

3. Plotting the training data

It is possible to obtain important data from the training process, such as accumulated reward or success rate across the training episodes.
This information can be found in the log file, under `~/racing_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/data` within the folder of your training.

To get the plots, execute the following commands in your terminal: 
```bash
cd ~/racing_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav
```
```bash
python utils/plot.py data/your_model_name/output.log --plot_sr
```

Then you will get the plots for success rate and accumulated reward across all training episodes: 
<div align="center">
  <img src="https://github.com/user-attachments/assets/b79aed1a-93bd-4c5a-a5ab-d84150566408" width="800"/>
</div>

## Multi-Robot Setup

In order to support multiple robots in a single environment, ROS namespaces were used to isolate each robot in execution. 
This results in a `rqt_graph` with multiple robots, and nodes running under namespaces, sharing only the global map in 
which they interact. 

<div align="center">
  <img src="https://github.com/user-attachments/assets/a64e7ae3-2bea-4f9e-9e51-b59661030570" width="1000"/>
</div>

This behavior is achieved by modifying the ROS launchfile for the environment. This file is located at: `~/racing_ws/src/sarl_dif_env/sarl_star_ros/launch/turtlebot3_sarl_star_world_track_multi1.launch.xml`

To include more agents, you just need to create a new namespace in the launchfile, copying the structure of the other namespaces defined in the file. These
namespaces already include all the necessary remappings to include new agents in the simulation. 

```xml
<!-- First Robot -->
   <group ns="$(arg sarl_tb3)">

      <!-- AMCL for First Robot -->
      <arg name="custom_amcl_launch_file" default="$(find sarl_star_ros)/launch/includes/amcl/amcl1.launch.xml"/>
      <include file="$(arg custom_amcl_launch_file)">
         <arg name="initial_pose_x" value="$(arg sarl_tb3_x_pos)"/>
         <arg name="initial_pose_y" value="$(arg sarl_tb3_y_pos)"/>
         <arg name="initial_pose_a" value="0"/>
      </include>

      <!-- Move Base for First Robot -->
      <arg name="custom_param_file" default="$(find sarl_star_ros)/param/$(arg laser_type)_costmap_params.yaml"/>
      <include file="$(find sarl_star_ros)/launch/includes/move_base/move_base1.launch.xml">
         <arg name="custom_param_file" value="$(arg custom_param_file)"/>
      </include>

      <!-- SARL_star Planner for First Robot -->
      <node pkg="sarl_star_ros" type="sarl_star_node1.py" name="sarl_star_node" output="screen">
        <remap from="/amcl_pose" to="/sarl_tb3/amcl_pose"/>
        <remap from="/move_base/global_costmap/costmap" to="/sarl_tb3/move_base1/global_costmap/costmap"/>
        <remap from="/cmd_vel" to="/sarl_tb3/cmd_vel"/>
        <remap from="/move_base_simple/goal" to="/sarl_tb3/move_base_simple/goal"/>
        <remap from="/local_goal" to="/sarl_tb3/local_goal"/>
        <remap from="/odom" to="/sarl_tb3/odom"/>
        <remap from="/obstacles" to="/sarl_tb3/obstacles"/>
        <remap from="/trajectory_marker" to="/sarl_tb3/trajectory_marker"/>
        <remap from="/vehicle_marker" to="/sarl_tb3/vehicle_marker"/>
        <remap from="/goal_marker" to="/sarl_tb3/goal_marker"/>
        <remap from="/action_marker" to="/sarl_tb3/action_marker"/>
      </node>

      <!-- Laser Filter for First Robot -->
      <node pkg="laser_filters" type="scan_to_scan_filter_chain" name="laser_filter_sarl_tb3">
        <rosparam command="load" file="$(find laser_filters)/laserscan_filter.yaml"/>
        <remap from="/scan" to="/sarl_tb3/scan"/>
      </node>

      <!-- Obstacle Detector for First Robot -->
      <include file="$(find obstacle_detector)/launch/nodes.launch"/>
   </group>

```

## Test the SARL* Agents in Gazebo

To test the trained models in Gazebo, make sure the ROS nodes used by the spawned robots point to the correct DRL model. 
This can be checked in `~/racing_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node1.py`. Make sure, you are checking 
the same `.py` file as defined in the launchfile.

The model being used by the robot can be changed in this file, inside the main function:
```python
if __name__ == '__main__':
    begin_travel = False
    # set file dirs
    pack_path = rospkg.RosPack().get_path('sarl_star_ros')
    model_dir = pack_path + '/CrowdNav/crowd_nav/data/your_output_folder/'
    env_config_file = model_dir + 'env.config'
    policy_config_file = model_dir + 'policy.config'
```

To launch the simulation environment, first `cd` into the created workspace: `racing_ws`. Then, source into the
workspace with
```bash
source devel/setup.bash
```

Once sourced into the workspace, export the turtlebot3 model to be used as:
```bash
export TURTLEBOT3_MODEL=burger
```

Finally, launch the simulation environment: 
```bash
roslaunch sarl_star_ros turtlebot3_sarl_star_world_track_multi1.launch.xml
```
This will automatically run the robots, as they incorporate an automated goal sending mechanism. To modify this feature, 
see the `sarl_star_node.py` code corresponding to the robot. There, it's possible to modify the navigation goals sent to the robot.
```python
try:
  rospy.init_node('sarl_star_node', anonymous=True)
  rate = rospy.Rate(4)  # 4Hz, time_step=0.25
  robot_act = RobotAction()
  listener_v = tf.TransformListener()
  listener_g = tf.TransformListener()
  listener_ob = tf.TransformListener()
  
  robot_act.send_goal(partial_goals[0][0], partial_goals[0][1], 0.0, 0.0)
```
The behavior in RViz will be similar to this:

[Screencast from 22-10-24 19:32:22.webm](https://github.com/user-attachments/assets/6aae57ff-b96e-4173-b7b9-2f33273d9991)

And with multiple robots and traditional agents: 

[Screencast from 23-11-24 23:08:03.webm](https://github.com/user-attachments/assets/8bb94212-ab64-431e-95e9-6d16083238a5)

## Reference
SARL* : https://github.com/LeeKeyu/sarl_star

CrowdNav : https://github.com/vita-epfl/CrowdNav

Turtlebot3 : https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

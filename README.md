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



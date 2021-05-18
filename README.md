# MAEG5755-2021-Team-PARK
Team PARK Repo for MAEG5755 Robotics Project
Pretty, Alice, Rui, Kenson


### Objectives
1. Dexnet 4.0 bin picking demo with Baxter
2. Refresh Baxter! (ROS Noetic, baxter_interface, realsense2, Moveit)
3. Documenting how to run a baxter in 2021

### Wiki
[Wiki](https://github.com/mfkenson/MAEG5755-2021-Team-PARK/wiki) is more comprehensive and up-to-date for various topics.

Especically the [FAQ](https://github.com/mfkenson/MAEG5755-2021-Team-PARK/wiki#faq)

### First time setup 
```
mkdir -p $HOME/5755_ws/src
cd $HOME/5755_ws/src
git clone --recursive https://github.com/rojas70/learning_ros_external_pkgs_noetic.git
git clone --recursive https://github.com/rojas70/learning_ros_noetic.git
git clone --recursive https://github.com/mfkenson/MAEG5755-2021-Team-PARK.git team-park

cd $HOME/5755_ws/
cp src/learning_ros_external_pkgs_noetic/baxter/baxter.sh .
cd $HOME/5755_ws/src/team-park/dependencies/ && bash install_dependencies.sh
cat $HOME/5755_ws/src/team-park/dependencies/shortcuts_alias.txt >> ~/.bashrc
```
Real environment you need to also [install librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

### Pulling latest from upstream repo
```
cd $HOME/5755_ws/src/team-park
git pull
git submodule update
```


### Build the workspace
```
cd $HOME/5755_ws/
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
Successful compile gives 100% built messages..
Example:
```
...
...
[100%] Built target command_bundler
```
If there there is an error you can't escape, delete the devel folder and build folder then catkin_make again.
```
rm -rf $HOME/5755_ws/devel
rm -rf $HOME/5755_ws/build
cd $HOME/5755_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j1
```


## Make it run

### Gazebo Simulation

* Not recommend to use baxter.sh anymore. We should manually handle the environment variables.
* In simulation mode, make sure your ~/.bashrc DO export ROS_IP and export ROS_MASTER_URI (pointing to localhost, i.e. 127.0.0.1)
```
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
```
* You can verify your environment variable by `echo $ROS_IP` and `echo $ROS_MASTER_URI` in terminal

Terminal 1: Just an empty world
```
sw && roslaunch gazebo_ros empty_world.launch
```
Terminal 2: Spawn baxter with realsense
```
sw && roslaunch park_gazebo baxter_on_pedestal_w_realsense.launch
```
Terminal 3: Moveit planning
```
roslaunch baxter_moveit_tutorial moveit_init.launch
```
Terminal 4: Rviz
```
baxter_home && sw && rviz -d src/team-park/park_simulation/park_gazebo/park.rviz
```
Terminal 5: Enable and unstuck the robot
```
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
```

![Alt text](/screenshots/baxter_realsense_gazebo_rviz.png?raw=true "Baxter_D435")

### Real environment in CUHK Robotics Lab

* Not recommend to use baxter.sh anymore. We should manually handle the environment variables.
* In real environment mode, make sure your ~/.bashrc DO export ROS_IP and export ROS_MASTER_URI
* In this case IP address is 192.168.11.101 and the baxter hostname is 011508P0007.local. Check out the [wiki](https://github.com/mfkenson/MAEG5755-2021-Team-PARK/wiki/Baxter-Simulation-and-Real-Robot) for hostname resolving
```
export ROS_IP=192.168.11.101
export ROS_MASTER_URI=http://011508P0007.local:11311
```
* You can verify your environment variable by `echo $ROS_IP` and `echo $ROS_MASTER_URI` in terminal

Terminal 1: Realsense node
```
sw && roslaunch park_demo camera.launch
```
Terminal 2: Moveit planning
```
sw && roslaunch baxter_moveit_tutorial moveit_init.launch
```
Terminal 3: Rviz
```
baxter_home && sw && rviz -d src/team-park/park_demo/park_demo/park.rviz
```
Terminal 4: Enable the robot
```
rosrun baxter_tools enable_robot.py -e
```
Terminal 5: Publishing transform for realsense camera and world frame (after hand eye calibration only)
```
TODO TODO TODO 
```
here simply launch a generated launch file obtained from hand-eye calibration process. See [wiki](https://github.com/mfkenson/MAEG5755-2021-Team-PARK/wiki/Hand-Eye-Calibration)

### Caution
In rviz you can plan your path under motion planning tab even with scene planner. Be aware of surroundings when executing trajectory.

### Tips for pycharm users
* Get pycharm IDE [here](https://www.jetbrains.com/pycharm/download/#section=linux)
* community version is more than sufficient
* There is a collection of python launcher scripts managing the roslaunch process.
* See [park_demo/park_demo/scripts/launcher](https://github.com/mfkenson/MAEG5755-2021-Team-PARK/tree/main/park_demo/park_demo/scripts/launcher)
* Make sure you source the workspace before opening pycharm in terminal and having the roscore running somewhere else
# MAEG5755-2021-Team-PARK
Team PARK Repo for MAEG5755 Robotics Project
Pretty, Alice, Rui, Kenson


### Objectives
1. Dexnet 4.0 bin picking demo with Baxter
2. Refresh Baxter! (ROS Noetic, baxter_interface, realsense2, Moveit, Moveit Task Constructor)
3. Showcase how to run a baxter in 2021

### Wiki
[Wiki](https://github.com/mfkenson/MAEG5755-2021-Team-PARK/wiki) is more comprehensive and up-to-date.

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
bash $HOME/5755_ws/src/team-park/dependencies/install_dependencies.sh
cat $HOME/5755_ws/src/team-park/dependencies/shortcuts_alias.txt >> ~/.bashrc
```

### Build the workspace
```
cd $HOME/5755_ws/
catkin_make
```
Successful compile gives 100% built messages..
Example:
```
[ 98%] Built target straddle_block_client
[ 98%] Built target acquire_block_client
[ 98%] Built target open_loop_yaw_service
[ 98%] Built target object_grabber_action_server
[ 98%] Built target example_cart_move_client
[100%] Built target coordinator_action_client_tester
[100%] Built target part_fetcher_exmpl
[100%] Built target fetch_and_stack_client
[100%] Built target object_grabber_action_server2
[100%] Built target command_bundler
```


### Run and test
```
source ~/.bashrc
sb
sw
baxter_sim
```

### PARK Gazebo
Terminal 1
```
sw && roslaunch gazebo_ros empty_world.launch
```
Terminal 2
```
sw && roslaunch park_gazebo baxter_on_pedestal_w_realsense.launch
```
Terminal 3
```
baxter_home && rviz -d src/team-park/park_simulation/park_gazebo/park.rviz
```
![Alt text](/screenshots/baxter_realsense_gazebo_rviz.png?raw=true "Baxter_D435")


### moveit
```
roslaunch baxter_moveit_tutorial moveit_init.launch
```

### export image
```
rosrun park_demo export_image.py :topic='/cameras/head_camera/image'
```

### Depth Image Visualization
![RGB](/screenshots/visualize_color.png?raw=true "RGB")
![339mm](/screenshots/visualize_depth.png?raw=true "Depth object 339mm")
![446mm](/screenshots/visualize_depth_table.png?raw=true "Depth table 446mm")


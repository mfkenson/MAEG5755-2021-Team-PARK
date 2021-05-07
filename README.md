# MAEG5755-2021-Team-PARK
Team PARK Repo for MAEG5755 Robotics Project
Pretty, Alice, Rui, Kenson

### First time setup 
```
mkdir -p $HOME/5755_ws/src
cd $HOME/5755_ws/src
git clone --resursive https://github.com/rojas70/learning_ros_external_pkgs_noetic.git
git clone --resursive https://github.com/rojas70/learning_ros_noetic.git
git clone --resursive https://github.com/mfkenson/MAEG5755-2021-Team-PARK.git team-park

cd $HOME/5755_ws/
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

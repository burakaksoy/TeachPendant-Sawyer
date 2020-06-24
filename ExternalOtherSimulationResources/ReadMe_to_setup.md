# OTHER ROBOTS (eg. ABB, UR5, STAUBLI) SIMULATOR SETUP STEPS #


#### Latest Update: Mon 22 June 2020 ####
##### Author: Burak Aksoy #####

**_TESTED ON UBUNTU 20.04_**


### A. INSTALL ROBOT RACONTEUR (RR) GAZEBO PLUG-IN ###

*This plug-in exposes Gazebo as an RR service.*

*You can also take a look at the steps at [the repo](https://github.com/johnwason/RobotRaconteur_Gazebo_Server_Plugin)*

1. Create a folder named _catkin\_ws\_RR_
```
mkdir -p catkin_ws_RR/src
cd catkin_ws_RR/src
```
2. Clone RR Gazebo Plug-in and Standart Robot Def
```
git clone --recursive https://github.com/johnwason/RobotRaconteur_Gazebo_Server_Plugin.git
git clone --recursive https://github.com/johnwason/robotraconteur_standard_robdef_cpp.git
```
3. Create and .sh file to build the packages
```
cd ..
touch command.sh 
```
4. Paste the following inside `command.sh` and save. 
```
catkin_make_isolated -DROBOTRACONTEUR_ROS=1 -DCMAKE_BUILD_TYPE=Release
```
5. Run it to build
```
chmod +x command.sh
./command.sh
```
6. Source the workspace for each opened terminal, so we need to edit ~/.bashrc file
```
nano ~/.bashrc
``` 
and paste the devel location of the workspace. e.g:
```
source ~/catkin_ws_RR/devel_isolated/setup.bash
```
7. Also paste some required environment variables
```
export GAZEBO_PLUGIN_PATH=~/catkin_ws_RR/devel_isolated/gazebo_ros_link_attacher/lib
```
8. Don't close the .bahsrc file yet, we need to export one more environment variable in the next section.

### B. DOWNLOAD HONGLU'S RR_PROJECT REPO ###
1. Honglu's [RR project repo](https://github.com/hehonglu123/RR_Project) has a simulation folder for robot models. In a new terminal tab Download it to your home directory with
```
git clone https://github.com/hehonglu123/RR_Project.git
```
2. Add the following line to .bashrc file
```
export GAZEBO_MODEL_PATH=~/RR_Project/simulation/models
```
3. Save and source the .bashrc
4. 'simulation' folder in RR_project repo also has a folder called 'burak' which we will use later to run our world.

### C. DOWNLOAD GAZEBO_LINK_ATTACHER AS VACUUM ###
1. We also need to download the service for vacuum gripper to picking up object with robot as end effector.
```
cd ~/catkin_ws_RR/src
git clone https://github.com/johnwason/gazebo_ros_link_attacher.git
```
2. Build the workspace 
```
./command.sh
```

### C. DOWNLOAD GAZEBO_BIN_LOCAL ###
1. Honglu shared a drive link for already built binaries for Gazebo RR service. This is very similar to the Sawyer_bin_local folder.  Download it to your directory from [this link](https://drive.google.com/file/d/1G1jFxZfuNDkNXuYrplAQGmgsKm79-AnU/view?usp=sharing.)
2. It is a .zip file, you need to extract it after downloading. 
3. This folder has four .sh files named:
```
run_abb_driver.sh
run_sawyer_driver.sh
run_staubli_driver.sh
run_ur5_driver.sh
```
These are the commands to start that robot service. 

### D. START THE SIMULATION ###
1. Start roscore
```
roscore
```
2. In a new tab Make some required files executable
```
cd ~/RR_Project/simulation/burak
sudo chmod +x start_gazebo
```
3. Then start the all simulation with one .sh file
```
./start_gazebo
```
4. This file has
```
gnome-terminal --tab -e ./start_gazebo
sleep 5s
gnome-terminal --tab -e ./initialization.py
cd ~/Gazebo_bin_local/
gnome-terminal --tab -e ./run_abb_driver.sh
cd ~/RR_Project/simulation/burak

sleep 3s

gnome-terminal  --tab -e ./vacuum2.py
sleep 2s
gnome-terminal --tab -e ./testbed_service2.py
gnome-terminal --tab -e ./Cognex_sim.py
```

5. Now you should see an ABB robot and the conveyor the world is ready.


***Trouble shooting*** 
*If you see error with python3, you may need to change the 1st lines of the following files for python3*:
```
initialization.py
reset_models.py
simple_client.py
testbed_service2.py
vacuum2.py
```
*as* 
```
#!/usr/bin/env python3
```
5. To start the pick and place demo, on a new tab
```
cd ~/RR_Project/simulation/burak
python3 simple_client.py
```

6. Now You should see that the robot picks and place an object on the fixed box.

***Trouble shooting*** 
*If you see error about xrange, again this is about python3, so you need to change xrange to range for python3 in the shown file of the error. For example I had to edit*: 
```
RR_project/toolbox/general_robotics_toolbox.py
```

7. To reset the models you can use 
```
python3 reset_models.py
```

------
### More Explanation on The Files ###
1. In RR_project/simulation there is a file
```
world.world
```
This file is the configuration of our world in the Gazebo.
2. `start_gazebo` command file runs this world on Gazebo and start the Gazebo RR service.
3. `start_all` command file runs the `start_gazebo` file and then 
4. calls `initialization.py`. This where we connect `GazeboServer` and insert the `models/ABB1200/model.sdf` file. `.sdf` file is the robot description in Gazebo. You can change the robot model name to insert other robots. There is also `calibration/ABB.yaml` file insterted here to configure where to put the robot in the world with defining `H`.
5. Then, run the driver for the robot e.g: `run_abb_driver.sh` from the `~/Gazebo_bin_local`.
6. Then, it starts some RR services such as `vacuum2.py`, `testbed_service2.py`, and `Cognex_sim.py`. 
7. `vacuum2.py` communicates with the `link_attacher_node` and starts the vacuum service.
8. `testbed_service2.py` loads the conveyor and the objects perfume,toothpaste,soap,bottle on it. 
9. `Cognex_sim.py` in real world cognez device tells the related coordinates if the objects in plane. in this case it tells the location of the objects for us as a RR service.
10. `reset_models.py` resets the objects on the test_bed.
11. `simple_client.py` connects to cognex, robot, and vacuum services. Loads the yaml file, gets H, defines the pick and place functions and executes them.

12. Honglu uses his own `jog_joint` function. The difference btw. RR jog_joint and this is that, This takes the robot, Emulated Velocity Control object as `vel_ctrl`, desired joint angles as `q`, and time to get to that that desired joint angles `t`. RR jog_joint function does not have velocity option for now. you may need to change constant multiplier at line 12 of `jog_joint.py` to set up qdot. 





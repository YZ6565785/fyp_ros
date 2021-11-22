# Final Year Project

Author: Yuhang Zhang

Student number: 1********

This package contains all the essential software materials to execute system implemented based on the FYP report. The Robotic arm with the simulated world is coded in the package "arq_gazebo", and the main implementation of the task execution is coded in the package "yuhang_gzebo" using python. Both packages are under the path "submit_code/fyp_ws/src/yuhang/".

This system does not support an executable script, because before running the code serval requirements listed below have to be satisfied. And the system must be running separately in 3 terminals.

Important: The system is only implemented and tested on Linux OS Ubuntu 16.04.

## Prepare the system:
1. make sure the following packages are prepared:

* python 3.6.5+ is installed
* ROS kinetic Kame - full version installed
* rviz installed
* universal_robot package installed
* OpenCV (cv2) installed

2. make sure the following software installed:
* Gazebo 7
* MoveIt!

3. replace the Gazebo default model database with the provided model folder:

```
sudo cp -r ./models/{aruco_board_yuhang,aruco_cube,board,camera_plug,kinect} ~/.gazebo/models/
```
4. copy the "submit_code" folder into the home diretory

## Run the system
1. load the world in Gazebo simulator in a new terminal
```
cd ~/submit_code/fyp_ws
source devel/setup.bash
roslaunch yuhang_gazebo yuhang_ur5_robotiq.launch 
```
2. run Image Tracking in a new terminal
```
cd ~/submit_code/fyp_ws
source devel/setup.bash
rosrun yuhang_gazebo tracking_object.py 
```
3. execute the system in a new terminal
```
cd ~/submit_code/fyp_ws
source devel/setup.bash
rosrun yuhang_gazebo state_machine_userdata.py
```

Now, you can start to control the robot using demonstrations (demonstrations can be given by moving the cubes in the Gazebo Simulator)


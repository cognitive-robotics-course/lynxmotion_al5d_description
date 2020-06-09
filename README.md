# Lynxmotion AL5D simulator
This package exposes the Lynxmotion AL5D arm robot in a simulation world. It makes use of the kinematics of the robot in terms of length of joints
, axis of rotations and acceptable range of rotation to simulate the robot.

# Usage
The visualization tool used for this simulation at the moment is *Rviz*. For running the visualization, you will first need to clone the project
in your ROS workspace by running the following commands:
``` bash
roscd
cd ../src
git clone https://github.com/CRAM-Team/lynxmotion-description.git 
```
You can then run the visualization by issuing the command `roslaunch lynxmotion_description al5d.launch` from your terminal. This will
open an RViz window with the robot. Beside the Rviz window, a small window identifable by a question mark in the launch bar will also appear.
There you will see the different joints of the robot and play around to move the robot to different places.

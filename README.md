# Lynxmotion AL5D simulator
This package exposes the Lynxmotion AL5D arm robot in a simulation world. It makes use of the kinematics of the robot in terms of length of joints
, axis of rotations and acceptable range of rotation to simulate the robot.

# Usage
The visualization tool used for this simulation at the moment is *Rviz*. For running the visualization, first make sure that the joint\_state\_publisher package is installed. Install them by running:
``` bash 
sudo apt install ros-kinetic-joint-state-publisher ros-kinetic-joint-state-publisher-gui
```
You can then proceed to cloning the repository as follows:
``` bash
roscd
cd ../src
git clone https://github.com/CRAM-Team/lynxmotion_al5d_description.git 
```
You can then run the visualization by issuing the command `roslaunch lynxmotion_al5d_description al5d.launch` from your terminal. This will
open an RViz window with the robot. Beside the Rviz window, a small window identifable by a question mark in the launch bar will also appear.
There you will see the different joints of the robot and play around to move the robot to different places.

### Note
A camera has been added to the model and is supposed to be at the center of the
blue board in the visualization. But no resource is available to verify the proper working of the camera in Rviz. Any suggestion on this is very welcome.

# Lynxmotion AL5D simulator
This package exposes a simulated version of the Lynxmotion AL5D arm robot. The robot model developed reproduces the physical properties of the robot as well as the kinematics. As it is, it can be used in any physics engine or visualization tool. For the purpose of conciseness, the current package only exposes the use of the simulator in [Rviz](https://github.com/ros-visualization/rviz) and in [Gazebo](http://gazebosim.org).

![Overview of the simulated robot in Rviz](screenshots/rviz.png?raw=true "Overview of the simulated robot in Rviz")

# Usage

## Visualization in Rviz

This set of instructions assume that the user has a complete and functional ROS installation. If that's not the case, please do so by following the instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and install the desktop-full-package of ROS.
After confirming that the ROS installation is working, clone the repository and  install some additional packages to visualize the robot in Rviz. Those packages can be installed by running the set of commands below:
``` bash
sudo apt install ros-kinetic-joint-state-publisher ros-kinetic-joint-state-publisher-gui
roscd
cd ../src
git clone https://github.com/CRAM-Team/lynxmotion_al5d_description.git 
```
You can then run the visualization by issuing the command 

`roslaunch lynxmotion_al5d_description al5d.launch`

from your terminal. This will open an RViz window with the robot. Beside the Rviz window, a small window identifable by a question mark in the launch bar will also appear.

There you will see the different joints of the robot and play around to move the robot to different places. Here is a screenshot of the aforementioned window.

![Overview of the simulated robot in Rviz](screenshots/publisher_window.png?raw=true "Joint Publisher GUI")

## Running the simulation in Gazebo
One limitation of Rviz is that it is just a visualization tool and does not provide a real simulation engine where physics can be tested. Beside, no sensors can be used inside Rviz. This has motivated the developement of a second version of the simulation in Gazebo that is more realistic and which exposes the subscriber-publisher architecture of ROS through which sensor data is read and joint values published.

In order to launch the Gazebo simulation, some additional packages are needed that allow to control the joints and to subscribe to their values using the interface between ROS and Gazebo. The set of commands to run for accessing those packages are as follows:

```bash
sudo apt install ros-kinetic-gazebo-ros-control 
sudo apt install ros-kinetic-effort-controllers ros-kinetic-joint-state-controller ros-kinetic-position-controllers
```

After successful installation of the packages listed above, you can visualize the robot in Gazebo by running the command 

`roslaunch lynxmotion_al5d_description al5d_gazebo_control.launch`

 from your terminal. This will open up a Gazebo with the robot spawned at the center of an empty world. Zoom in to see closer and observe the robot movement carefully as seen below.

![Lynxmotion AL5D spawned in an empty Gazebo simulation world](screenshots/robot_in_gazebo.png?raw=true "Overview of the simulated robot in Gazebo")

Beside opening the Gazebo window, issuing the previous command also creates a set of ROS topics for the joints and also the external camera added to the robot model. You can view the topics by running the following command in a terminal.

`rostopic list`

This will print out the list of all available topics among which you will observe some with the prefix `lynxmotion_al5d` which refers to the simulators subsribers and publishers. The image below, picturing the joints of the AL5D robot shows the mapping of each controller to the physical robot joints.


![Mapping of the controllers to the robot joint](screenshots/controllers_mapping.jpg?raw=true "Mapping the controllers to the robot joint")

### Sending joint values to the Gazebo simulator
The syntax for sending joint positions to the simulated robot is:
```bash
rostopic pub -1 /lynxmotion_al5d/[controller_name]/command std_msgs/Float64 "data: [joint_position]"
```
*Note: You should make sure that the joint values that you are sending are inside the acceptable range as described below:*
```
link1_rotx: [-PI/2; PI/2]
link1_rotz: [-PI; PI]
link2_rotx: [-PI/2; PI/2]
link3_rotx: [-PI/2; PI/2]
link4_rotz: [-PI; PI]
left_finger_mov: [-0.02; 0]
right_finger_mov: [0; 0.02]
```

### Accessing the vision sensor
To view the images captured by the external vision sensor added to the model. run the following command from a terminal. (The camera is represented by a transparent cube in the simulation world)

`rosrun image_view image_view image:=/lynxmotion_al5d/al5d_external_vision/image_raw`

This will open an image\_view screen with the image getting updated every half of a second.

![Visualization of the image captured by the camera sensor](screenshots/image_view.png?raw=true "Image captured by the camera sensor linked to the simulator")

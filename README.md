
# Lynxmotion AL5D simulator
This package exposes a simulated version of the Lynxmotion AL5D arm robot. The robot model developed reproduces the physical properties of the robot as well as the kinematics. As it is, it can be used in any physics engine or visualization tool. For the purpose of conciseness, the current package only exposes the use of the simulator in [Rviz](https://github.com/ros-visualization/rviz) and in [Gazebo](http://gazebosim.org).

![Overview of the simulated robot in Rviz](screenshots/rviz.png?raw=true "Overview of the simulated robot in Rviz")

# Usage

+ [Visualization in Rviz](#visualization-in-rviz)
+ [Running the simulation in Gazebo](#running-the-simulation-in-gazebo)
+ [Sending joint values to the Gazebo simulator](sending-joint-values-to-the-gazebo-simulator)
+ [Sending joint values to the Gazebo simulator](sending-joint-values-to-the-gazebo-simulator)
+ [Accessing the joint states](accessing-the-joint-states)
+ [Accessing the vision sensor](#accessing-the-vision-sensor)
+ [Spawning Lego Blocks](#spawning-lego-blocks)

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

`
sudo apt update
`

`
sudo apt install ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros ros-kinetic-gazebo-dev ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros-pkgs libignition-math2-dev
`

`sudo apt install ros-kinetic-effort-controllers ros-kinetic-joint-state-controller ros-kinetic-position-controllers`

`sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable ``lsb_release -cs`` main" > /etc/apt/sources.list.d/gazebo-stable.list'`

```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install gazebo7 -y
```

The Gazebo simulation also has a dependency on the [mimic joints plugin from roboticsgroup](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins). To install the dependency, run the following command.

```bash
roscd

cd ../src

git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git

cd ..

catkin_make
```

After successful installation of the packages and dependencies listed above, you can visualize the robot in Gazebo by running the command 

`roslaunch lynxmotion_al5d_description al5d_gazebo_control.launch`

 from your terminal. This will open up a Gazebo with the robot spawned at the center of an empty world as seen on the image below.


![Lynxmotion AL5D spawned in an empty Gazebo simulation world](screenshots/robot_in_gazebo.png?raw=true "Overview of the simulated robot in Gazebo")

Beside opening the Gazebo window, issuing the previous command also creates a set of ROS topics for the joints and also the external camera added to the robot model. You can view the topics by running the following command in a terminal.

`rostopic list`

This will print out the list of all available topics among which you will observe some with the prefix `lynxmotion_al5d` which refers to the simulators subsribers and publishers. Among the important topics here, we have one publisher for the robot joint states (`/lynxmotion_al5d/joint_states`), another for publishing all the joint values at once (`/lynxmotion_al5d/joints_positions/command`) and a topic to access the images from the camera sensor attached to the robot.

### Sending joint values to the Gazebo simulator
The syntax for sending joint positions to the simulated robot is:

`rostopic pub -1 /lynxmotion_al5d/joints_positions/command std_msgs/Float64MultiArray "data: [<Array of the joint values for the five joints and distance between the two fingers>]"
`

For example, to send the robot the joint values for the initial/home position, we would run the following command:

`rostopic pub -1 /lynxmotion_al5d/joints_positions/command std_msgs/Float64MultiArray "data: [0, 1.57,-1.57, 0, 0, 0.03175]"`

*Note: The available joints and they positions on the robot are shown in the following images. Also, the order of the values to send to the robot are: Joint1, Joint2, Joint3, Joint4, Joint5, Gripper*

![Simulated robot joints positions mapping](screenshots/joints_mapping.png?raw=true "Mapping the robot joints")

The range of the values acceptable by the joints are as follows:
```markdown
Joint1: [-PI; PI]
Joint2: [0; PI]
Joint3: [-PI/2; PI/2]
Joint4: [-PI/2; PI/2]
Joint5: [-PI; PI]
Gripper: [0; 0.03175]
```

### Accessing the joint states
The `joint_states` publisher publishes the robot joint states at a rate of 50Hz (i.e every 2ms). In order to access the positions of the joints from the terminal, the command to run is `rostopic echo /lynxmotion_al5d/joint_states`. The output is very verbose so it is advised, if possible, to redirect the output to a file.

### Accessing the vision sensor
To view the images captured by the external vision sensor added to the model. run the following command from a terminal. (The camera is represented by a transparent cube in the simulation world)

`rosrun rqt_image_view rqt_image_view`

In the window that appears, select the topic `/lynxmotion_al5d/external_vision/image_raw` in the dropdown to visualize the images captured by the robot. The image captured by the camera when the robot is in the  (0, 0, 0, 0, 0, 0) position is displayed below.

![Visualization of the image captured by the camera sensor](screenshots/rqt_image_view.png?raw=true "Image captured by the camera sensor linked to the simulator")

### Spawning Lego blocks

For the purpose of experinmentation, a set of [lego blocks](https://www.lego.com/en-us/vip/vippromotions.jsp) has been created to run various exercises using the robot model. Three colors of blocks have been set up (Blue, Red and Green) and integrated to the package. In order to spawn them, you should go to the **Insert** menu in Gazebo and you would see the three blocks under the name of the package in your ROS package. Click on the block that you want to insert and drop it in the workspace by clicking at the position you want to insert it to. You can also adjust the position using the pose values in Gazebo.
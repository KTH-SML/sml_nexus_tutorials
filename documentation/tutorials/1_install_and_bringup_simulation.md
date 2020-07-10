# Tutorial 1 - Install and bringup simulation

## Prerequisites
### Install Gazebo for ROS
Please follow instruction [here]() to install Gazebo for ROS

### Install ROS Dependencies
First update your package list by running:

`sudo apt-get update`

And install the needed package by running the following lines (replacing <your_ros_distro> by your ROS distro):

```
sudo apt-get install ros-<your_ros_distro>-urdf
sudo apt-get install ros-<your_ros_distro>-robot-state-publisher
sudo apt-get install ros-<your_ros_distro>-kdl-parser
sudo apt-get install ros-<your_ros_distro>-geometry_msgs
sudo apt-get install ros-<your_ros_distro>-gazebo-dev
sudo apt-get install ros-<your_ros_distro>-nav_msgs
sudo apt-get install ros-<your_ros_distro>-std_msgs
sudo apt-get install ros-<your_ros_distro>-std_srvs
sudo apt-get install ros-<your_ros_distro>-tf
```

Most of these packages should already be installed, as they are included in the default ROS desktop installation.

## Install SML Nexus packages
Download the required SML Nexus packages from Github to your workspace source directory:

```
cd <your_workspace>/src
git clone https://github.com/KTH-SML/sml_nexus.git
git clone https://github.com/KTH-SML/sml_nexus_simulator.git
```
And the current tutorial package:

`git clone https://github.com/KTH-SML/sml_nexus_tutorials.git`

Build the packages using either "catkin_make" or "catkin build"

```
catkin_make
```

Don't forget to source once everything has been built

```
cd ..
source devel/setup.bash
```


## Bringup The Simulation

Start the following launch file to bringup Gazebo and spawn a nexus robot:
`roslaunch sml_nexus_tutorial 1_install_and_bringup.launch`

You should see Gazebo starting and a nexus robot spawning on screen.

<a href="url"><img src="/documentation/pictures/tutorial_1_picture_1.png" align="center" height="400" width="600"></a>


## Command & Feedback

In a new terminal window, type the following command to get the list of topics currently advertised:

`rostopic list`

In the list our topics of interest are:

* For command:
	* /cmd_vel

* For feedback:
	* /qualisys/nexus1/pose
	* /qualisys/nexus1/velocity
	* /qualisys/nexus1/odom

### Getting Feedback

In our simulation setting, a node is simulating the motion capture system of the SML, and therefor the feedback topic naming and data are similar to what the real system provides.

The motion capture provides feedback on the following topics:

* `/qualisys/nexus1/pose`: A 3D pose (cartesian coordinates) and an 3D orientation (quaternion) relative to the motion capture system frame of reference.

* `/qualisys/nexus1/velocity`: A 3D linear velocity (along x, y and z axis) and an angular velocity (quaternion) relative to the motion capture system frame of reference.

* `/qualisys/nexus1/odom`: A combined pose and velocity message including covariances (not simulated). The pose is expressed relative to the motion capture frame of reference and the velocity is expressed relative to the tracked object frame of reference.

You can listen to this topics by using the following command (Ctrl+C to exit):

`rostopic echo <topic_name>`

For example, use the following to listen to the pose:

`rostopic echo /qualisys/nexus1/pose`

It should give a pose feedback like the following:

```
---
header: 
  seq: 85
  stamp: 
    secs: 10
    nsecs: 388000000
  frame_id: "mocap"
pose: 
  position: 
    x: 7.38091996374e-05
    y: -0.000609077519851
    z: 8.03377774028e-09
  orientation: 
    x: -8.60502922864e-08
    y: 1.78305125828e-08
    z: 2.96037453166e-06
    w: -0.999999999996
---
```


### Sending Commands
The nexus robots are velocity-controlled (second order). As ground holonomic robots, they can be controlled in linear velocity along the x, y-axis and in angular velocity around the z-axis.

The command uses a Twist message (ROS message type for velocity), and needs to be published on the `/cmd_vel` topic at a high enough frequency.

Use the following command in a new terminal window to publish a velocity command and drive the robot forward (Ctrl+C to exit):

```
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 50 
```

The command will be published on the topic at a 50Hz frequency (`-r 50`). You can play with the values of the command and look at how the robot reacts. You can also listen to the different feedback topics and see how they evolve.

## Examining The Launch File

Using the following command, you can get a list of all the running ROS nodes:
`rosnode list`

```
/gazebo
/gazebo_gui
/qualisys
/robot_state_publisher
/rosout
```

The list includes "gazebo", the simulation software, and "qualisys", our simulated motion capture system node.

Looking at the launch file in `/sml_nexus_tutorials/sml_nexus_tutorials/launch`:

```
<?xml version="1.0"?>
<launch>
  <arg name="use_sim_time" default="true" />
  <arg name="gui" default="true" />
  <arg name="headless" default="false" />

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="0" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="use_sim_time" value="$(arg use_sim_time)" />
    <arg name="headless" value="$(arg headless)" />
    <arg name="paused" value="false"/>
  </include>

  <!-- Load robot description -->
  <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />

  <!-- Spawn the robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -model nexus1 -param robot_description -x 0 -y 0 -z 0.5" />

  <!-- Motion capture system simulation -->
  <include file="$(find mocap_simulator)/launch/qualisys_simulator.launch" />

</launch>
```

The first part is defining the arguments used by Gazebo and the launching Gazebo:

```
  <arg name="use_sim_time" default="true" />
  <arg name="gui" default="true" />
  <arg name="headless" default="false" />

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="0" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="use_sim_time" value="$(arg use_sim_time)" />
    <arg name="headless" value="$(arg headless)" />
    <arg name="paused" value="false"/>
  </include>
```

Then the robot description is loaded and the robot model is spawn in Gazebo:

```
  <!-- Load robot description -->
  <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />

  <!-- Spawn the robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -model nexus1 -param robot_description -x 0 -y 0 -z 0.5" />
```

Finally the motion capture simulation node is launched:

```
  <!-- Motion capture system simulation -->
  <include file="$(find mocap_simulator)/launch/qualisys_simulator.launch" />
```

## Next Tutorial
Once you're done, you can continue to the next tutorial: [Simple Controller Simulation (Python)](/documentation/tutorials/2_simple_controller_simulation_python.md)

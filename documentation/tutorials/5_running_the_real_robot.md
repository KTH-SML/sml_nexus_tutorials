# Tutorial 4 - Running the Real Robot

In this tutorial we will learn how to implement the previously seen controllers on the real nexus robot.

## Prerequisites
The SML Nexus packages must have been installed and the simulation setup as detailed in the [first tutorial](/documentation/tutorials/1_install_and_bringup_simulation.md). It's also better to have followed the previous tutorials to understand how the controllers are behaving.

## Hardware presentation

<a href="url"><img src="/documentation/pictures/tutorial_5_picture_1.png" align="center" height="512" width="1000"/></a>

The Nexus robots includes the following components:
* **Onboard computer:** In most case a NVidia Jetson TX2, NVidia Jetson Nano or Inter NUC. The computer is connected to the internal low-level controller by USB and can be connected to accessories (camera, manipulator,...) by USB or other interface. The computer provides the Wi-Fi interface to the robot.
* **Low-level controller:** An internal Arduino Mega runs the motor drivers and handle the encoders
* **Internal battery:** An internal battery is integrated into the base and provide power to the low-level controller and motors
* **External battery:** An external battery and a voltage converter provide a power supply to the onboard computer and accessories.
* **Ultrasonic range sensor:** A range sensor on each side of the nexus provides a basic distance to obstacle estimation.

Below is presented a simplified diagram of the Nexus robot control.

<a href="url"><img src="/documentation/pictures/tutorial_5_picture_2.png" align="center" height="313" width="1000"/></a>

## Motion Capture System
Please see with a lab engineer to get explainations on how to register the robot in the motion capture system.

You need to install the ROS motion capture system package from github. Go to your catkin workspace and download the package:

```
cd <your_workspace>/src
git clone https://github.com/KTH-SML/motion_capture_system.git
```

Then build the packages using either "catkin_make" or "catkin build"

```
cd ..
catkin_make
```

Don't forget to source once everything has been built

```
source devel/setup.bash
```

If you cannot connect to the motion capture system later on, try changing the *"udp_port"* parameter in the launch file `<your_workspace>/src/motion_capture_system/mocap_qualisys/launch/qualisys.launch` to *-1*:

```
<param name="udp_port" value="-1"/>
```


## Network
All nexus robots are connected to the Wi-Fi network *"QuadrotorDevices"* of the SML (Smart Mobility Lab). To communicate with them, your computer needs to be on the same network. Ask a lab engineer for details and on the password.

Once connected on the network, you can try pinging the onboard computer of the Nexus. You would need first to power the nexus computer. The IP address of the computer is fixed and should be labeled on the computer module.

From you own computer simply type (by replacing <ip-adress> by the onboard computer IP address): `ping <ip-address>`

If both computers are connected properly you should see something similar to the following (done with IP address 12.0.5.5):
```
ping 12.0.5.5
PING 12.0.5.5 (12.0.5.5) 56(84) bytes of data.
64 bytes from 12.0.5.5: icmp_seq=1 ttl=64 time=0.078 ms
64 bytes from 12.0.5.5: icmp_seq=2 ttl=64 time=0.066 ms
64 bytes from 12.0.5.5: icmp_seq=3 ttl=64 time=0.064 ms
64 bytes from 12.0.5.5: icmp_seq=4 ttl=64 time=0.063 ms
64 bytes from 12.0.5.5: icmp_seq=5 ttl=64 time=0.067 ms
```

If successful, you can try connecting to the nexus onboard computer by running the following command. To obtain the login and password, take a look at the module label or ask a lab engineer.

`ssh <login>@<ip-address>`

Your command terminal is now running commands on the nexus onboard computer! Type `exit` or CTRL+D to exit.

### Network configuration
A few global parameters have to be set to allow proper ROS communication with networked computers. In our case, we will run the ROS master on our computer and follow the configuration below. The ROS master is launched by using the command `roscore` or when launching the first launch file with `roslaunch` (if a ros master is already running, the launch file won't launch a new master).

<a href="url"><img src="/documentation/pictures/tutorial_5_picture_3.png" align="center" height="437" width="1000"/></a>

For more information, check the ROS wiki page about [network setup](http://wiki.ros.org/ROS/NetworkSetup)

#### User computer
We need to set the parameters ROS_MASTER_URI and ROS_IP before running anything. Type the following command in your computer terminal, replacing <user-pc-ip> by your own IP address (type `ifconfig` to obtain it).

```
export ROS_MASTER_URI=http://<user-pc-ip>:11311
export ROS_IP=<user-pc-ip>
```

**Tip:** Add those command to your *.bashrc* file so that they are run everytime you open a new terminal window.

You can check that the commands worked by running the following: `echo $ROS_MASTER_URI` and `echo $ROS_IP`

### Nexus onboard computer
Now we need to set the network on the nexus onboard computer. Connect to the onboard computer using SSH (as described in the "Network" section):

`ssh <login>@<ip-address>`

Once connected, set the global variable. The ROS master URI is the same across all networked computers, as only one of them is running the master. The ROS IP is the one of the computer you are running the command from.

```
export ROS_MASTER_URI=http://<user-pc-ip>:11311
export ROS_IP=<onboard_computer-ip>
```

**Tip:** Add those command to your *.bashrc* file so that they are run everytime you open a new terminal window.

You can check that the commands worked by running the following: `echo $ROS_MASTER_URI` and `echo $ROS_IP`

## Running the robot
We will now run the code and run the robot.

### Powering the robot
Both the onboard computer and the robot base itself need to be powered.
Power the robot base by pushing the red button on the side. Power the onbard computer by plugging a battery (if not already done).

<a href="url"><img src="/documentation/pictures/tutorial_5_picture_4.jpg" align="center" height="600" width="800"/></a>

### Starting ROS master
Launch the ROS master and the motion capture node from your computer by running:

`roslaunch mocap_qualisys qualisys.launch`.

### Onboard computer
Using another terminal window, connect by SSH to the nexus onboard computer (if not already connected) and run the following ROS launch file:

`roslaunch sml_nexus_robot sml_nexus_bringup.launch`

You might need to source the workspace:

```
cd catkin_ws
source devel/setup.bash
```

If everything is working properly, you get the same topics as in [Tutorial 1 - Install and bringup simulation](/documentation/tutorials/1_install_and_bringup_simulation.md). Notably:
* `/qualisys/nexus1/pose`: A 3D pose (cartesian coordinates) and an 3D orientation (quaternion) relative to the motion capture system frame of reference.
* `/cmd_vel`: A velocity command for the robot

Use the following command in a new terminal window to publish a velocity command and drive the robot forward (Ctrl+C to exit). **Do not forget to set the ROS master URI and ROS IP everytime you open a new terminal window**.

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

You can echo the robot position by using the following command:

`rostopic echo /qualisys/nexus1/pose`

To stop the code, use CTRL+C on both terminal running the code.

### Running simple controller
You can now run the "Back and Forth" controller from [Tutorial 2 - Simple Controller Simulation (Python)](/documentation/tutorials/2_simple_controller_simulation_python.md).

On your computer run the following launch file:

`roslaunch sml_nexus_tutorials 5_simple_controller_python.launch`

This launch file is a simplified version of the launch file from tutorial 2. The Gazebo simulation and robot spawn has been removed, and the mocap simulation has been replaced by the real mocap launch file. The controller node is exactly the same.

```XML
<?xml version="1.0"?>
<launch>
  <!-- Load robot description -->
  <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />

  <!-- Motion capture system -->
  <include file="$(find mocap_qualisys)/launch/qualisys.launch" />

  <!-- Controller node -->
  <node name="back_and_forth_controller" pkg="sml_nexus_tutorials" type="back_and_forth_controller.py" output="screen" />

</launch>
```

On the nexus, start the local launch file:

`roslaunch sml_nexus_robot sml_nexus_bringup.launch`

You should see the robot going back and forth like in the simulation! As always use CTRL+C to stop the code from running

## And now, what's next?
You now have all the knowledge needed to develop your own controllers in the simulation and moving on to demonstrate on a real robot. Following the same network structure you can add more robots, run accessories on them and so on... Don't forget to ask a lab engineer if you have any question and good luck!
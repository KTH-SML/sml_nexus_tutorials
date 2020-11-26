# Tutorial 2 - Simple controller example (Python)

In this tutorial we are going to look at a simple controller that drives the robot to a pre-determined pose.

## Prerequisites
The SML Nexus packages must have been installed and the simulation setup as detailed in the [previous tutorial](/documentation/tutorials/1_install_and_bringup_simulation.md) 

## Bringup the simulation and the controller

Start the following launch file to bringup Gazebo and spawn a nexus robot:

`roslaunch sml_nexus_tutorials 2_simple_controller_python.launch`

After a few seconds, you should be able to see the robot moving back and forth between two locations.

<a href="url"><img src="/documentation/pictures/tutorial_2_picture_1.png" align="center" height="539" width="1000"/></a>


## Examining the controller
A quick look at the ROS node list will reveal that an additionnal node is running compared to the previous tutorial. In another termnal run:

`rosnode list`

```
/back_and_forth_controller
/gazebo
/gazebo_gui
/qualisys
/robot_state_publisher
/rosout
```

We can get more information on this node by using the rosnode command:

`rosnode info /back_and_forth_controller`

Ẁe can see that the node is subscribed to `/tf` and `/qualisys/nexus1/pose`, and publishes to `/cmd_vel`

Another useful tool to know is `rqt_graph` that plots the relation between the different nodes and topics:

`rosrun rqt_graph rqt_graph`

<a href="url"><img src="/documentation/pictures/tutorial_2_picture_2.png" align="center" height="229" width="1000"/></a>

The relationship between the controller, the motion capture system (qualisys) and the robot (in gazebo) is made very clear.

Use CTRL+C to stop the program.

### Examining the launch file
To understand how everything is launched, take a quick look into [sml_nexus_tutorials/launch/2_simple_controller_simulation_python.launch](sml_nexus_tutorials/launch/2_simple_controller_simulation_python.launch)

```XML
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

  <!-- Controller node -->
  <node name="back_and_forth_controller" pkg="sml_nexus_tutorials" type="back_and_forth_controller.py" output="screen" />

</launch>
```

The `<include>` tags are used to include other launch files into our own. The first part of the file defines a few argument and uses them to launch gazebo with an empty world.

```XML
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

The following line launches the nexus robot description, that contains the model of the robot. It will be used to spawn the robot.

```XML
  <!-- Load robot description -->
  <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />
```

A spawner node is used to spawn the nexus in the simulation. Please take a look at the arguments passed to the node. The robot name (nexus1), model description (robot_description, loaded from the previous line) and spawn location. You can play with the spawn location and see how it affects the controller (z-value shouldn't be changed thought).

```XML
  <!-- Spawn the robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model nexus1 -param robot_description -x 0 -y 0 -z 0.5" />
```

The following line launches the qualisys mocap simulator.

```XML
  <!-- Motion capture system simulation -->
  <include file="$(find mocap_simulator)/launch/qualisys_simulator.launch" />
```

Finally, the controller is launched.

```XML
  <!-- Controller node -->
  <node name="back_and_forth_controller" pkg="sml_nexus_tutorials" type="back_and_forth_controller.py" output="screen" />
```

### Examining the code

Let's look at the code in [sml_nexus_tutorials/src/back_and_forth_controller.py](/sml_nexus_tutorials/src/back_and_forth_controller.py) line by line.

First part of the code is just about importing the needed libraries.

```python
#!/usr/bin/env python
#=====================================
#      Simple controller for the        
#          SML Nexus robot
#               ----
#   Listen to the robot pose from
#    the mocap system and output a
#   velocity command to drive the
#    robot back and forth between
#             two points
#=====================================
import sys
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

```

We then create a class for the controller

```python
class BackAndForthController():
```

In the constructor of the class, the ROS node is initialized. This is needed before accessing any ROS time function or subcribing/publishing on a ROS topic.

```python
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #Initialize node
        rospy.init_node('back_and_forth_controller')
```

Constants and variables are declared and initialized. In this controller we drive the robot alternatively between two poses (goal1 and goal2) that are declared here.

```python
        #First pose to reach, written as (x, y, heading angle)
        goal1 = (1.5, 0, 0)
        #Second pose to reach, written as (x, y, heading angle)
        goal2 = (-1.0, -1.0, 1.57)
        #Set current goal
        current_goal = goal1

        #Tolerance interval for declaring a position as reached (x, y, heading angle)
        reach_tolerance = (0.05, 0.05, 0.02)

        #Controller gains (for x, y, heading)
        gains = (0.5, 0.5, 0.5)
```

The robot pose variable is declared here as a ROS Pose Stamped message ([definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)). This is the same message type received from the robot position feedback.

A "last received feedback time" variable is declared here as [ROS time instance](http://wiki.ros.org/rospy/Overview/Time). Together with the timeout, it is used to stop sending commands if we lose the motion capture feedback.

```python
        #Init robot pose
        self.robot_pose = geometry_msgs.msg.PoseStamped()

        #Init last received pose time
        self.last_received_pose = rospy.Time()

        #Timeout in seconds
        timeout = 0.5
```

The feedback subscriber is setup here. It subscribes to the motion capture system topic (`"/qualisys/nexus1/pose"`) with ROS Pose Stamped message ([definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)). A callback function of the class (`self.pose_callback`) is used and will be called everytime a new message is published. This function declaration is done later in the code.

The command publisher is also declared here. When called, it will publish on the robot command topic (`"/cmd_vel"`) a ROS Twist message (velocity message, [definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html))

For more information about ROS subcribers/publishers, you can take a look at the ROS Python tutorial [Writing a Simple Publisher and Subscriber](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)

```python
        #Setup pose subscriber
        rospy.Subscriber("/qualisys/nexus1/pose", geometry_msgs.msg.PoseStamped, self.pose_callback)

        #Setup velocity command publisher
        vel_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=100)
```

The robot pose and goal pose are expressed in the motion capture system frame of reference but the robot command has to be sent in the robot reference frame. Hopefully the motion capture system node also provides the transform between its frame and the tracked subject frame. ROS provides a framework for transforms (called tf2, [here for more information](http://wiki.ros.org/tf2/Tutorials)). 

The program will wait for a transform to be available before continuing.

```python
        #Setup transform subscriber
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        #Wait for transform to be available
        while not tf_buffer.can_transform('mocap', 'nexus1/base_link', rospy.Time()):
            rospy.loginfo("Wait for transform to be available")
            rospy.sleep(1)
```

Lastly, the command message variable is declared as a ROS Twist message ([definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html))

```python
        #Create a ROS Twist message for velocity command
        vel_cmd_msg = geometry_msgs.msg.Twist()
```

The main loop frequency is regulated using a ROS rate object declared here. The main loop uses the "while ROS is running" condition. The time at the beginning of the loop is also saved.

```python
        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)
        
        while not rospy.is_shutdown():
            now  = rospy.Time.now()
```

In the loop the error is computed using the robot pose variable (`"self.robot_pose"`) from the callback (definition later in the code). In a ROS Pose Stamped message, orientation is defined as a quaternion and therefor needs to be converted to get the robot heading.

```python
            #-----------------------------------
            # Compute error on x and y position
            #-----------------------------------
            error_x = current_goal[0] - self.robot_pose.pose.position.x 
            error_y = current_goal[1] - self.robot_pose.pose.position.y

            #-----------------------
            # Compute heading error
            #-----------------------
            #Get euler angle from pose quaternion
            (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                        self.robot_pose.pose.orientation.y,
                                                        self.robot_pose.pose.orientation.z,
                                                        self.robot_pose.pose.orientation.w])
            error_heading = current_goal[2] - yaw
```

If the last feedback as been received recenlty, the command is computed using the errors and the gains.

```python
            #----------------
            # Compute output
            #----------------
            #If pose feedback has been received recently
            if (now.to_sec() < self.last_received_pose.to_sec() + timeout):
                vel_cmd_msg.linear.x = error_x * gains[0]
                vel_cmd_msg.linear.y = error_y * gains[1]
                vel_cmd_msg.angular.z = error_heading * gains[2]
            #Else stop robot
            else:
                vel_cmd_msg.linear.x = 0
                vel_cmd_msg.linear.y = 0
                vel_cmd_msg.angular.z = 0
```

The commands need to be transformed into the robot frame of reference. A `transform_twist()` function defined later in the code is used. Everything is wrapped in a try-except block to avoid the code from crashing if no transform is available.

The command message is then published.

```python               
            #-----------------
            # Publish command
            #-----------------
            try:
                #Get transform from mocap frame to robot frame
                transform = tf_buffer.lookup_transform('mocap', 'nexus1/base_link', rospy.Time())
                #
                vel_cmd_msg_transformed = transform_twist(vel_cmd_msg, transform)
                #Publish cmd message
                vel_pub.publish(vel_cmd_msg_transformed)
            except:
                continue
```

If the error is withing tolerances, the goal is considered reached.

```python
            #----------------------------------------
            # Check if current goal has been reached
            #----------------------------------------
            if ((abs(error_x) < reach_tolerance[0]) and (abs(error_y) < reach_tolerance[1]) and (abs(error_heading) < reach_tolerance[2])):
                if current_goal == goal1:
                    rospy.loginfo("goal 1 reached")
                    current_goal = goal2
                else:
                    rospy.loginfo("goal 2 reached")
                    current_goal = goal1
```

The rate object created before provides a `sleep()` function that will wait enough time for the frequency loop to be respected.

```python
            #---------------------------------
            # Sleep to respect loop frequency
            #---------------------------------
            r.sleep()

```

The pose callback used to receive the motion capture system feedback. The function takes as argument the message being received and store it into the `self.robot_pose` variable. Current time is also save for the timeout in the main loop.

```python
    #=====================================
    #          Callback function 
    #      for robot pose feedback
    #=====================================
    def pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.robot_pose = pose_stamped_msg

        #Save when last pose was received
        self.last_received_pose = rospy.Time.now()
```

The `transform_twit()` function definition. The function is used to convert the velocity command to the robot reference frame.

```python
#=====================================
# Apply transform to a twist message 
#     including angular velocity
#=====================================
def transform_twist(twist= geometry_msgs.msg.Twist, transform_stamped = geometry_msgs.msg.TransformStamped):

    twist_vel = geometry_msgs.msg.Vector3Stamped()
    twist_rot = geometry_msgs.msg.Vector3Stamped()
    twist_vel.vector = twist.linear
    twist_rot.vector = twist.angular
    out_vel = tf2_geometry_msgs.do_transform_vector3(twist_vel, transform_stamped)
    out_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, transform_stamped)

    #Populate new twist message
    new_twist = geometry_msgs.msg.Twist()
    new_twist.linear = out_vel.vector
    new_twist.angular = out_rot.vector

    return new_twist
```

The main program. It creates a controller class object and handle the ROS thread until asked to shut off or when crashing.

```python
#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    back_and_forth_controller = BackAndForthController()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print "Shutting down"
        sys.exit(0)
```

## Next Tutorial
Let's now add more robots in the next tutorial: [Multi Robot Simulation (Python)](/documentation/tutorials/3_multi_robot_simulation_python.md)

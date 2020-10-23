# Tutorial 3 - Multi-robot simulation (Python)

In this tutorial we are spicing up things by adding more robot to our simulation.

## Prerequisites
The SML Nexus packages must have been installed and the simulation setup as detailed in the [first tutorial](/documentation/tutorials/1_install_and_bringup_simulation.md) and it is recommended to follow the [previous tutorial](/documentation/tutorials/2_simple_controller_simulation_python.md) first for a better understanding.

## Bringup the simulation and the controllers

Start the following launch file to bringup Gazebo and spawn three nexus robot:

`roslaunch sml_nexus_tutorials 3_multi_robot_simulation_python.launch`

After a few seconds, you should be able to see the intial robot moving back and forth between two locations and the two new robots trying to keep same position and heading relative to this one.

<a href="url"><img src="/documentation/pictures/tutorial_3_picture_1.png" align="center" height="539" width="1000"/></a>

## Follower controller
Two robots have been added, each one running its own follower controller. The goal is not to make for a complex formation control but simply to demonstrate the use of multiple nexus robot at the same time.

If you look at the ROS node list you will see all the new nodes that are running. In another termnal run:

`rosnode list`

```
/gazebo
/gazebo_gui
/nexus1/back_and_forth_controller
/nexus1/robot_state_publisher
/nexus2/follower_controller
/nexus2/robot_state_publisher
/nexus3/follower_controller
/nexus3/robot_state_publisher
/qualisys
/rosout
```

As you can see, most nodes are now preceded by a namespace (nexus1, nexus2 or nexus3). Namespaces are very convenients when running multiple time the same node with different robots as they allow for automatic renaming of topics (by adding the namespace in front of it). We will see more in details how to use namespaces in the section about the launch file.

We can get more information on the follower controller node by using the rosnode command:

`rosnode info /nexus2/follower_controller`

We can see that the node is subscribed to `/tf`, the two robot poses `/qualisys/nexus1/pose` and `/qualisys/nexus2/pose`, and publishes to `/nexus2/cmd_vel`. The node itself normally publishes to `/cmd_vel` but the namespace is added. This controller uses the robot position (nexus2 is this example) and the tracked object position (here nexus1) to generate a velocity command.

Using `rqt_graph` gives a good overview of the namespaces and topics:

`rosrun rqt_graph rqt_graph`

<a href="url"><img src="/documentation/pictures/tutorial_3_picture_2.png" align="center" height="457" width="1000"/></a>

The namespaces are represented as rectangular boxes containing all the nodes on the same namespace.

Use CTRL+C to stop the program.


### Examining the multi-robot launch file

To understand how everything is launched, take a quick look into [sml_nexus_tutorials/launch/3_multi_robot_simulation_python.launch.launch](sml_nexus_tutorials/launch/3_multi_robot_simulation_python.launch)

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

  <group ns="nexus1">
    <!-- Load robot description -->
    <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />
    <!-- Spawn the robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model nexus1 -param robot_description -x 0 -y 0 -z 0.5" />
      <!-- Controller node -->
    <node name="back_and_forth_controller" pkg="sml_nexus_tutorials" type="back_and_forth_controller.py" output="screen" />
  </group>

  <!-- Nexus 2  -->
  <group ns="nexus2">
    <!-- Load robot description -->
    <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />
    <!-- Spawn the robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model nexus2 -param robot_description -x 0 -y -2 -z 0.5" />
    <!-- Follower controller node -->
    <node name="follower_controller" pkg="sml_nexus_tutorials" type="follower_controller.py" output="screen" >
      <param name="robot_name" value="nexus2"/>
      <param name="tracked_object" value="nexus1"/>
      <rosparam param="tracking_offset">[0, -1, 0]</rosparam>
    </node>
  </group>

  <!-- Nexus 3  -->
  <group ns="nexus3">
    <!-- Load robot description -->
    <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />
    <!-- Spawn the robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model nexus3 -param robot_description -x -1 -y 2 -z 0.5" />
    <!-- Follower controller node -->
    <node name="follower_controller" pkg="sml_nexus_tutorials" type="follower_controller.py" output="screen" >
      <param name="robot_name" value="nexus3"/>
      <param name="tracked_object" value="nexus1"/>
      <rosparam param="tracking_offset">[0, 1, 0]</rosparam>
    </node>
  </group>

  <!-- Motion capture system simulation -->
  <include file="$(find mocap_simulator)/launch/qualisys_simulator.launch" />

</launch>
```

The main difference with the previous single robot launch file is the use of group with namespace. The robot model description, robot spawner and controller are all included into group with specified namespace.

```XML
  <!-- Nexus 2  -->
  <group ns="nexus2">
    <!-- Load robot description -->
    <include file="$(find sml_nexus_description)/launch/sml_nexus_description.launch" />
    <!-- Spawn the robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model nexus2 -param robot_description -x 0 -y -2 -z 0.5" />
    <!-- Follower controller node -->
    <node name="follower_controller" pkg="sml_nexus_tutorials" type="follower_controller.py" output="screen" >
      <param name="robot_name" value="nexus2"/>
      <param name="tracked_object" value="nexus1"/>
      <rosparam param="tracking_offset">[0, -1, 0]</rosparam>
    </node>
  </group>
```

Please note the use of private parameters in the controller node. They help make the code more modular and easier to tune than hard-coded constants.


### Examining the follower controller code

Let's look at the code in [sml_nexus_tutorials/src/follower_controller.py](/sml_nexus_tutorials/src/follower_controller.py) line by line.

First part of the code is just about importing the needed libraries.

```python
#!/usr/bin/env python
#=====================================
#     Follower controller for the        
#          SML Nexus robot
#               ----
#   Listen to both the robot pose 
# and another tracked object from
#    the mocap system and output a
#   velocity command to drive the
#    robot to the follow the object
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
class FollowerController():
```

In the constructor of the class, the ROS node is initialized. This is needed before accessing any ROS time function or subcribing/publishing on a ROS topic.

```python
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('follower_controller')
```


Constants and variables are declared and initialized. This time we use ROS parameters to make the controller more flexible. Values are retrived from the parameter server using the `rospy.get_param()` function. The `~` in front of the parameter name specifies a private parameter: the name of the node is added in front of the parameter name. Instead of looking at `/robot_name` the node will look the parameter `/follower_controller/robot/name`. More details on how to assign parameter values in the launch file section of this tutorial.


```python
        #Get tracked object name from parameters
        robot_name = rospy.get_param('~robot_name')

        #Get tracked object name from parameters
        tracked_object = rospy.get_param('~tracked_object')

        #Get offset from tracked object
        tracking_offset = rospy.get_param('~tracking_offset')

        #Controller gains (for x, y, heading)
        gains = (2, 2, 2)
```

The robot pose variable and the tracked object pose variable are declared here as a ROS Pose Stamped message ([definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)). This is the same message type received from the robot position feedback. The controller gains are also specified here (though one good practice is to have them as parameters)

Like for the previous controller, a "last received feedback time" variable is declared here as [ROS time instance](http://wiki.ros.org/rospy/Overview/Time). Together with the timeout, it is used to stop sending commands if we lose the motion capture feedback for either the robot or the tracked object. The two booleans "init_robot" and "init_track" are used to store that information.

```python
        #Init robot pose
        self.robot_pose = geometry_msgs.msg.PoseStamped()

        #Init tracked object pose
        self.tracked_object_pose = geometry_msgs.msg.PoseStamped()

        #Init last received pose time
        self.last_received_robot_pose = rospy.Time()

        #Init last received pose time
        self.last_received_tracked_pose = rospy.Time()

        #Booleans to check is positions have been received
        init_pose = False
        init_track = False
```

Then, our feedback subscribers are setup. Like in the previous tutorial, they subscribe to the motion capture system with ROS Pose Stamped message ([definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)). However this time one subscriber is for our robot position (`"/qualisys/"+robot_name+"/pose`) and one is for the tracked object (`"/qualisys/"+tracked_object+"/pose"`). Note the use of the robot name and tracked object name as variable (initialized by parameters).

Two callback functions (`self.robot_pose_callback` and `self.tracked_object_pose_callback`) are used and will be called everytime a new message is published. Those function declarations are done later in the code.

The command publisher is also declared here. When called, it will publish on the robot command topic (`"cmd_vel"`) a ROS Twist message (velocity message, [definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html))

**Important point:** Please note that the `/` before a topic name is important for namespace. If we use a slash, the path is absolute and won't change whatever namespace is used (as it is the case here for the subscription to the mocap feedback). Without a slash the path is relative and the namespace will be added in front of it (as it is the case here for the velocity command).

For more information about ROS subcribers/publishers, you can take a look at the ROS Python tutorial [Writing a Simple Publisher and Subscriber](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)

```python
        #Setup robot pose subscriber
        rospy.Subscriber("/qualisys/"+robot_name+"/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback)

        #Setup tracked object pose subscriber
        rospy.Subscriber("/qualisys/"+tracked_object+"/pose", geometry_msgs.msg.PoseStamped, self.tracked_object_pose_callback)

        #Setup velocity command publisher
        vel_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=100)
```

Same as last time: the robot pose and goal pose are expressed in the motion capture system frame of reference but the robot command has to be sent in the robot reference frame. Hopefully the motion capture system node also provides the transform between its frame and the tracked subject frame. ROS provides a framework for transforms (called tf2, [here for more information](http://wiki.ros.org/tf2/Tutorials)). 

The program will wait for a transform to be available before continuing.

```python
        #Setup transform subscriber
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        #Wait for transform to be available
        while not tf_buffer.can_transform('mocap', robot_name+'/base_link', rospy.Time()):
            rospy.loginfo("Wait for transform to be available")
            rospy.sleep(1)
```

Lastly, the command message variable is declared as a ROS Twist message ([definition here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html))

```python
        #Create a ROS Twist message for velocity command
        vel_cmd_msg = geometry_msgs.msg.Twist()
```

The main loop frequency is regulated using a ROS rate in the same way as in the previous tutorial. In addition we also print some information about the controller at initialization.

The last received pose for both the tracked object and the robot is tested for timeout and if everything is okay the loop will proceed.

```python
        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)

        rospy.loginfo("Follower controller Initialized for "+robot_name+
                      " tracking "+tracked_object+
                      " with offset x:"+str(tracking_offset[0])+
                      ", y:"+str(tracking_offset[1])+
                      ", heading:"+str(tracking_offset[2])
                      )

        while not rospy.is_shutdown():
            #Run controller if robot position and tracked object position have been received
            now  = rospy.Time.now()
            init_pose = (now.to_sec() < self.last_received_robot_pose.to_sec() + timeout)
            init_track = (now.to_sec() < self.last_received_tracked_pose.to_sec() + timeout)

            if init_pose and init_track:
```

Like in the previous controller, error is computed. This time however the difference between the tracked object (plus an offset) and the robot position is used. Both positions are retrived from the mocap feedback callbacks.

In a ROS Pose Stamped message, orientation is defined as a quaternion and therefor needs to be converted to get the robot heading.

Command is computed using the errors anf gains.

```python
                #-----------------------------------
                # Compute error on x and y position
                #-----------------------------------
                error_x = (self.tracked_object_pose.pose.position.x + tracking_offset[0]) - self.robot_pose.pose.position.x 
                error_y = (self.tracked_object_pose.pose.position.y + tracking_offset[1]) - self.robot_pose.pose.position.y

                #-----------------------
                # Compute heading error
                #-----------------------
                #Get euler angle from robot pose quaternion
                (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                            self.robot_pose.pose.orientation.y,
                                                            self.robot_pose.pose.orientation.z,
                                                            self.robot_pose.pose.orientation.w])

                #Get euler angle from robot pose quaternion
                (track_roll, track_pitch, track_yaw) = euler_from_quaternion([self.tracked_object_pose.pose.orientation.x,
                                                                              self.tracked_object_pose.pose.orientation.y,
                                                                              self.tracked_object_pose.pose.orientation.z,
                                                                              self.tracked_object_pose.pose.orientation.w])

                error_heading = (track_yaw + tracking_offset[2]) - yaw

                #----------------
                # Compute output
                #----------------
                vel_cmd_msg.linear.x = error_x * gains[0]
                vel_cmd_msg.linear.y = error_y * gains[1]
                vel_cmd_msg.angular.z = error_heading * gains[2]
```

If the last feedback as not been received recenlty  (if not init_track and init_pose), the command is set to zero.

```python
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

The rate object created before provides a `sleep()` function that will wait enough time for the frequency loop to be respected.

```python
            #---------------------------------
            # Sleep to respect loop frequency
            #---------------------------------
            r.sleep()

```

The pose callbacks used to receive the motion capture system feedback. We have an additional callback for the tracked object position compared to the last tutorial.

```python
    #=====================================
    #          Callback function 
    #      for robot pose feedback
    #=====================================
    def robot_pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.robot_pose = pose_stamped_msg

        #Save when last pose was received
        self.last_received_robot_pose = rospy.Time.now()

    #=====================================
    #          Callback function 
    #  for tracked object pose feedback
    #=====================================
    def tracked_object_pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.tracked_object_pose = pose_stamped_msg

        #Save when last pose was received
        self.last_received_tracked_pose = rospy.Time.now()
```

The `transform_twit()` function definition, identical to the one from the previous tutorial. The function is used to convert the velocity command to the robot reference frame.

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
    follower_controller = FollowerController()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print "Shutting down"
        sys.exit(0)
```


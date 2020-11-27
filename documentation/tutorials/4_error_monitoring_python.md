# Tutorial 4 - Error Monitoring (Python)

One key aspect when designing a controller is the ability to plot and analyse output and error. In this tutorial a few ways to plot and save data will be presented.

## Prerequisites
The SML Nexus packages must have been installed and the simulation setup as detailed in the [first tutorial](/documentation/tutorials/1_install_and_bringup_simulation.md). It is also advised to have followed the previous tutorials about multi[-robot simulation](/documentation/tutorials/3_multi_robot_simulation_python.md).

## Broadcasting error (and other information)
For this tutorial we will be using the follower controller from the previous multi-robot simulation. Let's take a look at the code in [sml_nexus_tutorials/src/follower_controller_with_error_pub.py](/sml_nexus_tutorials/src/follower_controller_with_error_pub.py).

The controller is essentially the same with the addition of the addition of ROS publishers at initialization:

```Python
        #Setup error publishers
        error_x_pub = rospy.Publisher("error/x", std_msgs.msg.Float32, queue_size=100)
        error_y_pub = rospy.Publisher("error/y", std_msgs.msg.Float32, queue_size=100)
        error_heading_pub = rospy.Publisher("error/heading", std_msgs.msg.Float32, queue_size=100)
```

Error on x, y and heading is published at the same as the output:

```Python
                #---------------
                # Publish error
                #---------------
                error_x_pub.publish(std_msgs.msg.Float32(error_x))
                error_y_pub.publish(std_msgs.msg.Float32(error_y))
                error_heading_pub.publish(std_msgs.msg.Float32(error_heading))

                #----------------
                # Compute output
                #----------------
                vel_cmd_msg.linear.x = error_x * gains[0]
                vel_cmd_msg.linear.y = error_y * gains[1]
                vel_cmd_msg.angular.z = error_heading * gains[2]
```

Here, a simple float32 message is used but depending on the controller and needed feedback, it's possible to send any type of ROS message: position with covariance, velocity, force/torque...

## Bringup the simulation and the controller

Let's run the simulation by using the following command:

`roslaunch sml_nexus_tutorials 4_error_monitoring_python.launch`

You should see the same simulation and behavior as the previous tutorial.

### Command Line Interface

In another terminal, run the following command to get the topic list:

`rostopic list`

The list get quite long but you should notice the following topics for the two follower controllers:
```
/nexus2/error/heading
/nexus2/error/x
/nexus2/error/y
/nexus3/error/heading
/nexus3/error/x
/nexus3/error/y
```

The value can simply be printed on screen by using `rostopic echo`, for example to get error in heading of nexus2:

`rostopic echo /nexus2/error/heading`

### PlotJuggler

In case plotting is needed, PlotJuggler is a very powerful tool that can be used.

Install PlotJuggler by running the following command (replacing <your_ros_distro> by your ROS distro):

`sudo apt-get install ros-<your_ros_distro>-plotjuggler`

And run it:

`plotjuggler`

On the main window, click "Streaming > Start: ROS Topic Subscriber".

<a href="url"><img src="/documentation/pictures/tutorial_4_picture_1.png" align="center" height="535" width="1000"/></a>

Then select the topics you want to plot, in our case we want to see the error on x and y for both followers (nexus2 and nexus3).

<a href="url"><img src="/documentation/pictures/tutorial_4_picture_2.png" align="center" height="511" width="800"/></a>

Then drag and drop the topics you want to display from the left to the main window (in our case all the topics ending up with x/data or y/data).

<a href="url"><img src="/documentation/pictures/tutorial_4_picture_3.png" align="center" height="535" width="1000"/></a>

The plot is displayed in real-time. You can adjust parameters, add more windows, save the data or even display x-y data as one position on a 2D plot. For more information please take a look at [PlotJuggler documentation](https://www.plotjuggler.io/)

<a href="url"><img src="/documentation/pictures/tutorial_4_picture_4.png" align="center" height="535" width="1000"/></a>

### MATLAB

MATLAB is a useful tool for data processing. A ROS toolbox exists and it is possible to listen to ROS topics from MATLAB.

For instruction on how to install the toolbox, please refer to the [toolbox documentation](https://www.mathworks.com/help/ros/ug/get-started-with-ros.html)

All scripts presented below are available [here](/resources/matlab).

The following script listens to the feedback and save the data:

```Matlab
clear;
clear global;
%% ================================
%          Connect to ROS
%  ================================
rosshutdown
pause(0.5);
rosinit('127.0.0.1',11311);

% Setup subscribers;
error_nexus2_x_sub = rossubscriber('/nexus2/error/x', 'std_msgs/Float32');
error_nexus2_x_message = rosmessage(error_nexus2_x_sub);
error_nexus2_y_sub = rossubscriber('/nexus2/error/y', 'std_msgs/Float32');
error_nexus2_y_message = rosmessage(error_nexus2_y_sub);
error_nexus2_heading_sub = rossubscriber('/nexus2/error/heading', 'std_msgs/Float32');
error_nexus2_heading_message = rosmessage(error_nexus2_heading_sub);
error_nexus3_x_sub = rossubscriber('/nexus3/error/x', 'std_msgs/Float32');
error_nexus3_x_message = rosmessage(error_nexus3_x_sub);
error_nexus3_y_sub = rossubscriber('/nexus3/error/y', 'std_msgs/Float32');
error_nexus3_y_message = rosmessage(error_nexus3_y_sub);
error_nexus3_heading_sub = rossubscriber('/nexus3/error/heading', 'std_msgs/Float32');
error_nexus3_heading_message = rosmessage(error_nexus3_heading_sub);

% Init array
error_nexus2 = zeros(3,1);
error_nexus3 = zeros(3,1);
index = 1;

% Set rate
dt = 100;
r = rosrate(dt);

% Loop at desired frequency:
while(1)
   error_nexus2_x_message = receive(error_nexus2_x_sub);
   error_nexus2_y_message = receive(error_nexus2_y_sub);
   error_nexus2_heading_message = receive(error_nexus2_heading_sub);
   error_nexus3_x_message = receive(error_nexus3_x_sub);
   error_nexus3_y_message = receive(error_nexus3_y_sub);
   error_nexus3_heading_message = receive(error_nexus3_heading_sub);   
   
   error_nexus2(1,index) = error_nexus2_x_message.Data;
   error_nexus2(2,index) = error_nexus2_y_message.Data;
   error_nexus2(3,index) = error_nexus2_heading_message.Data;
   
   error_nexus3(1,index) = error_nexus3_x_message.Data;
   error_nexus3(2,index) = error_nexus3_y_message.Data;
   error_nexus3(3,index) = error_nexus3_heading_message.Data;
   
   index = index + 1;
    
   % Wait at end of loop to achieve required frequency
   waitfor(r);
end
```

**Please note this is an inefficient way of doing it.** Indeed we have 3 topics per robot while it would make more sense to publish the error on x, y and heading in a pose message for example, and send it on one topic only.

Moreover topic messages are retrieved at a fixed frequency: if the feedback is sent faster we loose information, if the feedback is sent slower we add the previous message on the topic. Below is presented an alternative method with callbacks to avoid these issues.

#### MATLAB Callbacks
**Use callbacks if possible**

Below is an example on how to listen for nexus2 error on x using callback:

```Matlab
clear;
clear global;
%% ================================
%          Connect to ROS
%  ================================
rosshutdown
pause(0.5);
rosinit('127.0.0.1',11311);

global error_nexus2_x
error_nexus2_x = [];

% Setup subscribers;
error_nexus2_x_sub = rossubscriber('/nexus2/error/x', @ErrorNexus2XCallback);

while(1) 
  pause(0.1);
end
```

An external callback function, `ErrorNexus2XCallback()` is called everytime a new message is sent on the topic. It simply stores the received value at the end of an array. Below the function:

```Matlab
function ErrorNexus2XCallback(~, message)
  global error_nexus2_x;
  error_nexus2_x(end+1) = message.Data;
end
```
## Next Tutorial
You are now ready to learn how to implement your controller on the real robot in the next tutorial: [Running the Real Robot](/documentation/tutorials/5_running_the_real_robot.md)

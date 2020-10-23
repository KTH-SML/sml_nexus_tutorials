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
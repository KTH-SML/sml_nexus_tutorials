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
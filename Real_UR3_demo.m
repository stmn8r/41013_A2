
close all;
clc;

%% Initialising connection to the ROS computer from Matlab.

rosshutdown;
% rosinit('192.168.0.100'); % If unsure, please ask a tutor
rosinit('192.168.27.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');

%% Getting the current joint state from the real robot.

jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Checking latest message to ensure valid connection to robot
jointStateSubscriber.LatestMessage

% Establishing Open and Close service calls
openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

%% Creating a variable with joint names associated with real joints

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%% Sending joint angles to the real robot

[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory'); % Creating our client and defining a goal.
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 7; % This is how many seconds the movement will take

%% Position 1
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [ deg2rad(5.38), deg2rad(-69.64), deg2rad(129.88), deg2rad(-51.58), deg2rad(5.01), deg2rad(351.7)]; % % Joint Configurations
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Opening and closing gripper

openService.call(); % Opening in case it starts closed.
pause(5);
closeService.call(); % Closing gripper to hold bottle.
%% Copy Paste the following move commands for each joint configuration

%% Position 2
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [ deg2rad(4.27), deg2rad(-160.38), deg2rad(27.68), deg2rad(-17.83), deg2rad(-5.77), deg2rad(340.59)]; % Joint Configurations
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Position 3
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [ deg2rad(4.3), deg2rad(-36.48), deg2rad(33.54), deg2rad(-19.07), deg2rad(144.73), deg2rad(336.65)]; % Joint Configurations
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Position 4
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [ deg2rad(4.31), deg2rad(-53.76), deg2rad(-138.84), deg2rad(32.09), deg2rad(14.53), deg2rad(336.98)]; % Joint Configurations
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Position 5 - Returning to original position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [ deg2rad(5.38), deg2rad(-69.64), deg2rad(129.88), deg2rad(-51.58), deg2rad(5.01), deg2rad(351.7)]; % % Joint Configurations
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);
%% Opening the gripper to release drink 1 and pick up drink 2

openService.call(); % Releasing drink 1
pause(5); % Pausing for user to replace drink 1 with drink 2
%closeService.call(); % Closing gripper to hold drink 2

% Next, Robot can either repeat drink 1 movements for drink 2, or drink 2
% can go through different movements.

%% Service calls
%openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
%closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

%openService.call();
%pause(3);
%closeService.call();

%% Notes
% Teach pendant is in degrees, whereas Matlab code is in radians. Bear that
% in mind.
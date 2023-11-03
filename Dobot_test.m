%% To record
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2); % Allow some time for a message to appear

trajectories = {};
i = 0;

while true
    i = i+1;
    status = input("capture...", 's');
    currentJointState = jointStateSubscriber.LatestMessage.Position; % Get the latest message
    disp(currentJointState);
    trajectories{1,i} = currentJointState;
    if status == "s"
        trajectories{2,i} = 1;
    end
    if status == "r"
        trajectories{2,i} = 0;
    end
    if status == "exit"
        trajectories{2,i} = 0;
        disp(trajectories);
        break;
    end
end

%% To edit a prerecorded step
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2); % Allow some time for a message to appear


i;

i = input("enter step you want to change: ");
    
    status = input("capture...", 's');
    currentJointState = jointStateSubscriber.LatestMessage.Position; % Get the latest message
    disp(currentJointState);
    trajectories{1,i} = currentJointState;
    if status == "s"
        trajectories{2,i} = 1;
    end
    if status == "r"
        trajectories{2,i} = 0;
    end
    
   
  

%% run to view a certain position
[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');

i = input("enter step you want to view: ");
    disp(['trajectory: ', num2str(i)]);
    jointTarget = trajectories{1,i};
    

    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(0.5);
    % suction change happens at end of movement
    toolStateMsg.Data = [trajectories{2,i}]; % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);
    pause(1);



%% load existing trajectores
load("dobot_traj_examples.mat");
%%
[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');

pause(3);
for i = 1:length(trajectories)
    disp(['trajectory: ', num2str(i)]);
    jointTarget = trajectories{1,i};
    

    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(0.5);
    % suction change happens at end of movement
    toolStateMsg.Data = [trajectories{2,i}]; % Send 1 for on and 0 for off
    
    send(toolStatePub,toolStateMsg);
    pause(1);
end



%%
% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

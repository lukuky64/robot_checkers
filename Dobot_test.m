%% To record
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2); % Allow some time for a message to appear

trajectores = {};
i = 0;

while true
    i = i+1;
    status = input("capture...", 's');
    currentJointState = jointStateSubscriber.LatestMessage.Position; % Get the latest message
    disp(currentJointState);
    trajectores{i} = currentJointState;
    if status == "exit"
        disp(trajectores);
        break;
    end
end


%% load existing trajectores
load("dobot_traj_examples.mat");
%%

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");


for i = 1:length(trajectores)
    disp(['trajectory: ', num2str(i)]);
    jointTarget = trajectores{i};

    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(4);
end



%%
% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

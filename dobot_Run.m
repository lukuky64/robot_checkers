function[]= dobot_Run(data)

    %this function accepts a 2d data matrix that contains joint trajectories
    %in the first row and suction toggle in the 2nd row. Change in suction
    %occurs at the end of each trajectory move so should be included in the
    %same column as the trajectory that you want it to happen after. for
    %example:

    %q1=[n;n;n;n]
    
    % data{q1, q2, q3, q4;
    %       1,  0,  0,  1};
    
    % if this data was passed to the function then the suction would turn on
    % after moving to q1, then turn off after it has moved to q4
    
 
    
    trajectories = data;
    suction = 0; % 1 = on, 0 = off
    
    
    
    
    % setup ROS publisher
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    
    pause(3); %wait for message to send



    for i = 1:length(trajectories)
        disp(['trajectory: ', num2str(i)]);
        jointTarget = trajectories{1,i};
        
        
    %% move robot
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);
    
        pause(0.5); % delay to allow robot to move before changing suction state
    
     %% Suction
    
      %toggles suction state;
        if trajectories{2,i} == 1
            if suction == 0
                suction = 1;
            else 
                suction = 0;
            end
        end
      
        toolStateMsg.Data = [suction]; 
        send(toolStatePub,toolStateMsg);
    
        pause(1); % change this to make robot move faster
    end
end
classdef Dobot
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
    end

    methods(Static)
        function[]= Run(data)

    %this function accepts a 2d data matrix that contains joint trajectories
    %in the first row and suction toggle in the 2nd row. Change in suction
    %occurs at the end of each trajectory move so should be included in the
    %same column as the trajectory that you want it to happen after. for
    %example:

    % q is a 4x1 matrix
    
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

        function[]=camera_Pose()

         % setup ROS publisher
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    
    pause(3); %wait for message to send
         t = load("camera_Pos.mat");
         jointTarget = t.trajectories{1,1};
         
     %% move robot
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);
    
        pause(0.5); % delay to allow robot to move before changing suction state
    
        end

        function trajectories = Record()
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
        end
        function[] = edit_Record()
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
        end
        function[] = run_Record(data)
                     %%

            trajectories = data;
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
        end

        function[] = view_Position()
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
        end
        function[] = tool_State(input)
                      %%
            % Turn on the tool
            [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
            toolStateMsg.Data = [input]; % Send 1 for on and 0 for off 
            send(toolStatePub,toolStateMsg);
  
        end
    end
end
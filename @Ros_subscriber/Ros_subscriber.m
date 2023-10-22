classdef Ros_subscriber < handle
    properties
        subscriber;
        data;
        keepRunning = true;
        timerObj;  % Timer object for checking ROS subscriber
    end
    
    methods
        function obj = Ros_subscriber(topicName, messageType)
            try
                rosnode('list');
            catch
                setenv('ROS_MASTER_URI', 'http://172.30.31.249:11311/'); %http://localhost:11311/
                rosinit;
            end
            
            obj.subscriber = rossubscriber(topicName, messageType, @(src, msg) obj.callback(src, msg));
            obj.data = {};
            
            % Create a timer object
            obj.timerObj = timer('TimerFcn', @(~,~) obj.checkSubscriber(), ...
                                 'Period', 0.1, ...
                                 'ExecutionMode', 'fixedRate');
        end
        
        function callback(obj, ~, msg)  % ~ to ignore the src parameter
            %disp('Callback invoked'); % debugging message
            obj.data{end+1, 1} = msg.Data;
        end
        
        function checkSubscriber(obj)
            % This function can be empty as long as the callback function in the ROS subscriber is set to update obj.data
            % You can also put additional code here to process the received data
        end
        
        function run(obj)
            obj.keepRunning = true;
            
            % Start the timer
            start(obj.timerObj);
        end
        
        function stop(obj)
            obj.keepRunning = false;
            
            % Stop the timer
            stop(obj.timerObj);
        end
    end
end

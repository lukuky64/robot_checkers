% this class is used to subscribe to a ROS topic and store the received data

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
            
        end
        
        function callback(obj, ~, msg)  % ~ to ignore the src parameter
            %disp('Callback invoked'); % debugging message
            obj.data{end+1, 1} = msg.Data;
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

classdef Blackout < handle
    %BLACKOUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        subscriber;
        emergency_state;
    end
    
    methods
        % input collision and ros listener (eStop)
        function obj = Blackout()
            %[obj.vertex, obj.faces, obj.faceNormals] = checkCollision.createCollisionPrisms(prisms);
            %obj.collision_state = false;
            obj.emergency_state = false;
            obj.ros_subscriber();
            
        end

        function ros_subscriber(obj)
            try
                rosnode('list');
            catch
                setenv('ROS_MASTER_URI', 'http://172.30.31.249:11311/'); %http://localhost:11311/
                rosinit;
            end
            
            obj.subscriber = rossubscriber('/eStop_state', 'std_msgs/Int32', @(src, msg) obj.callback(src, msg));
        end

        % change state of emergency variable based on ros node data
        function callback(obj, ~, msg)  % ~ to ignore the src parameter
            if msg.Data == 0
                obj.emergency_state = false; 
            end

            if msg.Data == 1
                obj.emergency_state = true;
            end
        end
        
        % return if the blackout has been acivated
        function status = activated(obj)
            status = obj.emergency_state;
            return 
        end
    end
end

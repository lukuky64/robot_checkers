classdef Player
    % Executive management of motion-planning operations of robot.

    properties
        robot
        mp
    end

    methods
        function self = Player(motionPlanner, robot, Tboard, squareSize, Tbase)
            self.robot = robot;
            self.mp = MotionPlanner(robot, Tboard, squareSize, Tbase);
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end
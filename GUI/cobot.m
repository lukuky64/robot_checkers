classdef cobot < handle  % Subclass 'handle' to make it a reference object
    properties
        robotModel;  % Stores the SerialLink object
        workspace;
        scale;
    end

    methods
        % Constructor
        function obj = cobot()
            % Define the DH Parameters to create the kinematic model
            link(1) = Link('d', 0.1739, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-165 165]));
            link(2) = Link('d', 0.08878, 'a', 0.135, 'alpha', 0,'offset',pi/2, 'qlim', deg2rad([-165 165]));
            link(3) = Link('d', -0.08878, 'a', 0.120, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(4) = Link('d', 0.08878, 'a', 0.095, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(5) = Link('d', 0.06550, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(6) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-175 175]));

            % Generate the model
            obj.robotModel = SerialLink([link(1), link(2), link(3), link(4), link(5), link(6)], 'name', 'myRobot');

            % Set workspace and scale properties
            obj.workspace = [-0.5 0.5 -0.5 0.5 -0.01 0.8];
            obj.scale = 0.5;

            % Define joint angles
            q = [0 0 0 0 0 0];
            obj.plotRobot(q);
        end

        % Method to plot the robot
        function plotRobot(obj, jointAngles)
            obj.robotModel.plot(jointAngles, 'workspace', obj.workspace, 'scale', obj.scale);
        end

        % Method to get forward kinematics
        function T = getForwardKinematics(obj, jointAngles)
            T = obj.robotModel.fkine(jointAngles);
        end

        % Method to get current joint positions (if needed)
        function q = getCurrentJointPositions(obj)
            q = obj.robotModel.getpos();
        end
    end
end

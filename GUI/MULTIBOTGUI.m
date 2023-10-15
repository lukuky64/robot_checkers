% To Do:
% Add cartesian (tc = ctraj(T0, T1, n)) control also

classdef MULTIBOTGUI < handle

    properties
        robot_ = {};  % using cells
        currentJointPos_ = {};
        nextJointPos_ = {};
        guiHandles = struct();  % GUI handles
    end

    methods
        function obj = MULTIBOTGUI(bots)
            for i = 1:length(bots)
                obj.robot_{i} = bots{i};
            end
            obj.initialise();
            obj.createGUI();
        end

        function initialise(obj)

            for i = 1:length(obj.robot_)
                obj.currentJointPos_{i} = obj.robot_{i}.model.getpos();
                qlim = obj.robot_{i}.model.qlim;
                goal = mean(qlim, 2);
                obj.moveRobot(goal, i, 2);
                obj.currentJointPos_{i} = obj.robot_{i}.model.getpos();
            end
        end

        function createGUI(obj)
            obj.guiHandles.fig = gcf;
            set(obj.guiHandles.fig, 'Position', [600, 200, 800, 450]);
            ax = gca;
            ax.Position = [0.3 0.1 0.6 0.8];
            
            uicontrol('Style', 'text', 'Position', [30, 380, 100, 20], 'String', 'Robot ID:', 'HorizontalAlignment', 'left');
            
            obj.guiHandles.robotSelect = uicontrol('Style', 'popupmenu', 'String', num2str((1:length(obj.robot_))'), 'Position', [30, 350, 100, 30], 'Callback', @(src, event) obj.updateSliders());
            obj.updateSliders();
            
            obj.guiHandles.goButton = uicontrol('Style', 'pushbutton', 'String', 'GO', 'Position', [30, 20, 50, 30], 'Callback', @(src, event) obj.goButtonCallback());
            
            obj.guiHandles.liveCheckbox = uicontrol('Style', 'checkbox', 'Position', [90, 20, 50, 30], 'String', 'Live', 'Callback', @(src, event) obj.liveCheckboxCallback());
        end

        function updateSliders(obj)
            if isfield(obj.guiHandles, 'sliders')
                delete(obj.guiHandles.sliders);
                delete(obj.guiHandles.labels);
            end
            
            robotNum_ = get(obj.guiHandles.robotSelect, 'Value');
            qlim = obj.robot_{robotNum_}.model.qlim;
            currentJointAngles = obj.currentJointPos_{robotNum_};
            nJoints = size(qlim, 1);
            
            obj.guiHandles.sliders = gobjects(1, nJoints);
            obj.guiHandles.labels = gobjects(1, nJoints);
            
            uicontrol('Style', 'text', 'Position', [30, 330, 100, 20], 'String', 'Joint Angles:', 'HorizontalAlignment', 'left');
            
            for i = 1:nJoints
                obj.guiHandles.sliders(i) = uicontrol('Style', 'slider', 'Min', qlim(i, 1), 'Max', qlim(i, 2), 'Value', currentJointAngles(i), 'Position', [30, 300 - (i-1)*35, 100, 30]);
                obj.guiHandles.labels(i) = uicontrol('Style', 'text', 'Position', [140, 300 - (i-1)*35, 40, 30], 'String', num2str(get(obj.guiHandles.sliders(i), 'Value')));
                
                addlistener(obj.guiHandles.sliders(i), 'ContinuousValueChange', @(src, event) obj.sliderCallback(i, src));
            end
        end

        function liveCheckboxCallback(obj)
            if get(obj.guiHandles.liveCheckbox, 'Value')
                set(obj.guiHandles.goButton, 'Enable', 'off');
            else
                set(obj.guiHandles.goButton, 'Enable', 'on');
            end
        end

        function sliderCallback(obj, jointNum, src)
            set(obj.guiHandles.labels(jointNum), 'String', num2str(get(src, 'Value')));
            if get(obj.guiHandles.liveCheckbox, 'Value')
                [jointGoal, robot_ID] = obj.setJointsFromSliders();

                obj.moveRobot(jointGoal, robot_ID, 2);
            end
        end

        function [jointGoal, robot_ID] = setJointsFromSliders(obj)
            robotNum_ = get(obj.guiHandles.robotSelect, 'Value');
            nJoints = length(obj.guiHandles.sliders);
            jointGoal_ = zeros(1, nJoints);
            
            for i = 1:nJoints
                jointGoal_(i) = get(obj.guiHandles.sliders(i), 'Value');
            end

            jointGoal = jointGoal_;
            robot_ID = int32(robotNum_);
        end

        function goButtonCallback(obj)
            robotNum_ = get(obj.guiHandles.robotSelect, 'Value');
            nJoints = length(obj.guiHandles.sliders);
            jointGoal_ = zeros(1, nJoints);
            
            for i = 1:nJoints
                jointGoal_(i) = get(obj.guiHandles.sliders(i), 'Value');
            end
            
            obj.moveRobot(jointGoal_, robotNum_, 50);
        end

        function status = moveRobot(obj, jointGoal_, robotNum_, steps_)
            if robotNum_ > length(obj.robot_)
                status = false;
                return;
            end
            
            obj.nextJointPos_{robotNum_} = jointGoal_;
            q = jtraj(obj.currentJointPos_{robotNum_}, obj.nextJointPos_{robotNum_}, steps_);
            
            for i = 1:steps_
                obj.robot_{robotNum_}.model.animate(q(i,:));
                pause(0.01);
            end
            
            obj.currentJointPos_{robotNum_} = obj.nextJointPos_{robotNum_};
            status = true;
            return;
        end
    end
end

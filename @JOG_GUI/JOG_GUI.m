% To Do:
% Add cartesian (tc = ctraj(T0, T1, n)) control also

classdef JOG_GUI < handle

    properties
        robot_ = {};  % using cells
        currentJointPos_ = {};
        nextJointPos_ = {};
        guiHandles = struct();  % GUI handles
    end

    methods
        function obj = JOG_GUI(bots)
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
            
            uicontrol('Style', 'text', 'Position', [30, 340, 100, 20], 'String', 'Robot ID:', 'HorizontalAlignment', 'left');
            obj.guiHandles.robotSelect = uicontrol('Style', 'popupmenu', 'String', num2str((1:length(obj.robot_))'), 'Position', [30, 315, 100, 30], 'Callback', @(src, event) obj.updateSliders());
            obj.updateSliders();
            
            obj.guiHandles.goButton = uicontrol('Style', 'pushbutton', 'String', 'GO', 'Position', [30, 20, 50, 30], 'Callback', @(src, event) obj.goButtonCallback());
            
            obj.guiHandles.liveCheckbox = uicontrol('Style', 'checkbox', 'Position', [90, 20, 50, 30], 'String', 'Live', 'Callback', @(src, event) obj.liveCheckboxCallback());

            % Radio buttons for mode selection
            obj.guiHandles.modePanel = uibuttongroup('Position', [0.03 0.82 0.135 0.15]);  % Increase the height
            obj.guiHandles.jointMode = uicontrol('Style', 'radiobutton', 'String', 'Joint control', 'Position', [3 30 100 30], 'parent', obj.guiHandles.modePanel, 'Callback', @(src, event) obj.switchMode());  % Reposition
            obj.guiHandles.cartesianMode = uicontrol('Style', 'radiobutton', 'String', 'Cartesian control', 'Position', [3 0 100 30], 'parent', obj.guiHandles.modePanel, 'Callback', @(src, event) obj.switchMode());  % Reposition
        end

        function switchMode(obj)
            selectedMode = obj.guiHandles.modePanel.SelectedObject.String;
            
            if strcmp(selectedMode, 'Joint control')
                % Delete existing UI elements
                delete(obj.guiHandles.upArrow);
                delete(obj.guiHandles.downArrow);
                delete(obj.guiHandles.leftArrow);
                delete(obj.guiHandles.rightArrow);
                delete(obj.guiHandles.forwardArrow);
                delete(obj.guiHandles.backwardArrow);
                % ... (create UI elements for Joint angle mode)
                obj.updateSliders();
            elseif strcmp(selectedMode, 'Cartesian control')
                % Delete existing UI elements
                delete(obj.guiHandles.sliders);
                delete(obj.guiHandles.labels);
                % ... (create UI elements for Global cartesian mode)
                obj.createCartesianControls();
            end
        end

        function createCartesianControls(obj)
            disp("createCartesianControls");
            % Create arrows and associate them with callback functions
            obj.guiHandles.upArrow = uicontrol('Style', 'pushbutton', 'String', '^', 'Position', [125, 245, 30, 30], 'Callback', @(src, event) obj.moveCartesian('up'));
            obj.guiHandles.downArrow = uicontrol('Style', 'pushbutton', 'String', 'v', 'Position', [125, 215, 30, 30], 'Callback', @(src, event) obj.moveCartesian('down'));

            obj.guiHandles.leftArrow = uicontrol('Style', 'pushbutton', 'String', '<', 'Position', [30, 230, 30, 30], 'Callback', @(src, event) obj.moveCartesian('left'));
            obj.guiHandles.rightArrow = uicontrol('Style', 'pushbutton', 'String', '>', 'Position', [90, 230, 30, 30], 'Callback', @(src, event) obj.moveCartesian('right'));
            obj.guiHandles.forwardArrow = uicontrol('Style', 'pushbutton', 'String', '^', 'Position', [60, 260, 30, 30], 'Callback', @(src, event) obj.moveCartesian('forward'));
            obj.guiHandles.backwardArrow = uicontrol('Style', 'pushbutton', 'String', 'v', 'Position', [60, 200, 30, 30], 'Callback', @(src, event) obj.moveCartesian('backward'));
        end

        function moveCartesian(obj, direction, ~, ~)
            switch direction
                case 'up'
                    % move robot up

                case 'down'
                    % move robot down
                
                case 'left'
                    % move robot left

                case 'right'
                    % move robot right

                case 'forward'
                    % move robot forward

                case 'backward'
                    % move robot backward
            end
            % something else
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
        
            uicontrol('Style', 'text', 'Position', [30, 300, 100, 20], 'String', 'Joint Angles:', 'HorizontalAlignment', 'left');
        
            for i = 1:nJoints
                obj.guiHandles.sliders(i) = uicontrol('Style', 'slider', 'Min', qlim(i, 1), 'Max', qlim(i, 2), 'Value', currentJointAngles(i), 'Position', [30, 270 - (i-1)*35, 100, 30]);
                obj.guiHandles.labels(i) = uicontrol('Style', 'edit', 'Position', [140, 270 - (i-1)*35, 50, 30], 'String', num2str(get(obj.guiHandles.sliders(i), 'Value')), 'Callback', @(src, event) obj.textBoxCallback(i, src));
        
                addlistener(obj.guiHandles.sliders(i), 'ContinuousValueChange', @(src, event) obj.sliderCallback(i, src));
            end
        end

        function textBoxCallback(obj, jointNum, src)
            newValue = str2double(get(src, 'String'));
            set(obj.guiHandles.sliders(jointNum), 'Value', newValue);
            if get(obj.guiHandles.liveCheckbox, 'Value')
                [jointGoal, robot_ID] = obj.setJointsFromSliders();
                obj.moveRobot(jointGoal, robot_ID, 2);
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

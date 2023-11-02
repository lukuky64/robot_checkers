% To Do:
% Add cartesian (tc = ctraj(T0, T1, n)) control also

classdef JOG_GUI < handle

    properties
        robot_ = {};  % using cells
        currentJointPos_ = {};
        nextJointPos_ = {};
        guiHandles = struct();  % GUI handles
        estop_ = false;  % e-stop state: false means no e-stop, true means e-stop active

        % obstacles
        vertex;
        faces;
        faceNormals;

    end

    methods
        function obj = JOG_GUI(bots)
            for i = 1:length(bots)
                obj.robot_{i} = bots{i};
            end
            obj.createObstacles();
            obj.initialise();
            obj.createGUI();
        end

        function createObstacles(obj)
            prisms = {
                [-0.15, -0.15,  0  ], [0.15, 0.15, 0.05];  % checkerboard
                [-0.6, -0.6, -0.002], [0.6, 0.6, -0.001]}; % ground plane, has to be slightly below 0 to not trigger from the base
            
            [obj.vertex, obj.faces, obj.faceNormals] = checkCollision.createCollisionPrisms(prisms, 0.01);
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
            obj.guiHandles.robotSelect = uicontrol('Style', 'popupmenu', 'String', num2str((1:length(obj.robot_))'), 'Position', [30, 315, 100, 30], 'Callback', @(src, event) obj.switchMode());
            obj.updateSliders();
            
            obj.guiHandles.goButton = uicontrol('Style', 'pushbutton', 'String', 'GO', 'Position', [30, 20, 50, 30], 'Callback', @(src, event) obj.goButtonCallback());
            
            obj.guiHandles.liveCheckbox = uicontrol('Style', 'checkbox', 'Position', [90, 20, 50, 30], 'String', 'Live', 'Callback', @(src, event) obj.liveCheckboxCallback());

            % Radio buttons for mode selection
            obj.guiHandles.modePanel = uibuttongroup('Position', [0.03 0.82 0.135 0.15]);  % Increase the height
            obj.guiHandles.jointMode = uicontrol('Style', 'radiobutton', 'String', 'Joint control', 'Position', [3 30 100 30], 'parent', obj.guiHandles.modePanel, 'Callback', @(src, event) obj.switchMode());  % Reposition
            obj.guiHandles.cartesianMode = uicontrol('Style', 'radiobutton', 'String', 'Cartesian control', 'Position', [3 0 100 30], 'parent', obj.guiHandles.modePanel, 'Callback', @(src, event) obj.switchMode());  % Reposition

            % Add e-stop button
            obj.guiHandles.estopButton = uicontrol('Style', 'pushbutton', 'String', 'E-STOP', 'Position', [130, 20, 60, 30], 'Callback', @(src, event) obj.estopButtonCallback(), 'BackgroundColor', 'green');

            %obj.animateTable();
        end

        function estopButtonCallback(obj)
            % Toggle e-stop state
            obj.estop_ = ~obj.estop_;
            
            % Update status dot colour based on e-stop state
            if obj.estop_
                obj.guiHandles.estopButton.BackgroundColor = 'red';
            else
                obj.guiHandles.estopButton.BackgroundColor = 'green';
            end
        end

        function switchMode(obj)
            selectedMode = obj.guiHandles.modePanel.SelectedObject.String;
            
            if isfield(obj.guiHandles, 'upArrow')
                % Delete existing cartesian UI elements
                delete(obj.guiHandles.upArrow);
                delete(obj.guiHandles.downArrow);
                delete(obj.guiHandles.leftArrow);
                delete(obj.guiHandles.rightArrow);
                delete(obj.guiHandles.forwardArrow);
                delete(obj.guiHandles.backwardArrow);
            end

            if isfield(obj.guiHandles, 'sliders')
                % Delete existing joint UI elements
                delete(obj.guiHandles.sliders);
                delete(obj.guiHandles.labels);
            end

            if strcmp(selectedMode, 'Joint control')
                set(obj.guiHandles.goButton, 'Enable', 'on');
                set(obj.guiHandles.liveCheckbox, 'Enable', 'on');
                % create UI elements for Joint angle mode
                obj.updateSliders();
            elseif strcmp(selectedMode, 'Cartesian control')
                set(obj.guiHandles.goButton, 'Enable', 'off');
                set(obj.guiHandles.liveCheckbox, 'Enable', 'off');
                % create UI elements for Global cartesian mode
                obj.createCartesianControls();
            end
        end

        function createCartesianControls(obj)
            % Create arrows and associate them with callback functions
            obj.guiHandles.upArrow = uicontrol('Style', 'pushbutton', 'String', '^', 'Position', [125, 245, 30, 30], 'Callback', @(src, event) obj.moveCartesian('up'));
            obj.guiHandles.downArrow = uicontrol('Style', 'pushbutton', 'String', 'v', 'Position', [125, 215, 30, 30], 'Callback', @(src, event) obj.moveCartesian('down'));

            obj.guiHandles.leftArrow = uicontrol('Style', 'pushbutton', 'String', '<', 'Position', [30, 230, 30, 30], 'Callback', @(src, event) obj.moveCartesian('left'));
            obj.guiHandles.rightArrow = uicontrol('Style', 'pushbutton', 'String', '>', 'Position', [90, 230, 30, 30], 'Callback', @(src, event) obj.moveCartesian('right'));
            obj.guiHandles.forwardArrow = uicontrol('Style', 'pushbutton', 'String', '^', 'Position', [60, 260, 30, 30], 'Callback', @(src, event) obj.moveCartesian('forward'));
            obj.guiHandles.backwardArrow = uicontrol('Style', 'pushbutton', 'String', 'v', 'Position', [60, 200, 30, 30], 'Callback', @(src, event) obj.moveCartesian('backward'));
        end

        function moveCartesian(obj, direction, ~, ~)
            % Disable the buttons
            set(obj.guiHandles.upArrow, 'Enable', 'off');
            set(obj.guiHandles.downArrow, 'Enable', 'off');
            set(obj.guiHandles.leftArrow, 'Enable', 'off');
            set(obj.guiHandles.rightArrow, 'Enable', 'off');
            set(obj.guiHandles.forwardArrow, 'Enable', 'off');
            set(obj.guiHandles.backwardArrow, 'Enable', 'off');

            robotNum_ = get(obj.guiHandles.robotSelect, 'Value');
            
            speed_ = 20;

            switch direction
                case 'up'
                    % move robot up
                    nextQ = obj.RRMCNextQ([0,0,speed_], robotNum_);
                    obj.moveRobot(nextQ, robotNum_, 10);

                case 'down'
                    % move robot down
                    nextQ = obj.RRMCNextQ([0,0,-speed_], robotNum_);
                    obj.moveRobot(nextQ, robotNum_, 10);
                
                case 'left'
                    % move robot left
                    nextQ = obj.RRMCNextQ([0,speed_,0], robotNum_);
                    obj.moveRobot(nextQ, robotNum_, 10);

                case 'right'
                    % move robot right
                    nextQ = obj.RRMCNextQ([0,-speed_,0], robotNum_);
                    obj.moveRobot(nextQ, robotNum_, 10);

                case 'forward'
                    % move robot forward
                    nextQ = obj.RRMCNextQ([speed_,0,0], robotNum_);
                    obj.moveRobot(nextQ, robotNum_, 10);

                case 'backward'
                    % move robot backward
                    nextQ = obj.RRMCNextQ([-speed_,0,0], robotNum_);
                    obj.moveRobot(nextQ, robotNum_, 10);
            end
            
            set(obj.guiHandles.upArrow, 'Enable', 'on');
            set(obj.guiHandles.downArrow, 'Enable', 'on');
            set(obj.guiHandles.leftArrow, 'Enable', 'on');
            set(obj.guiHandles.rightArrow, 'Enable', 'on');
            set(obj.guiHandles.forwardArrow, 'Enable', 'on');
            set(obj.guiHandles.backwardArrow, 'Enable', 'on');
        end

        function animateTable(obj)
            %offsets change the location of the object
            xOffset = 0;
            yOffset = .2;
            zOffset = -1;
            % enter name of ply file to be displayed
            [f,v,data] = plyread('Scenery_complete.ply','tri'); 
            % sets vertex colours in rgb values from ply file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
            %plotting
            plot on;
            trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
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
                result = checkCollision.IsCollision(obj.robot_{robotNum_}.model, q(i,:), obj.faces, obj.vertex, obj.faceNormals); % collision detection part
                if result
                    return;
                end
                
                % Check if e-stop is active
                if obj.estop_
                    obj.currentJointPos_{robotNum_} = obj.robot_{robotNum_}.model.getpos();
                    %obj.updateSliders();
                    status = false;
                    return;
                end

                obj.robot_{robotNum_}.model.animate(q(i,:));
                pause(0.01);
            end
            
            obj.currentJointPos_{robotNum_} = obj.nextJointPos_{robotNum_};
            status = true;
            return;
        end


        function nextQ = RRMCNextQ(obj, eeVel, robotNum_)

            eeVel = [eeVel, 0, 0, 0]';
            qlim = obj.robot_{robotNum_}.model.qlim;
            nJoints = size(qlim, 1);
            q = obj.robot_{robotNum_}.model.getpos();
            
            dt = 0.002;
            J = obj.robot_{robotNum_}.model.jacob0(obj.robot_{robotNum_}.model.getpos());
        
            % damping parameter
            lambda = 0.02;

            % damped least squares
            Jinv_dls = inv((J'*J)+lambda^2*eye(nJoints))*J';

            dq = Jinv_dls*eeVel;
            nextQ = q + dq'*dt;

            % Check if any joint is out of its bounds
            out_bounds = any(nextQ > qlim(:, 2)') || any(nextQ < qlim(:, 1)');
            
            if out_bounds
                nextQ = obj.robot_{robotNum_}.model.getpos();
            end
            
        end

    end
end

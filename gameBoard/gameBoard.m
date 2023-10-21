classdef gameBoard < handle
    properties
        gameSubscriber;  % Instance of the subscriber_gameBoard class
        localGameBoard;
        timerObj;      % Timer object for running plotData
        real_location; % contains the localGameBoard data but converted to metres where the origin is 0,0 at the top left
        next_iteration;
        board_; % physical gameboard object class
        tasks_ = {};
        sideLineTr; % this is where all the checkers that are out go
        checkerHeight_;
    end
    
    methods
        function obj = gameBoard()
            obj.gameSubscriber = subscriber_gameBoard('/board_state', 'std_msgs/Int32MultiArray');
    
            % initialising the sideline
            obj.sideLineTr = transl(-0.1,0,0);
            % assigning the checker height
            obj.checkerHeight_ = 0.005; % 5mm
            % initialising and plotting the physical gameboard 
            Tboard = transl(0,0.2,0);
            %obj.board_ = Board(0.32, 0.05, Tboard);
            %figure(1);
            %obj.board_.plotBoard();
            axis equal;
            
            % Create a timer object
            obj.timerObj = timer('TimerFcn', @(~,~) obj.plotData(), ...
                                 'Period', 0.25, ...             % Run every 0.25 seconds
                                 'ExecutionMode', 'fixedRate');
            obj.next_iteration = 0;
        end


        function M = getLastCellAsMatrix(obj)
            % Check if cell array is empty
            if isempty(obj.gameSubscriber.data)
                error('Input cell array is empty.');
            end
        
            % Get the last cell from the cell array
            lastCell = obj.gameSubscriber.data{end};
        
            % Calculate the size of the lastCell
            size_ = length(lastCell);
        
            if mod(size_, 3) ~= 0
                error('Size of last cell is not a multiple of 3.');
            end
            
            % Initialize the new array
            newArray = zeros(3, size_/3);
            
            % Populate the new array
            for n = 1:3:size_
                colIndex = (n + 2) / 3;
                newArray(1, colIndex) = lastCell(n);
                newArray(2, colIndex) = lastCell(n + 1);
                newArray(3, colIndex) = lastCell(n + 2);
            end

            M = newArray;
        end

        
        % function gameBoardToRealDimensions(obj)
        %     % Extract the 3xN double array from the 1x1 cell array
        %     localData = obj.localGameBoard{1};  
        % 
        %     scale_factor = obj.board_square_size;
        %     offset = scale_factor / 2;
        % 
        %     % Initialize real_location if it's empty, matching the size of localData
        %     if isempty(obj.real_location)
        %         obj.real_location = cell(size(localData));
        %     end
        % 
        %     % Perform element-wise operations
        %     real_location_1 = localData(1, :) .* scale_factor + offset;
        %     real_location_2 = localData(2, :) .* scale_factor + offset;
        %     real_location_3 = localData(3, :);  % Assuming you want to copy this row as-is
        % 
        %     % Combine the rows into a single 3xN array
        %     combined_real_location = [real_location_1; real_location_2; real_location_3];
        % 
        %     % Place this array into a cell if needed
        %     obj.real_location{1} = combined_real_location;
        % end


        function [movedFrom, movedTo, removedFrom] = compareCheckerStates(obj, previous, current)
            % Transpose the matrices to make them easier to work with
            previous = previous';
            current = current';
            
            % Initialising empty arrays for the output
            movedFrom = [];
            movedTo = [];
            removedFrom = [];
            
            % Finding removed checkers
            for i = 1:size(previous, 1)
                prev_checker = previous(i, :);
                if ~any(ismember(current(:, 1:2), prev_checker(1:2), 'rows'))
                    removedFrom = [removedFrom; prev_checker];
                end
            end
            
            % Finding moved checkers
            for i = 1:size(current, 1)
                curr_checker = current(i, :);
                if ~any(ismember(previous(:, 1:2), curr_checker(1:2), 'rows'))
                    movedTo = [movedTo; curr_checker];
                end
            end
            
            % Finding the original position of moved checkers
            for i = 1:size(movedTo, 1)
                moved_checker = movedTo(i, :);
                for j = 1:size(previous, 1)
                    prev_checker = previous(j, :);
                    if moved_checker(3) == prev_checker(3) && ~ismember(prev_checker(1:2), current(:, 1:2), 'rows')
                        movedFrom = [movedFrom; prev_checker];
                    end
                end
            end

            % Filter out the moved checkers from the removedFrom list
            if ~isempty(movedFrom)
                removedFrom = setdiff(removedFrom, movedFrom, 'rows');
            end
        end

        function Tr = convertIndexToTrans(obj, indexPos)
            width_ = obj.board_.squareSize;
            origin_ = obj.board_.base(1:3, 4); % only concerned about translation for offsets
            rotation_ = obj.board_.base(1:3, 1:3);
            
            % Calculate the new x and y positions
            x_ = (indexPos(1) * width_) + (width_ / 2) + origin_(1);
            y_ = (indexPos(2) * width_) + (width_ / 2) + origin_(2);
            
            % Create the translation vector
            translationVector = [x_; y_; origin_(3)];
            
            % Incorporate the rotation
            rotatedVector = rotation_ * translationVector;
            
            % Create the final homogeneous transformation matrix
            Tr = [rotation_, rotatedVector; 0 0 0 1];
        end

        function plotTr(obj, Tr, colour)
            figure(1);
            if colour == 1
                trplot(Tr, 'length', 0.05, 'color', 'r');
            else
                trplot(Tr, 'length', 0.05, 'color', 'b');
            end
        end
        

        function assignTasks(obj, movedFrom, movedTo, removedFrom)
            tasksGrouped = {};

            if (~isempty(movedFrom) && ~isempty(movedTo))
                %startTr = obj.convertIndexToTrans(movedFrom);
                tasksGrouped{1} = movedFrom;
                tasksGrouped{2} = movedTo;
                %obj.plotTr(startTr, movedFrom(3));
                obj.tasks_{end+1} = tasksGrouped;
                
            end

            if (~isempty(removedFrom))
                %removedTr = obj.convertIndexToTrans(removedFrom);
                tasksGrouped{1} = removedFrom;
                %obj.plotTr(removedTr, removedFrom(3));
                % !!! what happens if more than 1 checker is out?? !!!
                tasksGrouped{2} = -1; % move to side
                %obj.sideLineTr = obj.sideLineTr * transl(0,0, obj.checkerHeight_); % offsetting for next checker to be stacked on top
                %obj.plotTr(obj.sideLineTr, removedFrom(3));
                obj.tasks_{end+1} = tasksGrouped;
            end
        end


        function plotData(obj)
            % Check if data is empty
            if isempty(obj.gameSubscriber.data)
                disp('No data...');
                return;
            end

            if ~isempty(obj.localGameBoard)
                newMatrix = getLastCellAsMatrix(obj);
                lastBoard = obj.localGameBoard{end};
                
                if isequal(lastBoard, newMatrix)  % if the state of the board hasn't changed, don't update
                    %disp('skipping'); %debugging
                    return;
                else
                    disp("New data...");
                    [movedFrom, movedTo, removedFrom] = obj.compareCheckerStates(lastBoard, newMatrix);

                    obj.assignTasks(movedFrom, movedTo, removedFrom);
                    obj.tasks_

                end
            end

            obj.localGameBoard{end+1} = getLastCellAsMatrix(obj);

            obj.next_iteration = length(obj.localGameBoard);
            
            % Plot the data
            figure(2); % Reuse the same figure
            cla; % Clear the current figure
            
            % load new gameboard state
            x = obj.localGameBoard{end}(1, :);
            y = obj.localGameBoard{end}(2, :);
            color = obj.localGameBoard{end}(3, :);
        
            % batch plotting red and blue
            idx_red = (color == 1);
            idx_blue = ~idx_red;
            
            scatter(x(idx_red), y(idx_red), 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
            hold on;
            scatter(x(idx_blue), y(idx_blue), 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
            hold off;

            % Reverse the direction of the Y-axis
            set(gca, 'YDir', 'reverse');
            
            % Make the axis square
            axis square;
            xlabel('X-axis');
            ylabel('Y-axis');
            title('Game Map Coordinates');

        end
        
        function run(obj)
            obj.gameSubscriber.run(); % Start running the gameBoard
            
            % Start the timer
            start(obj.timerObj);
        end
        
        function stop(obj)
            obj.gameSubscriber.stop(); % Stop the gameBoard
            
            % Stop the timer
            stop(obj.timerObj);
            rosshutdown;
        end
    end
end

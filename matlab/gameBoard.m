classdef gameBoard < handle
    properties
        gameBoardObj;  % Instance of the subscriber_gameBoard class
        localGameBoard;
        timerObj;      % Timer object for running plotData
        real_location; % contains the localGameBoard data but converted to metres where the origin is 0,0 at the top left
        board_square_size; % the width/height of an individial square
        next_iteration
    end
    
    methods
        function obj = gameBoard()
            obj.gameBoardObj = subscriber_gameBoard('/test_topic', 'std_msgs/Int32MultiArray');
            
            % Create a timer object
            obj.timerObj = timer('TimerFcn', @(~,~) obj.plotData(), ...
                                 'Period', 0.25, ...             % Run every 1 second
                                 'ExecutionMode', 'fixedRate');
            obj.next_iteration = 0;
        end


        function M = getLastCellAsMatrix(obj)
            % Check if cell array is empty
            if isempty(obj.gameBoardObj.data)
                error('Input cell array is empty.');
            end
        
            % Get the last cell from the cell array
            lastCell = obj.gameBoardObj.data{end};
        
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

        
        function gameBoardToRealDimensions(obj)
            % Extract the 3xN double array from the 1x1 cell array
            localData = obj.localGameBoard{1};  
            
            scale_factor = obj.board_square_size;
            offset = scale_factor / 2;
            
            % Initialize real_location if it's empty, matching the size of localData
            if isempty(obj.real_location)
                obj.real_location = cell(size(localData));
            end
            
            % Perform element-wise operations
            real_location_1 = localData(1, :) .* scale_factor + offset;
            real_location_2 = localData(2, :) .* scale_factor + offset;
            real_location_3 = localData(3, :);  % Assuming you want to copy this row as-is
            
            % Combine the rows into a single 3xN array
            combined_real_location = [real_location_1; real_location_2; real_location_3];
            
            % Place this array into a cell if needed
            obj.real_location{1} = combined_real_location;
        end


        function plotData(obj)
            % Check if data is empty
            if isempty(obj.gameBoardObj.data)
                disp('No data');
                return;
            end

            if ~isempty(obj.localGameBoard)
                newMatrix = getLastCellAsMatrix(obj);
                lastBoard = obj.localGameBoard{end};
                
                if isequal(size(lastBoard), size(newMatrix))  % Check if sizes are the same
                    if lastBoard == newMatrix  % Element-wise comparison
                        % disp('skipping'); debugging
                        return;
                    end
                end
            end

            obj.localGameBoard{end+1} = getLastCellAsMatrix(obj);

           % gameBoardToRealDimensions(obj); % !!! last error: Arrays have incompatible sizes for this operation.

            obj.next_iteration = length(obj.localGameBoard);
            
            % Plot the data
            figure(1); % Reuse the same figure
            cla; % Clear the current axes
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
            obj.gameBoardObj.run(); % Start running the gameBoard
            
            % Start the timer
            start(obj.timerObj);
        end
        
        function stop(obj)
            obj.gameBoardObj.stop(); % Stop the gameBoard
            
            % Stop the timer
            stop(obj.timerObj);
            rosshutdown;
        end
    end
end

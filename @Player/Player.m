

classdef Player < handle
    % Executive management of motion-planning operations.
    properties
        mp
        isCobot = 0;
    end

    methods
        function self = Player(serialLink, q0, Tboard, squareSize, Tbin, ...
                IKmethod,varargin)
            % parse motion option mode – is cobotQready provided?
            for i = 1:2:length(varargin)
                argin = varargin{i};
                option = argin;
                value = varargin{i + 1};
                % check and set options:
                if strcmp(option, 'cobotQready')
                    self.mp = MotionPlanner(serialLink, q0, Tboard, squareSize, ...
                    Tbin, IKmethod,'cobotQready',value);
                    self.isCobot = 1;
                    return;
                else
                    error('Invalid option: %s', option);
                end
            end
            self.mp = MotionPlanner(serialLink, q0, Tboard, squareSize, ...
                    Tbin, IKmethod);
        end

        % this method is repsponsible for planning the player's piece to move
        function [traj,toggleGripAfterIndex] = processTaskTrajectory(self, task)
            % moving Player's piece:
            movePositions = self.movePositions(task);
            traj = zeros(1,self.mp.robot.n);
            if self.isCobot == 1
                traj = self.mp.interpolationTrajectoryToReady();
                traj = [traj; self.mp.trajectoryMoveToSquare( ...
                    movePositions(1,:))];
            else
                traj = self.mp.trajectoryMoveToSquare(movePositions(1,:));
            end

            % grip my piece:
            toggleGripAfterIndex = size(traj,1);

            if size(movePositions,1) > 2
                for i=2:size(movePositions,1)-1
                    traj = [traj; self.mp.trajectorySquareToSquare( ...
                        movePositions(i-1,:), movePositions(i,:))];
                end
            end
            traji = self.mp.trajectorySquareToSquare( ...
                movePositions(end-1,:), movePositions(end,:) );
            traj = [traj; traji];

            % release my piece:
            toggleGripAfterIndex = [toggleGripAfterIndex, ...
                size(traj,1)];

            traj = [traj; self.mp.trajectorySquareToAbove(movePositions ...
                (end,:))];
            
            % moving prisoners to bin
            if ~isempty(task{3})
                prisonerPositions = task{3};
                for i=1:size(prisonerPositions,1)
                    traj = [traj; self.mp.trajectoryMoveToSquare( ...
                        prisonerPositions(i,:))];
                    % grip my piece:
                    toggleGripAfterIndex = [toggleGripAfterIndex, ...
                        size(traj,1)];
                    traj = [traj; self.mp.trajectorySquareToBin( ...
                        prisonerPositions(i,:))];
                    % release my piece:
                    toggleGripAfterIndex = [toggleGripAfterIndex, ...
                        size(traj,1)];
                    traj = [traj; self.mp.trajectoryMoveToAboveBin()];
                end
            end
            
            % return home:
            if self.isCobot == 1
                traj = [traj; self.mp.interpolationTrajectoryToReady()];
                traj = [traj; self.mp.trajectoryToHome()];
            else
                traj = [traj; self.mp.trajectoryToHome()];
            end
        end

        function hasWon = hasWon(self)
            hasWon = (self.mp.numPrisoners == 8);
        end
    end

    methods (Static)
        function positions = movePositions(task)
            numPrisoners = size(task{3},1);
            if numPrisoners == 0
                positions = zeros(2, 2);
            else
                positions = zeros( 1+numPrisoners, 2);
            end
            positions(1,:) = task{2}(1,:);
            if numPrisoners > 1
                for i=2:numPrisoners
                    positions(i,:) = Player.interstitialPosition( task{3}(i-1,:), ...
                        task{3}(i,:) );
                end
            end
            positions(end,:) = task{2}(2,:);
        end

        function pos = interstitialPosition(pos1, pos2)
            pos = pos1+(pos2-pos1)./2;
        end
    end
end
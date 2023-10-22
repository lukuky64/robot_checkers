classdef Player < handle
    % Executive management of motion-planning operations.
    properties
        mp
    end

    methods
        function self = Player(serialLink, q0, Tboard, squareSize, Tbin, ...
                IKmethod, Tbase)
            if nargin > 6
                self.mp = MotionPlanner(serialLink, q0, Tboard, squareSize, ...
                    Tbin, IKmethod, Tbase);
            else
                self.mp = MotionPlanner(serialLink, q0, Tboard, squareSize, ...
                    Tbin, IKmethod);
            end
        end

        function traj = processTaskTrajectory(self, task)
            % moving Player's piece:
            movePositions = self.movePositions(task);
            traj = self.mp.trajectoryMoveToSquare(movePositions(1,:));
            if size(movePositions,1) > 2
                for i=2:size(movePositions,1)-1
                    traj = [traj; self.mp.trajectorySquareToSquare( ...
                        movePositions(i-1,:), movePositions(i,:))];
                end
            end
            traj = [traj; self.mp.trajectorySquareToSquare( ...
                movePositions(end-1,:), movePositions(end,:) )];
            traj = [traj; self.mp.trajectorySquareToAbove(movePositions ...
                (end,:))];
            
            % moving prisoners to bin
            if size(movePositions,1) > 2
                prisonerPositions = task{3};
                for i=1:size(prisonerPositions,1)
                    traj = [traj; self.mp.trajectoryMoveToSquare( ...
                        prisonerPositions(i,:))];
                    traj = [traj; self.mp.trajectorySquareToBin( ...
                        prisonerPositions(i,:))];
                    traj = [traj; self.mp.trajectoryMoveToAboveBin()];
                end
            end
            
            % return home:
            traj = [traj; self.mp.trajectoryToHome()];
        end

        function hasWon = hasWon(self)
            hasWon = self.mp.numPrisoners == 8;
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
            for i=2:numPrisoners
                positions(i,:) = Player.interstitialPosition( task{3}(i-1,:), ...
                    task{3}(i,:) );
            end
            positions(end,:) = task{2}(2,:);
        end

        function pos = interstitialPosition(pos1, pos2)
            pos = pos1+(pos2-pos1)./2;
        end
    end
end
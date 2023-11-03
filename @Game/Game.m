classdef Game < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        dobotQ0 = [pi/2 0 pi/4 3*pi/4 -pi/2]; % dobot's qHome
        cobotQ0 = [pi/2 pi/8 3*pi/4 -3*pi/8 -pi/2 0]
        cobotQready = deg2rad([0 25 90 -45 -90 0])
        % the following are interdependent:
        squareSize = .025 % checkers square size [m] (if change, change Tboard)
        boardHeight = .05; % checkers board height [m] (if change, change Tboard)
        Tboard = transl(-(.025*8)/2,.17,.05); % checkers board transform (ensure no rotation wrt. world)
        TbinDobot = transl(-(.025*8)/2,.17+(.025*8)/2,0);
        TbinCobot = transl((.025*8)/2,.17+(.025*8)/2,0); %---------
    end
    
    properties
        animator
        playerRed
        playerBlue
        gameBoard
    end
    
    methods
        function self = Game()
            self.animator = Animator(self.dobotQ0,self.cobotQ0,self.squareSize, ...
                self.boardHeight,self.Tboard);
            self.playerRed = Player(self.animator.dobot,self.dobotQ0, ...
                self.Tboard,self.squareSize,self.TbinDobot,'dobot');
            self.playerBlue = Player(self.animator.cobot,self.cobotQ0, ...
                self.Tboard,self.squareSize, self.TbinCobot,'cobot', ...
                'cobotQready',self.cobotQready);
            self.gameBoard = GameBoard();
            self.startGame();
        end
        
        % could have blackout interrupt delete this version of game and
        % then start a new recovered one
        
        function startGame(self)
            self.gameBoard.run();
            gameWinner = 0;
            wasStopped = 0;
            while gameWinner == 0
                if ~isempty(self.gameBoard.tasks_)
                    task = self.gameBoard.tasks_{1};
                    if (task{1} == 0) &&  ~self.animator.blackout.activated % blue/cobot turn-------------------
                        [traj,toggleGripAfterIndex] = self.playerBlue.processTaskTrajectory(task);
                        if ~wasStopped
                            [trajResidual, wasStopped] = self.animator.animatePlayerMove(traj,toggleGripAfterIndex,'robot','cobot');
                        else
                            [trajResidual, wasStopped] = self.animator.animatePlayerMove(trajResidual,toggleGripAfterIndex,'robot','cobot');
                        end
                        if ~(size(traj,1) == size(trajResidual,1)) % if: estop pressed during animatePlayerMove()
                            traj = trajResidual; % traj = rows of traj not yet animated
                        else
                            self.gameBoard.removeTask(1);
                        end
                        
                        if self.playerBlue.hasWon()
                            gameWinner = 'blue';
                        end
                    elseif (task{1} == 1) && ~self.animator.blackout.activated % red/dobot turn -------------------------
                        [traj,toggleGripAfterIndex] = self.playerRed.processTaskTrajectory(task);
                        if ~wasStopped
                            [trajResidual, wasStopped] = self.animator.animatePlayerMove(traj,toggleGripAfterIndex,'robot','dobot');
                        else
                            [trajResidual, wasStopped] = self.animator.animatePlayerMove(trajResidual,toggleGripAfterIndex,'robot','dobot');
                        end
                        if ~(size(traj,1) == size(trajResidual,1)) % if: estop pressed during animatePlayerMove()
                            traj = trajResidual; % traj = rows of traj not yet animated
                        else
                            self.gameBoard.removeTask(1);
                        end
                        if self.playerRed.hasWon()
                            gameWinner = 'red';
                        end
                    end
                end
                pause(0.1)
            end
            if gameWinner == 'red'
                text(0,0,0, ...
                    "Red player won.",'FontSize', ...
                    50,'Color','b');
            elseif gameWinner == 'blue'
                text(0,0,0, ...
                    "Blue player won.",'FontSize', ...
                    50,'Color','r');
            end
        end
    end
end





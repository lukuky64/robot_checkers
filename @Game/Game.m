classdef Game < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties (Constant)
        dobotQ0 = [pi/2 0 pi/4 3*pi/4 -pi/2]; % dobot's qHome
        cobotQ0 = [pi/2 0 3*pi/4 -pi/4 -pi/2 0]; %--------
        squareSize = .035; % checkers square size [m] (if change, change Tboard)
        boardHeight = .05; % checkers board height [m] (if change, change Tboard)
        Tboard = transl(-(.035*8)/2,.04,.05); % checkers board transform (ensure no rotation wrt. world)
        TbinDobot = transl(-.2,.2,0);
        TbinCobot = transl(0,0,0); %---------
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
                self.Tboard,self.squareSize, self.TbinCobot,'cobot');
            %self.gameBoard = GameBoard();
            %self.startGame();
        end

        function startGame(self)
            self.gameBoard.run();
            gameWinner = 0;
            while gameWinner == 0
                if ~isempty(self.gameBoard.tasks_)
                    task = self.gameBoard.tasks_{1};
                    if task{1} == 0 % blue/cobot turn
                        traj = self.playerBlue.processTaskTrajectory(task);
                        self.animator.animatePlayerMove(traj,'robot','cobot');
                        self.gameBoard.removeTask(1);
                        if self.playerBlue.hasWon()
                            gameWinner = 'blue';
                        end
                    elseif task{1} == 1 % red/dobot turn
                        traj = self.playerRed.processTaskTrajectory(task);
                        self.animator.animatePlayerMove(traj,'robot','dobot');
                        self.gameBoard.removeTask(1);
                        if self.playerRed.hasWon()
                            gameWinner = 'red';
                        end
                    end
                end
            pause(0.1)
            end
            display("The "+gameWinner+" team hasn won the game.")
        end
    end
end





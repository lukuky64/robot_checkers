classdef Game < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties (Constant)
        dobotQ0 = [pi/2 0 pi/4 3*pi/4 -pi/2]; % dobot's qHome
        cobotQ0 = zeros(1,6); %--------
        squareSize = .035; % checkers square size [m] (if change, change Tboard)
        boardHeight = .05; % checkers board height [m] (if change, change Tboard)
        Tboard = transl(-(.035*8)/2,.04,.05); % checkers board transform (ensure no rotation wrt. world)
        TbinDobot = transl(-.2,.2,0);
        TbinCobot = transl(0,0,0) %---------
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
                self.Tboard,self.squareSize,self.TbinDobot);
            self.playerBlue = Player(self.animator.cobot,self.cobotQ0, ...
                self.Tboard,self.squareSize, self.TbinCobot);
            self.gameBoard = GameBoard();
            self.startGame();
        end

        function startGame(self)
            self.gameBoard.run();
            
        end
    end
end





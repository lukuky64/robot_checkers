classdef Animator < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dobotRobotBaseClass
        cobotRobotBaseClass
        dobot
        cobot
        board
    end

    methods
        function self = Animator(dobotQ0,cobotQ0,squareSize,boardHeight, ...
                Tboard)
            self.setupEnvironment(dobotQ0,cobotQ0,squareSize,boardHeight, ...
                Tboard);
        end

        function setupEnvironment(self,dobotQ0,cobotQ0,squareSize, ...
                boardHeight,Tboard)
            self.dobotRobotBaseClass = DobotMagician(rpy2tr(0,0,pi/2));
            self.dobot = self.dobotRobotBaseClass.model;
            hold on
            cobotBaseBoardGap = .05;
            TcobotBase = transl(0,squareSize*8+Tboard(2,4)+ ...
                cobotBaseBoardGap,0)*rpy2tr(0,0,pi/2);
            self.cobotRobotBaseClass = MyCobot320(TcobotBase);
            self.cobot = self.cobotRobotBaseClass.model;
            
            self.board = Board(squareSize*8,boardHeight,Tboard);
            self.board.plotBoard();
            self.dobot.animate(dobotQ0);
            self.cobot.animate(cobotQ0);
        end

        function animatePlayerMove(self,traj,varargin)
            % parse option of dobot or cobot:
            robotSelection;
            for i = 1:2:length(varargin)
                argin = varargin;
                option = argin{i};
                value = argin{i + 1};
                % check and set options
                if strcmp(option, 'robot')
                    robotSelection = value;
                else
                    error('Invalid option: %s', option);
                end
            end
            % animate robot:
            if robotSelection == 'cobot'
                for i=1:size(traj,1)
                    self.cobot.animate(traj(i,:));
                    pause(25^-1);
                end
            elseif robotSelection == 'dobot'
                for i=1:size(traj,1)
                    self.dobot.animate(traj(i,:));
                    pause(25^-1);
                end
            end
        end
    end
end
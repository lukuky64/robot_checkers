classdef Animator < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dobotRobotBaseClass
        cobotRobotBaseClass
        dobot
        cobot
        board
        checkerPieces = zeros(1,24)
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
            cobotBaseBoardGap = .14;
            TcobotBase = transl(0,squareSize*8+Tboard(2,4)+ ...
                cobotBaseBoardGap,0)*rpy2tr(0,0,pi/2);
            self.cobotRobotBaseClass = MyCobot320(TcobotBase);
            self.cobot = self.cobotRobotBaseClass.model;
            
            self.board = Board(squareSize*8,boardHeight,Tboard);
            self.board.plotBoard();
            
            for i = 1:24
                if i < 12
                    if i < 5
                        self.checkerPieces(i) = CheckerPiece('blue', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(0,7*squareSize,0)*transl(squaresize,0,0)* ...
                            transl(2*(i-1)*squareSize,0,0));
                            continue
                    elseif i < 9
                        self.checkerPieces(i) = CheckerPiece('blue', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(0,6*squareSize,0)*transl(2*(i-5)*squareSize,0,0));
                            continue
                    else 
                        self.checkerPieces(i) = CheckerPiece('blue', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(squareSize,0,0)*transl(0,5*squareSize,0)* ...
                            transl(2*(i-9)*squareSize,0,0));
                            continue
                    end
                else
                    if i < 17
                        self.checkerPieces(i) = CheckerPiece('red', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(2*(i-13)*squareSize,0,0));
                            continue
                    elseif i < 21
                        self.checkerPieces(i) = CheckerPiece('red', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(squaresize,squareSize,0)*transl(2*(i-17)*squareSize,0,0));
                            continue
                    else 
                        self.checkerPieces(i) = CheckerPiece('red', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(0,2*squareSize,0)*transl(2*(i-21)*squareSize,0,0));
                            continue
                    end
                end
            end

            self.dobot.animate(dobotQ0);
            self.cobot.animate(cobotQ0);
        end

        function [traj,wasStopped] = animatePlayerMove(self,traj,varargin)
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
                    % check if estopped pressed
                    if Blackout.activated
                        % return residual trajectory:
                        traj = traj(i:end,:);
                        wasStopped = 1;
                        return
                    end
                    self.cobot.animate(traj(i,:));
                    pause(25^-1);
                end
            elseif robotSelection == 'dobot'
                for i=1:size(traj,1)
                    % check if estopped pressed
                    if Blackout.activated
                        % return residual trajectory:
                        traj = traj(i:end,:);
                        wasStopped = 1;
                        return
                    end
                    self.dobot.animate(traj(i,:));
                    pause(25^-1);
                end
            end
            wasStopped = 0;
        end
    end
end
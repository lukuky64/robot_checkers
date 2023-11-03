classdef Animator < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dobotRobotBaseClass
        cobotRobotBaseClass
        dobot
        cobot
        board
        checkerPieces = {}
        guiHandles = struct()
        blackout = Blackout()
        suctionTextHandle
        suctionIsOn = 0
        looseChecker
    end

    methods
        function self = Animator(dobotQ0,cobotQ0,squareSize,boardHeight, ...
                Tboard)
            self.setupEnvironment(dobotQ0,cobotQ0,squareSize,boardHeight, ...
                Tboard);
        end

        function setupEnvironment(self,dobotQ0,cobotQ0,squareSize, ...
                boardHeight,Tboard)
            self.animateTable();
            hold on
            self.animateFloor();
            self.dobotRobotBaseClass = DobotMagician_succ(rpy2tr(0,0,pi/2));
            self.dobot = self.dobotRobotBaseClass.model;            
            cobotBaseBoardGap = .14;
            TcobotBase = transl(0,squareSize*8+Tboard(2,4)+ ...
                cobotBaseBoardGap,0)*rpy2tr(0,0,pi/2);
            self.cobotRobotBaseClass = MyCobot320(TcobotBase);
            self.cobot = self.cobotRobotBaseClass.model;
            
            self.board = Board(squareSize*8,boardHeight,Tboard);
            self.board.plotBoard();

            for i = 1:24
                if i < 13
                    if i < 5
                        self.checkerPieces{i} = CheckerPiece('blue', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(0,7*squareSize,0)*transl(squareSize,0,0)* ...
                            transl(2*(i-1)*squareSize,0,0));
                            continue
                    elseif i < 9
                        self.checkerPieces{i} = CheckerPiece('blue', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(0,6*squareSize,0)*transl(2*(i-5)*squareSize,0,0));
                            continue
                    else 
                        self.checkerPieces{i} = CheckerPiece('blue', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(squareSize,0,0)*transl(0,5*squareSize,0)* ...
                            transl(2*(i-9)*squareSize,0,0));
                            continue
                    end
                else
                    if i < 17
                        self.checkerPieces{i} = CheckerPiece('red', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(2*(i-13)*squareSize,0,0));
                            continue
                    elseif i < 21
                        self.checkerPieces{i} = CheckerPiece('red', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(squareSize,squareSize,0)*transl(2*(i-17)*squareSize,0,0));
                            continue
                    else 
                        self.checkerPieces{i} = CheckerPiece('red', ...
                            Tboard*transl(squareSize/2,squareSize/2,0)* ...
                            transl(0,2*squareSize,0)*transl(2*(i-21)*squareSize,0,0));
                            continue
                    end
                end
            end

            self.dobot.animate(dobotQ0);
            self.cobot.animate(cobotQ0);
            axis([-1 1 -.8 1.2 -1 .9])
        end

        function [traj,wasStopped] = animatePlayerMove(self,traj,toggleGripAfterIndex,varargin)
            % parse option of dobot or cobot:
            % robotSelection;
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
                first = 1;
                for i=1:size(traj,1)
                    % check if estopped pressed:
                    if self.blackout.activated()

                        % return residual trajectory:
                        traj = traj(i:end,:);
                        wasStopped = 1;
                        return
                    end
                    self.cobot.animate(traj(i,:));

                    % grip/release end-effector:
                    if any(toggleGripAfterIndex == i)
                        % TOGGLE EE ANIMATION:
                        self.suctionIsOn = xor(self.suctionIsOn,1);
                        if self.suctionIsOn
                            self.suctionTextHandle = text(-.5,0,.4, ...
                                "Cobot is sucking.",'FontSize', ...
                                12,'Color','g');
                        else
                            delete(self.suctionTextHandle)
                        end
                    end

                    % animate checker on end effector:
                    if self.suctionIsOn && first == 1
                        T = self.cobot.fkine(traj(i,:));
                        self.looseChecker('blue',T);
                        first = 0;
                    elseif self.suctionIsOn
                        T = self.cobot.fkine(traj(i,:));
                        self.looseChecker.moveMe(T);
                    else
                        self.looseChecker.deleteMe();
                    end

                    pause(25^-1);
                end
            elseif robotSelection == 'dobot'
                first = 1;
                for i=1:size(traj,1)
                    % check if estopped pressed:
                    if self.blackout.activated()
                        % return residual trajectory:
                        traj = traj(i:end,:);
                        wasStopped = 1;
                        return
                    end
                    self.dobot.animate(traj(i,:));
                    
                    % grip/release end-effector:
                    if any(toggleGripAfterIndex == i)
                        % TOGGLE EE ANIMATION:
                        self.suctionIsOn = xor(self.suctionIsOn,1);
                        if self.suctionIsOn
                            self.suctionTextHandle = text(-.5,0,.4, ...
                                "Dobot is sucking.",'FontSize', ...
                                12,'Color','g');
                        else
                            delete(self.suctionTextHandle)
                        end
                    end

                    % animate checker on end effector:
                    if self.suctionIsOn && first == 1
                        T = self.cobot.fkine(traj(i,:));
                        self.looseChecker('red',T);
                        first = 0;
                    elseif self.suctionIsOn
                        T = self.cobot.fkine(traj(i,:));
                        self.looseChecker.moveMe(T);
                    else
                        self.looseChecker.deleteMe();
                    end

                    pause(15^-1);
                end
            end
            wasStopped = 0;
        end
        
        function animateTable(self)
            %offsets change the location of the object
            xOffset = 0;
            yOffset = .2;
            zOffset = -1;
            % enter name of ply file to be displayed
            [f,v,data] = plyread('Scenery_complete.PLY','tri'); 
            % sets vertex colours in rgb values from ply file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
            %plotting
            trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
        end

        function animateFloor(self)
            % Define the grid for the plane
            [X, Y] = meshgrid(-2:0.1:2, -1.8:0.1:2.2);
            
            % Define the function for the plane (e.g., a simple z = f(x, y) equation)
            height = -1;
            Z = ones(size(X,1),size(X,2)).*height;
            
            % Create the surface plot
            surf(X, Y, Z,'EdgeColor','none');
        end
    end
end
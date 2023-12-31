%{ 
This class is responsible for a robot's motion planning and the maintenance 
of state information, such as joint state q. generates trajectories for a general, static SerialLink object
passed in through constructor-argument.
%}

classdef MotionPlanner < handle
    properties (Constant)
        endEffectorStepSize = 8 *1e-3; % m
        endEffectorStepSizePrecise = 3 *1e-3; % m
        checkerPieceHeight = 4 *1e-3; % m
    end

    properties
        robot % SerialLink model
        qHome
        q % ground truth for robot's joint state
        Tboard % transform to bottom left of board top (ie. playing surface)
        % x-y plane coincident with surface, z points up and y direction is
        % from white-piece team towards black-piece team
        squareSize % board square side length
        numPrisoners % number of slain pieces of other player's
        Tbin 
        IKmethod
        requiresReadyWaypoint = 0
        qReady 
    end
    

    methods
        % constructor receives information about the robot model and
        % checker board
        function obj = MotionPlanner(serialLink, qHome, Tboard, squareSize, Tbin, IKmethod, varargin)
            % parse motion option mode – is qReady provided?
            for i = 1:2:length(varargin)
                argin = varargin{i};
                option = argin;
                value = varargin{i + 1};
                % check and set options:
                if strcmp(option, 'cobotQready')
                    obj.qReady = value;
                    obj.requiresReadyWaypoint = 1;
                else
                    error('Invalid option: %s', option);
                end
            end
            obj.robot = serialLink;
            obj.qHome = qHome;
            obj.q = obj.qHome;
            obj.Tboard = Tboard;
            obj.squareSize = squareSize;
            obj.Tbin = Tbin*rpy2tr(0,0,-pi/2);
            obj.IKmethod = IKmethod;
            obj.numPrisoners = 0;
        end

        % -----------------------------------------------------------------
        % Note: the calling of the following 6 functions, with gripper close
        % and open commands being called between them, enable a full checkers
        % move. To make compound checkers moves, function 
        % trajectorySquare2Square() should be called multiple times.

        % ---------- 1 ----------
        % this function returns trajectory that preceeds an EE grip-pickup: 
        % it moves EE to above square and then down to square boardPos.
        function traj = trajectoryMoveToSquare(self,boardPos)
            % transform to board position (half the height of a checkers'
            % piece above particular square center):
            Tpos = self.Tboard*transl([(boardPos-0.5).*self.squareSize self.checkerPieceHeight/2])*rpy2tr(0, 0, -pi/2);
            % transform to point above target board position:
            Tabove = transl(0, 0, 5*self.checkerPieceHeight)*Tpos;

            % generate the component trajectories between interstitial 
            % waypoints of composite (full) trajectory:
            trajToAbove = self.cartesianTrajectoryTo(Tabove);
            trajAboveToPos = self.cartesianTrajectoryTo(Tpos,'precise',1);
            
            % compile and return composite (full) trajectory:
            size1 = size(trajToAbove,1);
            size2 = size(trajAboveToPos,1);
            traj = zeros(size1+size2, self.robot.n);
            traj(1:size1,:) = trajToAbove;
            traj(size1+1:size(traj,1),:) = trajAboveToPos;
        end
        
        % ---------- 2 ----------
        % this function returns trajectory that moves EE from square 
        % boardPos0 directly to boardPosP. Note: it assumes that the EE is 
        % already at boardPos0 and it does not return home after arriving at
        % boardPosP. 
        function traj = trajectorySquareToSquare(self,boardPos0,boardPosP)
            % transforms to board positions (half the height of a checkers'
            % piece above particular square center):
            Tpos0 = self.Tboard*transl([(boardPos0-0.5).*self.squareSize self.checkerPieceHeight/2])*rpy2tr(0, 0, -pi/2);
            TposP = self.Tboard*transl([(boardPosP-0.5).*self.squareSize self.checkerPieceHeight/2])*rpy2tr(0, 0, -pi/2);
            % transforms to points above target board positions:
            Tabove0 = transl(0, 0, 5*self.checkerPieceHeight)*Tpos0;
            TaboveP = transl(0, 0, 5*self.checkerPieceHeight)*TposP;

            % generate the component trajectories between interstitial 
            % waypoints of composite (full) trajectory:
            trajPos0ToAbove0 = self.cartesianTrajectoryTo(Tabove0,'precise',1);
            trajAbove0ToAboveP = self.cartesianTrajectoryTo(TaboveP);
            trajAbovePToPosP = self.cartesianTrajectoryTo(TposP,'precise',1);            
            % compile and return composite (full) trajectory:
            size3 = size(trajPos0ToAbove0,1);
            size4 = size(trajAbove0ToAboveP,1);
            size5 = size(trajAbovePToPosP,1);
            traj = zeros(size3+size4+size5, self.robot.n);
            traj(1:size3,:) = trajPos0ToAbove0;
            traj(size3+1:size3+size4,:) = trajAbove0ToAboveP;
            traj(size3+size4+1:size(traj,1),:) = trajAbovePToPosP;
        end
        
        % ---------- 3 ----------
        % this function returns a trajectory that moves EE from square to
        % point above square: it should be called after a player has moved
        % their piece, before moving captured opponent's pieces to bin.
        function traj = trajectorySquareToAbove(self,boardPos)
            % transform to square:
            Tpos = self.Tboard*transl([(boardPos-0.5).*self.squareSize self.checkerPieceHeight/2])*rpy2tr(0, 0, -pi/2);
            % transform to point above square:
            Tabove = transl(0, 0, 5*self.checkerPieceHeight)*Tpos;
            % trajectory:
            traj = self.cartesianTrajectoryTo(Tabove,'precise',1);
        end

        % ---------- 4 ----------
        % this function returns trajectory that proceeds an EE grip-dropoff:
        % it moves EE from square boardPos to above square boardPos, and 
        % then returns robot to home pose using joint interpolation.
        function traj = trajectorySquareToBin(self,boardPos)
            % transforms to board positions (half the height of a checkers'
            % piece above particular square center):
            Tpos = self.Tboard*transl([(boardPos-0.5).*self.squareSize self.checkerPieceHeight/2])*rpy2tr(0, 0, -pi/2);
            % transforms to points above target board positions:
            TabovePos = transl(0, 0, 5*self.checkerPieceHeight)*Tpos;
            TeeAboveBin = transl(0, 0, TabovePos(3,4))*self.Tbin;
            TeeBin = transl(0, 0, self.numPrisoners*self.checkerPieceHeight)*self.Tbin;

            % generate the component trajectories between interstitial 
            % waypoints of composite (full) trajectory:
            trajPosToAbovePos = self.cartesianTrajectoryTo(TabovePos,'precise',1);
            trajAbovePosToAboveBin = self.cartesianTrajectoryTo(TeeAboveBin);
            trajAboveBinToBin = self.cartesianTrajectoryTo(TeeBin,'precise',1);
            % compile and return composite (full) trajectory:
            size6 = size(trajPosToAbovePos,1);
            size7 = size(trajAbovePosToAboveBin,1);
            size8 = size(trajAboveBinToBin,1);
            traj = zeros(size6+size7+size8, self.robot.n);
            traj(1:size6,:) = trajPosToAbovePos;
            traj(1+size6:size6+size7,:) = trajAbovePosToAboveBin;
            traj(1+size6+size7:size(traj,1),:) = trajAboveBinToBin;

            % add 1 to numPrisoners as we've just placed an enemy piece in
            % the bin:
            self.numPrisoners = self.numPrisoners+1;
        end

        % ---------- 5 ----------
        % this function returns trajectory that proceeds an EE grip-dropoff
        % at the bin: it moves EE to above the bin and should be called
        % after piece is placed in bin to raise EE.
        function traj = trajectoryMoveToAboveBin(self)
            % transform to above bin:
            TaboveBin = transl(0, 0, self.Tboard(3,4)+5*self.checkerPieceHeight)*self.Tbin;
            % trajectory:
            traj = self.cartesianTrajectoryTo(TaboveBin);
        end
        
        % ---------- 6 ----------
        % this function returns trajectory that brings manipulator to its
        % home pose: it should be the last trajectory implemented for the
        % corresponding robot's turn.
        function traj = trajectoryToHome(self)
            traj = self.interpolationTrajectoryTo(self.qHome);
        end
        
        function traj = interpolationTrajectoryToReady(self)
            traj = self.interpolationTrajectoryTo(self.qReady);
        end

        function traj = cartesianTrajectoryToReady(self)
            traj = self.cartesianTrajectoryTo(self.robot.fkine(self.qReady).T);
        end

        % function that returns cartesian trajectory to arguement 
        % transform (double) and updates q
        function traj = cartesianTrajectoryTo(self,Tp,varargin)
            % parse motion option mode – is precise mode requested?
            preciseModeOn = 0;
            for i = 1:2:length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                % check and set options:
                if strcmp(option, 'precise')
                    preciseModeOn = value;
                else
                    error('Invalid option: %s', option);
                end
            end
            
            % set transforms and points of trajectory extrema:
            T0 = self.robot.fkine(self.q).T;
            P0 = transl(T0);
            if self.IKmethod == 'cobot'
                % cobot TCP with respect to end-effector transform:
                eTc = transl(0,.095,0)*rpy2tr(-pi/2,0,0);
                % Tp conditioned for z-down and TCP offset:
                Tpp = Tp*rpy2tr(pi,0,0)*inv(eTc)*rpy2tr(0,-pi/2,0);
                Pp = transl(Tpp);
            else
                eTc = transl(0,0,-.02);
                Tpp = Tp*inv(eTc);
                Pp = transl(Tp);
            end

            % compute fractional increment by which end-effector will 
            % track the trajectory:
            trajDistance = sqrt(sum((Pp-P0).^2));
            if preciseModeOn == 1
                stepDistance = self.endEffectorStepSizePrecise; % mm x 1e-3 m/mm
            else
                stepDistance = self.endEffectorStepSize; % mm x 1e-3 m/mm
            end
            steps = double(round(trajDistance/stepDistance));
            % check if relevant stepIncrement appropriate for trajDistance:
            if steps < 1 || isempty(steps)
                steps = 5;
            end
            fractDists = jtraj(0,1,steps);
            
            % compute attendant matrix of joint states:
            trajTransforms = ctraj(T0, Tpp, fractDists);
            qMatrix = zeros(steps,self.robot.n);
            qMatrix(1,:) = self.q;
            if self.IKmethod == 'dobot'
                for i=2:steps
                    qMatrix(i,:) = self.robot.ikine(trajTransforms(:,:,i), ...
                        'q0',qMatrix(i-1,:),'mask',[1 1 1 0 1 1],'forceSoln');
                end
            elseif self.IKmethod == 'cobot'
                for i=2:steps
                    qMatrix(i,:) = self.robot.ikcon(trajTransforms(:,:,i),qMatrix(i-1,:));
                end
            else
                display("MotionPlanner property IKmethod must be set by constructor-arguement to either 'dobot' or 'cobot'.");
            end

            % update q state and return trajectory
            self.q = qMatrix(end,:);
            traj = qMatrix;
        end

        function traj = interpolationTrajectoryTo(self,qp)
            % compute number of steps based on maximum required angular
            % position delta:
            angularTrajDist = max(abs(qp-self.q));
            angularStep = 0.07; % approx. 4 deg.
            steps = double(round(angularTrajDist/angularStep));            
            % compute attendant matrix of joint states
            qMatrix = jtraj(self.q,qp,steps);
            % update q state and return trajectory
            self.q = qMatrix(end,:);
            traj = qMatrix;
        end

        % implements RRMC for 5 DoF robot based on input end-effector
        % velocity (m/s)
        function nextQ = RRMCNextQ(self, eeVel)
            dt = 0.001; % physical seconds per animation step – eg. 0.001 renders 1 mm/step
            J = self.robot.model.jacob0(self.q);
            invJ = inv(J(1:5,:));
            jointVel = invJ*[eeVel 0 0]';
            nextQ = self.q + (jointVel*dt)';
            self.q = nextQ
        end
    end
end

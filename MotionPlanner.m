%{ 
This class is responsible for a robot's motion planning and the maintenance 
of state information, such as joint state q. generates trajectories for a general, static SerialLink object
passed in through constructor-arguement.
%}

classdef MotionPlanner < handle
    properties (Constant)
        endEffectorStepSize = 20 *1e-3; % m
        checkerPieceHeight = 5 *1e-3; % m
        qHome = [0 -pi/2 3*pi/4 0 0 0];
    end

    properties
        robot % SerialLink model
        q % ground truth for robot's joint state
        Tboard % transform to bottom left of board top (ie. playing surface)
        % x-y plane coincident with surface, z points up and y direction is
        % from white-piece team towards black-piece team
        squareSize % board square side length
    end

    methods
        % constructor receives information about the robot model and
        % checker board
        function obj = MotionPlanner(serialLink, Tboard, squareSize, Tbase)
            if nargin > 3
                obj.robot = serialLink;
                obj.q = obj.qHome;
                obj.robot.base = Tbase;
                obj.Tboard = Tboard;
                obj.squareSize = squareSize;
            % no robot base transform Tbase provided --> assumes origin
            % base:
            elseif nargin > 2
                obj.robot = serialLink;
                obj.q = qInit;
                obj.Tboard = Tboard;
                obj.squareSize = squareSize;
            else
                error("Must pass SerialLink object, initial joint-state " + ...
                    "vector and checker board parameters as construction" + ...
                    " arguements.");
            end
        end

        % function that returns cartesian trajectory to arguement 
        % transform (double) and updates q
        function traj = cartesianTrajectoryTo(self,Tp)
            % set transforms and points of trajectory extrema:
            T0 = self.robot.fkine(self.q).T;
            P0 = transl(T0);
            Pp = transl(Tp);

            % compute fractional increment by which end-effector will 
            % track the trajectory:
            trajDistance = sqrt(sum((Pp-P0).^2));
            stepDistance = self.endEffectorStepSize; % mm x 1e-3 m/mm
            steps = double(round(trajDistance/stepDistance));            
            fractDists = jtraj(0,1,steps);
            
            % compute attendant matrix of joint states
            trajTransforms = ctraj(T0, Tp, fractDists);
            qMatrix = zeros(steps,self.robot.n);
            qMatrix(1,:) = self.q;
            for i=2:steps
                % perhaps change to .ikine6s if 6 DoF used ultimately 
                qMatrix(i,:) = self.robot.ikcon(trajTransforms(:,:,i),qMatrix(i-1,:));
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

        % this function returns trajectory that implements a checkers move
        function traj = trajectoryMakeMove(self,boardPos0,boardPosP)
            % transforms to board positions (half the height of a checkers'
            % piece above particular square center):
            Tpos0 = self.Tboard*transl([(boardPos0-0.5).*self.squareSize self.checkerPieceHeight/2]);
            TposP = self.Tboard*transl([(boardPosP-0.5).*self.squareSize self.checkerPieceHeight/2]);

            % transforms to points above target board positions:
            Tabove0 = Tpos0*transl(0, 0, 8*self.checkerPieceHeight);
            TaboveP = TposP*transl(0, 0, 8*self.checkerPieceHeight);

            % generate the component trajectories between interstitial 
            % waypoints of composite (full) trajectory:
            trajToAbove0 = self.cartesianTrajectoryTo(Tabove0);
            trajAbove0ToPos0 = self.cartesianTrajectoryTo(Tpos0);
            trajPos0ToAbove0 = self.cartesianTrajectoryTo(Tabove0);
            trajAbove0ToAboveP = self.cartesianTrajectoryTo(TaboveP);
            trajAbovePToPosP = self.cartesianTrajectoryTo(TposP);
            trajPosPToAboveP = self.cartesianTrajectoryTo(TaboveP);
            trajAbovePToHome = self.trajectoryToHome();
            
            % compile and return composite (full) trajectory:
            size1 = size(trajToAbove0,1);
            size2 = size(trajAbove0ToPos0,1);
            size3 = size(trajPos0ToAbove0,1);
            size4 = size(trajAbove0ToAboveP,1);
            size5 = size(trajAbovePToPosP,1);
            size6 = size(trajPosPToAboveP,1);
            size7 = size(trajAbovePToHome,1);
            traj = zeros(size1+size2+size3+size4+size5+size6+size7, self.robot.n);
            traj(1:size1,:) = trajToAbove0;
            traj(size1+1:size1+size2,:) = trajAbove0ToPos0;
            traj(size1+size2+1:size1+size2+size3,:) = trajPos0ToAbove0;
            traj(size1+size2+size3+1:size1+size2+size3+size4,:) = trajAbove0ToAboveP;
            traj(size1+size2+size3+size4+1:size1+size2+size3+size4+size5,:) = trajAbovePToPosP;
            traj(size1+size2+size3+size4+size5+1:size1+size2+size3+size4+size5+size6,:) = trajPosPToAboveP;
            traj(size1+size2+size3+size4+size5+size6+1:size(traj,1),:) = trajAbovePToHome;
        end

        function traj = trajectoryToHome(self)
            traj = self.interpolationTrajectoryTo(self.qHome);
        end
    end
end











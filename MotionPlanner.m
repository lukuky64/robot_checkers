%{ 
This class is responsible for a robot's motion planning and the maintenance 
of state information. generates trajectories for a general, static SerialLink object
passed in through constructor-arguement.
%}

classdef MotionPlanner
    properties (Constant)
        endEffectorStepSize = 10 % mm
        checkerPieceHeight = 5 % mm
    end

    properties
        robot % SerialLink model
        q % ground truth for robot's joint state
        Tboard % transform to center of top of board (ie. playing surface)
        % need to specify orientation of white/black side
        squareSize % board square side length
    end

    methods
        % constructor receives information about the robot model and
        % checker board
        function obj = MotionPlanner(serialLink, qInit, Tboard, squareSize, Tbase)
            if nargin > 4
                obj.robot = serialLink;
                obj.q = qInit;
                obj.robot.base = Tbase;
                obj.Tboard = Tboard;
                obj.squareSize = squareSize;
            % no robot base transform Tbase provided --> assumes origin
            % base:
            elseif nargin > 3 
                obj.robot = serialLink;
                obj.q = qInit;
                obj.Tboard = Tboard;
                obj.squareSize = squareSize;
            else
                error("Must pass SerialLink object, initial joint-state vector and checker board parameters as construction arguements.");
            end
        end

        % function that returns cartesian trajectory to arguement transform (double)
        function traj = cartesianTrajectoryTo(self,Tp)
            T0 = self.robot.fkine(self.q).T;
            P0 = transl(T0);
            Pp = transl(Tp);
            trajDistance = sqrt(sum((Pp-P0).^2));
            stepDistance = self.endEffectorStepSize *1e-3; % mm x 1e-3 m/mm
            steps = double(round(trajDistance/stepDistance));            
            fractDists = jtraj(0,1,steps);
            trajTransforms = ctraj(T0, Tp, fractDists);
            qMatrix = zeros(steps,self.robot.n);
            qMatrix(1,:) = self.q;
            for i=2:steps
                % perhaps change to .ikine6s if 6 DoF used ultimately 
                qMatrix(i,:) = self.robot.ikcon(trajTransforms(:,:,i),qMatrix(i-1,:));
            end
            traj = qMatrix;
        end
    end
end











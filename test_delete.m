close all
clear all
clf

%% THIS SECTION FOR SIMPLE TESTING OF ABILITY OF COBOT TO REALISE DESIRED 
% END-EFFECTOR FRAMES...
close all;
display("New test...")
T = transl(0,.2,.05);
q = cobot.ikine(T,'forceSoln','mask',[1 1 1 0 0 1]) % NOTE WE NEED THE END
% EFFECTOR TO BE VERTICAL HERE, SO NEED SOME DEGREE OF ORIENTATION CONTROL
cobot.plot(q,'jointdiam',.2);
board = Board(squareSize*8,boardHeight,Tboard);
board.plotBoard();
trplot(T,'axes','rgb')



%%
%--------%--------%--------%--------%--------%--------%--------%--------%--------
% NEXT 2 SECTIONS FOR TUNING COBOT MOTION IN ACTUA CODE – GET THE ABOVE
% WORKING FIRST...
%--------%--------%--------%--------%--------%--------%--------%--------
%% RUN THIS FIRST TO SETUP ENVIRONMENT
dobotQ0 = [pi/2 0 pi/4 3*pi/4 -pi/2]; % dobot's qHome
        cobotQ0 = [pi/2 pi/8 3*pi/2 -3*pi/8 -pi/2 0]; %--------
        squareSize = .035; % checkers square size [m] (if change, change Tboard)
        boardHeight = .05; % checkers board height [m] (if change, change Tboard)
        Tboard = transl(-(.035*8)/2,.04,.05); % checkers board transform (ensure no rotation wrt. world)
        TbinDobot = transl(-.2,.2,0);
        TbinCobot = transl(0,0,0); %---------
        cobotBaseBoardGap = .05;
            TcobotBase = transl(0,squareSize*8+Tboard(2,4)+ ...
                cobotBaseBoardGap,0)*rpy2tr(0,0,pi/2);
        cobotRobotBaseClass = MyCobot320(TcobotBase);
            cobot = cobotRobotBaseClass.model;
            T = cobot.ikine(transl(.2,.2,.2));
            cobot.animate(T);
        board = Board(squareSize*8,boardHeight,Tboard);
board.plotBoard();
mp = MotionPlanner(cobot,cobotQ0,Tboard,squareSize,TbinCobot,'cobot');

%% EXPERIMENT WITH CHANGES TO mp.cartesianTrajectoryTo() HERE; IF YOU CHANGE
% MOTIONPLANNER BE SURE TO REVERSE CHANGES 
cobot.qlim(1:5,:) = deg2rad([-165 90; 0 90; 0 90; -90 0; -90 0]);

% set transforms and points of trajectory extrema:
T0 = cobot.fkine(cobotQ0).T;
P0 = transl(T0);
% cobot ee adjustment:
Tp = Tp*inv(transl(0,.095,0)*rpy2tr(-3*pi/2,0,0));
Pp = transl(Tp);

% compute fractional increment by which end-effector will 
% track the trajectory:
trajDistance = sqrt(sum((Pp-P0).^2));
if preciseModeOn == 1
    stepDistance = self.endEffectorStepSizePrecise; % mm x 1e-3 m/mm
else
    stepDistance = self.endEffectorStepSize; % mm x 1e-3 m/mm
end
steps = double(round(trajDistance/stepDistance));            
fractDists = jtraj(0,1,steps);

% compute attendant matrix of joint states
trajTransforms = ctraj(T0, Tp, fractDists);
qMatrix = zeros(steps,self.robot.n);
qMatrix(1,:) = self.q;
if self.IKmethod == 'dobot'
    for i=2:steps
        qMatrix(i,:) = self.robot.ikine(trajTransforms(:,:,i), ...
            'q0',qMatrix(i-1,:),'mask',[1 1 1 0 1 1],'forceSoln');
    end
elseif self.IKmethod == 'cobot'
    for i=2:steps
        qMatrix(i,:) = self.robot.ikcon(trajTransforms(:,:,i),qMatrix(i-1,:));% ...
            %'q0',qMatrix(i-1,:),'mask',[1 1 1 0 0 1],'forceSoln');
    end
else
    display("MotionPlanner property IKmethod must be set by constructor-arguement to either 'ikine' or 'ikcon'.");
end

% update q state and return trajectory
self.q = qMatrix(end,:);
traj = qMatrix;

%{
traj = mp.cartesianTrajectoryTo(transl(0, 0, 0.05));
for i=1:size(traj,1)
    cobot.animate(traj(i,:));
    pause(.12);
end
%}
%%
p = Player(cobot,cobotQ0,Tboard,squareSize,TbinCobot,'cobot');
traj = p.processTaskTrajectory({[],[1,1;3,3],[2,2]});
%%
board = Board(squareSize*8,boardHeight,Tboard);
board.plotBoard();
for i=1:size(traj,1)
    cobot.animate(traj(i,:));
    pause(1);
end
%%
rb = MyCobot320();
r = rb.model;
Y = transl(.2,.2,.2);
q = r.ikcon(Y);
r.animate(q)
T = transl(0,.095,0)*rpy2tr(-pi/2,0,0)
hold on
trplot(Y*T,'axes','rgb');
hold off
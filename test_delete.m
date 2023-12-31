close all
clear all
clf

%% test cobot (next 3)
squareSize = .035; % checkers square size [m] (if change, change Tboard)
boardHeight = .05; % checkers board height [m] (if change, change Tboard)
Tboard = transl(-(.035*8)/ ...
    2,.09,.05); % checkers board transform (ensure no rotation wrt. world)
TbinCobot = transl(.2,.2,0); %---------

cobotQ0 = [pi/2 pi/8 3*pi/4 -3*pi/8 -pi/2 0];
        cobotQready = deg2rad([0 25 90 -45 -90 0]);

cobotBaseBoardGap = .10;
    TcobotBase = transl(0,squareSize*8+Tboard(2,4)+ ...
        cobotBaseBoardGap,0)*rpy2tr(0,0,pi/2);
cobotRobotBaseClass = MyCobot320(TcobotBase);
    cobot = cobotRobotBaseClass.model;
    
%board = Board(squareSize*8,boardHeight,Tboard);
%board.plotBoard();
p = Player(cobot,cobotQ0, ...
    Tboard,squareSize, TbinCobot,'cobot', ...
    'cobotQready',cobotQready);

cobot.animate(cobotQ0);

%%
traj = p.processTaskTrajectory({0,[1,1;8,8],[2,2]});

%%
for i=1:size(traj,1)
    cobot.animate(traj(i,:));
    pause(.2);
end

%% test dobot (next 3)
squareSize = .035; % checkers square size [m] (if change, change Tboard)
boardHeight = .05; % checkers board height [m] (if change, change Tboard)
Tboard = transl(-(.035*8)/ ...
    2,.09,.05); % checkers board transform (ensure no rotation wrt. world)
TbinDobot = transl(-.2,.2,0); %---------
dobotQ0 = [pi/2 0 pi/4 3*pi/4 -pi/2];

dobotRobotBaseClass = DobotMagician(rpy2tr(0,0,pi/2));
dobot = dobotRobotBaseClass.model;

board = Board(squareSize*8,boardHeight,Tboard);
board.plotBoard();

p = Player(dobot,dobotQ0, ...
    Tboard,squareSize, TbinDobot,'dobot');

dobot.animate(dobotQ0);

%%
traj = p.processTaskTrajectory({0,[1,1;3,3],[2,2]});

%%
for i=1:size(traj,1)
    dobot.animate(traj(i,:));
    pause(.1);
end


%% THIS SECTION FOR SIMPLE TESTING OF ABILITY OF COBOT TO REALISE DESIRED 
% END-EFFECTOR FRAMES...
close all;
display("New test...")
T = transl(0,.2,.05);
q = cobot.ikine6s(T)
%q = cobot.ikine(T,'forceSoln','mask',[1 1 1 0 0 1],'slimit',1000) % NOTE WE NEED THE END
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
squareSize = .035; % checkers square size [m] (if change, change Tboard)
        boardHeight = .05; % checkers board height [m] (if change, change Tboard)
        Tboard = transl(-(.035*8)/ ...
            2,.04,.05); % checkers board transform (ensure no rotation wrt. world)
        TbinDobot = transl(-.2,.2,0);
        TbinCobot = transl(0,0,0); %---------
        cobotBaseBoardGap = .14;
            TcobotBase = transl(0,squareSize*8+Tboard(2,4)+ ...
                cobotBaseBoardGap,0)*rpy2tr(0,0,pi/2);
        cobotRobotBaseClass = MyCobot320(TcobotBase);
            cobot = cobotRobotBaseClass.model;
        board = Board(squareSize*8,boardHeight,Tboard);
board.plotBoard();
mp = MotionPlanner(cobot,cobotQ0,Tboard,squareSize,TbinCobot,'cobot');

%% EXPERIMENT WITH CHANGES TO mp.cartesianTrajectoryTo() HERE; IF YOU CHANGE
% MOTIONPLANNER BE SURE TO REVERSE CHANGES
cobotQ0 = [pi/2 pi/8 3*pi/4 -3*pi/8 -pi/2 0]
cobotQready = deg2rad([0 25 90 -45 -90 0])
%cobot.qlim(1:5,:) = deg2rad([-165 90; -67.5 135; -22.5 135; -90 90; -90 90]);
cobot.qlim(3,:) = deg2rad([-1 1]*155);

% set transforms and points of trajectory extrema:
T0 = cobot.fkine(cobotQ0).T;
P0 = transl(T0);
Tp = cobot.fkine(cobotQready).T
trplot(Tp,'axes');
% cobot ee adjustment because TCP offset:
Tpp = Tp*rpy2tr(pi,0,0)*inv(transl(0,.095,0)*rpy2tr(-pi/2,0,0));
Pp = transl(Tp);
% compute fractional increment by which end-effector will 
% track the trajectory:
trajDistance = sqrt(sum((Pp-P0).^2));
stepDistance = 0.010; % mm x 1e-3 m/mm
steps = double(round(trajDistance/stepDistance));            
fractDists = jtraj(cobotQ0,cobotQready,steps);
traj = fractDists;

% set transforms and points of trajectory extrema:
T0 = cobot.fkine(cobotQready).T;
P0 = transl(T0);
Tp = transl(0,.3,.05);
% cobot ee adjustment:
Tpp = Tp*rpy2tr(pi,0,0)*inv(transl(0,.095,0)*rpy2tr(-pi/2,0,0));
Pp = transl(Tpp);
% compute fractional increment by which end-effector will 
% track the trajectory:
trajDistance = sqrt(sum((Pp-P0).^2));
stepDistance = 0.010; % mm x 1e-3 m/mm
steps = double(round(trajDistance/stepDistance));            
fractDists = jtraj(0,1,steps);
% compute attendant matrix of joint states
trajTransforms = ctraj(T0, Tpp, fractDists);
qMatrix = zeros(steps,cobot.n);
qMatrix(1,:) = cobotQready;
for i=2:steps
    qMatrix(i,:) = cobot.ikcon(trajTransforms(:,:,i),qMatrix(i-1,:));% ...
        %'q0',qMatrix(i-1,:),'mask',[1 1 1 0 0 1],'forceSoln');
end
traj = [traj; qMatrix];

trplot(Tp,'axes','rgb');
%trplot(Tpp,'axes','rgb');
for i=1:size(traj,1)
    cobot.animate(traj(i,:));
    pause(.2);
end

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
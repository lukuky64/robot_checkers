close all
clear all
clf
%%
%--------%--------%--------%--------%--------%--------%--------%--------%--------
% NEXT 2 SECTIONS FOR TUNING COBOT MOTION
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
        board = Board(squareSize*8,boardHeight,Tboard);
board.plotBoard();
%% EXPERIMENT WITH CHANGES TO mp.cartesianTrajectoryTo() HERE; IF YOU CHANGE
% MOTIONPLANNER BE SURE TO REVERSE CHANGES 
mp = MotionPlanner(cobot,cobotQ0,Tboard,squareSize,TbinCobot,'cobot');
mp.robot.qlim(1:5,:) = deg2rad([-165 90; 0 90; 0 90; -90 0; -90 0]);
mp.qHome = cobotQ0;
traj = mp.cartesianTrajectoryTo(transl(0, 0, 0.05));
for i=1:size(traj,1)
    cobot.animate(traj(i,:));
    pause(.12);
end

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
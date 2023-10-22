close all
clear all
clf

%% create robot and instanticate its motion planner
robotObj = DobotMagician;
%%
squareSize = .035;
boardLength = squareSize*8;
boardHeight = 0.05;
robot = robotObj.model;
Tboard = transl(-boardLength/2,.04,boardHeight);
mp = MotionPlanner(robot, Tboard, squareSize, transl(-.2,.2,0), rpy2tr(0,0,pi/2));
%% test Player

p = Player(robot, Tboard, squareSize, transl(-.2,.2,0), rpy2tr(0,0,pi/2));
task = {1,[1,1;5,5],[2,2;4,4]};
traj = p.processTaskTrajectory(task);
%%
mp.q = mp.qHome;
b = Board(boardLength,boardHeight,Tboard);
hold on
axis([-.5 .5 -.2 1.2 -.2 .5]);
b.plotBoard();
trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
robot.plot(traj,'loop','trail','r-','nojoints','jointdiam',2);
hold off
%%
b = Board(boardLength,boardHeight,Tboard);
hold on
b.plotBoard();
trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
mp.robot.teach();
axis([-.3 .3 -.2 .6 -.2 .6])


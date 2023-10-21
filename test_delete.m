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
%%
mp.q = mp.qHome;
traj1 = mp.trajectoryMoveToSquare([2,2]);
traj2 = mp.trajectorySquare2Square([2,2],[8,8]);
traj3 = mp.trajectorySquare2Bin([8 8]);
traj4 = mp.trajectoryMoveToAboveBin();
traj5 = mp.trajectoryToHome();
%%
b = Board(boardLength,boardHeight,Tboard);
hold on
axis([-.5 .5 -.2 1.2 -.2 .5]);
b.plotBoard();

trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
robot.plot([traj1;traj2;traj3;traj4;traj5],'loop','trail','r-','nojoints','jointdiam',2);
hold off
%%
b = Board(boardLength,boardHeight,Tboard);
hold on
b.plotBoard();
trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
mp.robot.teach();
axis([-.3 .3 -.2 .6 -.2 .6])


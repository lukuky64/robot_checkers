close all
clear all
clf

%% create robot and instantiate its motion planner
robotObj = UR5;
robot = robotObj.model;
Thome = transl(0, .3, .4);
Tboard = transl(0,.7,0);
mp = MotionPlanner(robot, Tboard, .01, rpy2tr(0,0,-pi/2));
%% 
traj = mp.trajectoryMakeMove([2 2],[5 5]);
axis([-1 1 -.4 2 -.4 1]);
hold on
trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
robot.plot(traj,'loop','trail','r-','nojoints','jointdiam',2);
hold off
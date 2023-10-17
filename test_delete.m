close all
clear all
clf

%% create robot and instantiate its motion planner
robotObj = DobotMagician;
robot = robotObj.model;
Tboard = transl(0,.2,0);
mp = MotionPlanner(robot, Tboard, .01, rpy2tr(0,0,-pi/2));
%% 
traj = mp.trajectoryMakeMove([2 2],[5 5]);
axis([-.5 .5 -.2 1.2 -.2 .5]);
hold on
trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
robot.plot(traj,'loop','trail','r-','nojoints','jointdiam',2);
hold off
%%
mp.RRMCNextQ([1,1,1]);
mp.robot.plot(mp.q);
%%
eeVel = [.5 .5 .5];
q = [1 1 1 1 1];
dt = 0.001; % physical seconds per animation step – eg. 0.001 renders 1 mm/step
            J = robot.jacob0([1 1 1 1 1]);
            invJ = inv(J(1:5,:));
            jointVel = invJ*[eeVel 0 0]';
            nextQ = q + (jointVel*dt)'

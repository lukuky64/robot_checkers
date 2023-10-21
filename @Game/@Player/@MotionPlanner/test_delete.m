close all
clear all
clf

%% create robot and instanticate its motion planner
robotObj = DobotMagician;
%%
squareSize = .03;
boardLength = squareSize*8;
boardHeight = 0.05;
robot = robotObj.model;
Tboard = transl(-boardLength,.04,boardHeight);
mp = MotionPlanner(robot, Tboard, squareSize, rpy2tr(0,0,pi/2, transl(-.2,.2,0)));
%%
traj1 = mp.trajectoryMoveToSquare([2,2]);
traj2 = mp.trajectorySquare2Square([2,2],[8,8]);
traj3 = mp.trajectorySquare2Bin([8 8]);
traj4 = mp.trajectoryToHome();
%%
b = Board(boardLength,boardHeight,Tboard);
hold on
axis([-.5 .5 -.2 1.2 -.2 .5]);
b.plotBoard();

trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
robot.plot([traj1;traj2;traj3;traj4],'loop','trail','r-','nojoints','jointdiam',2);
hold off
%%
b = Board(boardLength,boardHeight,Tboard);
hold on
b.plotBoard();
trplot(Tboard,'framelabel','Checkers Board','rgb','arrow');
mp.robot.teach();
axis([-.3 .3 -.2 .6 -.2 .6])

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
%%
hold on
PlaceObject('Checkers_Board.ply');
axis equal
camlight
axis([-.5 .5 -.2 1.2 -.2 .5]);
%%
hold on
b = Board(.32,.05,transl(.2,.2,.2));
b.plotBoard();
xlabel('x')
ylabel('y')

%%
            checkersRot = t2r(b.base);
            checkersNorm = checkersRot(1:3,3)';
            [s1,im1] = b.plotIm('CheckersS.jpeg',400,checkersNorm,(transl(b.base)+ ...
                [b.length, b.length, 0]'/2), b.length, b.length);
            set(s1, 'faceColor', 'texture',...
                'edgecolor', 'none',...
                'cdata', im1)

            hold on

            % sides x-x, then y-y
            sideRot = t2r(b.base);
            sideNorm = sideRot(1:3,1)';
            [s2,im2] = b.plotIm('Wood_Texture.jpeg',400,sideNorm,(transl(b.base)+ ...
                [0 b.length -b.height]'/2), b.length, b.height);
            set(s2, 'faceColor', 'texture',...
                'edgecolor', 'none',...
                'cdata', im2)

            hold off





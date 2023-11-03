clf
%g = Game();

%%
[t1,x1] = g.playerRed.processTaskTrajectory({[],[1,1; 5,5],[2,2; 4,4]});
%[t2,x2] = g.playerBlue.processTaskTrajectory({[],[7,7;7,3],[6,6; 6,4]});
%%

pause(4)
g.animator.animatePlayerMove(t1,x1,'robot','dobot');
%g.animator.animatePlayerMove(t2,x2,'robot','cobot');

g = Game();
%%
t = g.playerBlue.processTaskTrajectory({[],[1,1; 5,5],[2,2; 4,4]});
g.animator.animatePlayerMove(t,'robot','cobot');
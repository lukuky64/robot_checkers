%%
robot = DobotMagician_succ

%% Define line
point1OnLine = [0.5, 0, 0.25];
point2OnLine = [-0.5, 0, 0.25];

line = {point1OnLine, point2OnLine};

%% Create curtain
curtain = LightCurtain()

%% Check for collision
result = curtain.CheckCollision(line);
disp(result);
%% define robot and prisms
robot = DobotMagician;

%%
% Define multiple prisms ([minX, minY, minZ],[maxX, maxY, maxZ])
prisms = {
    [-0.2, -1.2, -0.5], [-0.201, 1.2,   0.5];  % wall
    [0.4, -1.2,  -0.5], [0.401,  1.2,   0.5];  % wall
    [-0.2, -0.5,  0.0], [0.4,    0.5, 0.001]}; % ground plane

[vertex, faces, faceNormals] = checkCollision.createCollisionPrisms(prisms);

%% Animate trajectory
q0 = robot.model.getpos;
q1 = [pi, 0.7854, 1.5708, 0.7854, 0];
qMatrix = jtraj(q0,q1, 40);

for i = 1:40
    result = checkCollision.IsCollision(robot.model,qMatrix(i,:),faces,vertex,faceNormals,true); % collision detection part
    if result
        return;
    end
    robot.model.animate(qMatrix(i, :));
    pause(0.04);
end










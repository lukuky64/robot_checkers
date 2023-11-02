%%
robot = DobotMagician_succ

%% Define line
point1OnLine = [1.0, 0, 0.25];
point2OnLine = [0.8, 0, 0.25];
line = {point1OnLine, point2OnLine};
hold on
visualLine = plot3([line{1}(1),line{2}(1)], [line{1}(2),line{2}(2)], [line{1}(3),line{2}(3)], 'b-');
axis equal;
%% Create curtain
curtain = LightCurtain()

%% Check for collision
pause(3);
for i = 1:100
    hold on
    point1OnLine = [1-(i/100), 0, 0.25];
    point2OnLine = [0.8-(i/100), 0, 0.25];
    line = {point1OnLine, point2OnLine      };
    delete(visualLine);
    visualLine = plot3([line{1}(1),line{2}(1)], [line{1}(2),line{2}(2)], [line{1}(3),line{2}(3)], 'b-');
    result = curtain.CheckCollision(line);
    if result
        break;
    end
    pause(0.01)
end

disp(result);
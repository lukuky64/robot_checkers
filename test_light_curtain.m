%%
robot = DobotMagician_succ

%% Define line
point1OnLine = [1.0, 0, 0.25];
point2OnLine = [0.8, 0, 0.25];
line = {point1OnLine, point2OnLine};
hold on
visualLine = plot3([line{1}(1),line{2}(1)], [line{1}(2),line{2}(2)], [line{1}(3),line{2}(3)], 'b-');
axis equal;
axis([-1, 1, -1, 1, -1, 1]);

%% Create curtain
curtain = LightCurtain()
hold on
animateTable()

%% Check for collision
pause(5);

input("enter:")
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



%%
function animateTable()
            %offsets change the location of the object
            xOffset = 0;
            yOffset = .2;
            zOffset = -1;
            % enter name of ply file to be displayed
            [f,v,data] = plyread('Scenery_complete.ply','tri'); 
            % sets vertex colours in rgb values from ply file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
            %plotting
            trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
        end
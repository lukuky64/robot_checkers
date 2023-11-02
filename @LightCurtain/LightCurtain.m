classdef LightCurtain
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        vertex
        faces
        faceNormals
    end

    methods
        function obj = LightCurtain()
            prisms = {[-0.6, -0.3, 0], [0.4, 0.6,   0.6]};  % wall
            [obj.vertex, obj.faces, obj.faceNormals] = checkCollision.createCollisionPrisms(prisms, 0.01);
        end


        function result = CheckCollision(obj, line_)
            result = false;
            for faceIndex = 1:size(obj.faces, 1)
                % Get a vertex on the obstacle face
                vertOnPlane = obj.vertex(obj.faces(faceIndex, 1), :);
                
                % Check for line-plane intersection
                [intersectP, check] = LinePlaneIntersection(obj.faceNormals(faceIndex, :), vertOnPlane, line_{1}, line_{2});
                
                % Check if the intersection point lies inside the triangle
                if check == 1 && checkCollision.IsIntersectionPointInsideTriangle(intersectP, obj.vertex(obj.faces(faceIndex, :), :))
                    hold on
                    plot3(intersectP(1), intersectP(2), intersectP(3), 'r*'); % plot the point of intersection
                    hold off
                    disp('Light Curtain Activated!');
                    msgbox('Light Curtain Activated!', 'Warning', 'warn');
                    result = true;
                end
            end
        end
    end
end
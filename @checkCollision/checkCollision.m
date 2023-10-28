classdef checkCollision < handle

    % checkCollision: Class for checking robot-obstacle collision
    % ----------------------------------------------------------
    % This class is derived from Lab 5 exercise of subject 41013
    % ----------------------------------------------------------
    
    properties
        % No properties defined, all methods are static
    end
    
    methods(Static)
        
        % Main collision detection function
        function result = IsCollision(robot, qMatrix, faces, vertex, faceNormals, returnOnceFound)
            % Check for optional 'returnOnceFound' argument
            if nargin < 6
                returnOnceFound = true;
            end
            
            % Initialize result to false (no collision)
            result = false;

            % Loop through each set of joint angles in qMatrix
            for qIndex = 1:size(qMatrix, 1)
                
                % Get the transformation matrices for each link
                tr = checkCollision.GetLinkPoses(qMatrix(qIndex, :), robot);
                
                % Check each link against each obstacle face
                for i = 1:size(tr, 3) - 1
                    for faceIndex = 1:size(faces, 1)
                        % Get a vertex on the obstacle face
                        vertOnPlane = vertex(faces(faceIndex, 1), :);
                        
                        % Check for line-plane intersection
                        [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex, :), vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i + 1)');
                        
                        % Check if the intersection point lies inside the triangle
                        if check == 1 && checkCollision.IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex, :), :))
                            hold on
                            plot3(intersectP(1), intersectP(2), intersectP(3), 'g*'); % plot the point of intersection
                            hold off
                            disp('Intersection detected!');
                            msgbox('Collision Detected!', 'Warning', 'warn');
                            result = true;
                            
                            % If one intersection is enough, exit the function
                            if returnOnceFound
                                return;
                            end
                        end
                    end
                end
            end
        end
        
        % Function to get the transformation matrices for each link of the robot
        function transforms = GetLinkPoses(q, robot)
            links = robot.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:, :, 1) = robot.base;
            
            % Loop through each link to compute its transformation matrix
            for i = 1:length(links)
                L = links(1, i);
                current_transform = transforms(:, :, i);
                current_transform = current_transform * trotz(q(1, i) + L.offset) * ...
                    transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
                transforms(:, :, i + 1) = current_transform;
            end
        end
        
        % Function to check if a point is inside a given triangle
        function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
            % Compute vectors related to the triangle
            u = triangleVerts(2, :) - triangleVerts(1, :);
            v = triangleVerts(3, :) - triangleVerts(1, :);
            w = intersectP - triangleVerts(1, :);
            
            % Compute dot products
            uu = dot(u, u);
            uv = dot(u, v);
            vv = dot(v, v);
            wu = dot(w, u);
            wv = dot(w, v);
            
            % Compute determinant
            D = uv * uv - uu * vv;
            
            % Compute parametric coordinates (s, t) and check if the point lies inside the triangle
            s = (uv * wv - vv * wu) / D;
            t = (uv * wu - uu * wv) / D;
            if (s >= 0 && s <= 1) && (t >= 0 && (s + t) <= 1)
                result = 1;  % Inside triangle
            else
                result = 0;  % Outside triangle
            end
        end

        function [vertex, faces, faceNormals] = createCollisionPrisms(prisms)
            % Initialize plotOptions as an empty struct
            plotOptions = struct;
            plotOptions.plotFaces = true;
            plotOptions.FaceColor = [0.8, 0.8, 0.8];
            plotOptions.EdgeColor = 'k';
            plotOptions.FaceAlpha = 0.01;
        
            vertex = [];
            faces = [];
            faceNormals = [];
            
            % Loop through each prism to collect vertex, faces, and faceNormals
            for i = 1:size(prisms, 1)
                [v, f, n] = checkCollision.RectangularPrism(prisms{i, 1}, prisms{i, 2}, plotOptions);
                
                % Update faces to match new vertex indices
                f = f + size(vertex, 1);
                
                % Concatenate the new vertex, faces, and faceNormals
                vertex = [vertex; v];
                faces = [faces; f];
                faceNormals = [faceNormals; n];
            end
        end

        function [vertex,face,faceNormals] = RectangularPrism(lower,upper,plotOptions,axis_h)
            if nargin<4
                    axis_h=gca;
            end
            hold on
            
            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;
            
            face=[1,2,3;1,3,7;
                 1,6,5;1,7,5;
                 1,6,4;1,4,2;
                 6,4,8;6,5,8;
                 2,4,8;2,3,8;
                 3,7,5;3,8,5;
                 6,5,8;6,4,8];
            
            if 2 < nargout    
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            %% If plot verticies
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1);
                    plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end
            
            %% If you want to plot the edges
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];
            
                for i=1:size(links,1)
                    plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
                end
            end
            
            %% To plot the edges
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                tcolor = [0.2, 0.2, 0.8];  % Your desired color
                patch('Faces',face,'Vertices',vertex,'FaceColor',tcolor,'LineStyle','none', 'FaceAlpha', plotOptions.FaceAlpha, 'EdgeColor', plotOptions.EdgeColor);
            end


            hold off
        end

    end
end

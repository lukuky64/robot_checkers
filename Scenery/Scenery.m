%plyread test script

% f = faceData;
% v = vertexData;


hold on

%offsets change the location of the object
xOffset = 0;
yOffset = 0;
zOffset = 0;



[f,v,data] = plyread('Scenery.ply','tri'); % enter name of ply file to be displayed
<<<<<<< HEAD
    
=======


>>>>>>> 5b6779e88cd643681d869541beb0d0b602ddb3ef
% sets vertex colours in rgb values from ply file
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;


%plotting
trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');

camlight;
axis equal;
view(3);


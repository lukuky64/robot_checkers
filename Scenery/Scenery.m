%plyread test script

% f = faceData;
% v = vertexData;

clf


%offsets change the location of the object
xOffset = 0;
yOffset = 0;
zOffset = 0;

<<<<<<< HEAD
[f,v,data] = plyread('checkerRed.ply','tri');
=======
[f,v,data] = plyread('Scenery.ply','tri'); % enter name of ply file to be displayed
>>>>>>> 031ae3ca1807a1dc41fb56f39046f16ceb0b73cf

% sets vertex colours in rgb values from ply file
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;


%plotting
trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');

camlight;
axis equal;
view(3);

<<<<<<< HEAD
xOffset = 0.05;
yOffset = 0.05;
zOffset = 0.05;

%clear c
%c = trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
=======
>>>>>>> 031ae3ca1807a1dc41fb56f39046f16ceb0b73cf

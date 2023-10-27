%plyread test script

% f = faceData;
% v = vertexData;

clf

xOffset = 0;
yOffset = 0;
zOffset = 0;

[f,v,data] = plyread('Scenery.ply','tri');

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;


% for xOffset = [-50 ,50]
%     for yOffset = [-50, 50]
%         trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3),'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
%     end
% end


c = trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
camlight;
axis equal;

view(3);

xOffset = 0.05;
yOffset = 0.05;
zOffset = 0.05;

pause(10)

clear c
c = trisurf(f,v(:,1)+ xOffset,v(:,2)+ yOffset, v(:,3)+ zOffset,'FaceVertexCData',vertexColours,'Edgecolor','interp','EdgeLighting','flat');
classdef CheckerPiece < handle
    properties
        base
        colour
        plotHandle
    end

    methods
        function self = CheckerPiece(colour, base)
            self.colour = colour;
            self.base = base;
            self.plotMe();
        end

        function plotMe(self)
            [f,v,data] = plyread('checkerRed.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
            self.plotHandle = trisurf(f,v(:,1)+ self.base(1,4),v(:,2)+ self.base(2,4), ...
                v(:,3)+ self.base(3,4),'FaceVertexCData',vertexColours, ...
                'Edgecolor','interp','EdgeLighting','flat');
            camlight;
        end

        function moveMe(self,T)
            delete(self.plotHandle);
            self.base = T;
            self.plotMe();
        end
     end 
end

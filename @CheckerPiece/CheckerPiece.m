classdef CheckerPiece < handle
    properties
        base
        colour
        plotHandle
        Toffset = transl(0,0,0)
    end

    methods
        function self = CheckerPiece(colour, base)
            self.colour = colour;
            self.base = base*self.Toffset;
            self.plotMe();
        end

        function plotMe(self)
            if strcmp(self.colour, 'red')
                [f,v,data] = plyread('checkerRed.ply','tri');
            elseif strcmp(self.colour, 'blue')
                [f,v,data] = plyread('checkerBlue.ply','tri');
            end
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
            self.plotHandle = trisurf(f,v(:,1)+ self.base(1,4),v(:,2)+ self.base(2,4), ...
                v(:,3)+ self.base(3,4),'FaceVertexCData',vertexColours, ...
                'Edgecolor','interp','EdgeLighting','flat');
        end

        function moveMe(self,T)
            delete(self.plotHandle);
            self.base = T*self.Toffset;
            self.plotMe();
        end
     end 
end

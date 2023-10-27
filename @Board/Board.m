classdef Board < handle
    properties
        length
        height
        base
        squareSize
        s
    end

    methods
        function self = Board(length,height,base)
            self.length = length;
            self.height = height;
            self.base = base;
            self.squareSize = self.length/8;
        end

        function [] = moveBoard(self, Tbase)
            for i=1:6
                delete(self.s(i))
            end
            self.base = Tbase;
            self.plotBoard();
        end

        function [] = plotBoard(self)
            hold on
            % top:
            checkersRot = t2r(self.base);
            checkersNorm = checkersRot(1:3,3)';
            self.s(1) = self.plotIm('CheckersS.jpeg',400,checkersNorm,(transl(self.base)+ ...
                [self.length, self.length, 0]'/2), self.length, self.length);
            % sides y-y, then x-x:
            sideRot = t2r(self.base);
            sideNorm = sideRot(1:3,1)';
            self.s(2) = self.plotIm('Wood_Texture2.jpeg',400,sideNorm,(transl(self.base)+ ...
                [0 self.length -self.height]'/2), self.length, self.height);
            self.s(3) = self.plotIm('Wood_Texture2.jpeg',400,sideNorm,(transl(self.base)+ ...
                [self.length self.length/2 -self.height/2]'), self.length, self.height);
            sideNorm = sideRot(1:3,2)';
            self.s(4) = self.plotIm('Wood_Texture2.jpeg',400,sideNorm,(transl(self.base)+ ...
                [self.length 0 -self.height]'/2), self.length, self.height);
            self.s(5) = self.plotIm('Wood_Texture2.jpeg',400,sideNorm,(transl(self.base)+ ...
                [self.length/2 self.length -self.height/2]'), self.length, self.height);
            % bottom:
            checkersRot = t2r(self.base);
            checkersNorm = checkersRot(1:3,3)';
            self.s(6) = self.plotIm('Wood_Texture2.jpeg',400,checkersNorm,(transl(self.base)+ ...
                [self.length/2, self.length/2, -self.height]'), self.length, self.length);
        end
    
        function s = plotIm(self, imPath, N, normalVector, centerPoint, length, width)
            w = null(normalVector); 
            [P,Q] = meshgrid(-length/2:length/N:length/2,-width/2:width/N:width/2); 
            X = centerPoint(1)+w(1,1)*P+w(1,2)*Q; 
            Y = centerPoint(2)+w(2,1)*P+w(2,2)*Q; 
            Z = centerPoint(3)+w(3,1)*P+w(3,2)*Q;
            im = imread(imPath);
            im = imresize(im,[N,N]);
            [im, map] = rgb2ind(im, 256);
            colormap(map);
            s = surf(X,Y,Z);
            set(s, 'faceColor', 'texture',...
                'edgecolor', 'none',...
                'cdata', im);
        end

        function [] = plotCheckers(self)
            checkersRot = t2r(self.base);
            checkersNorm = checkersRot(1:3,3)';
            centerPoint = (transl(self.base)+ ...
                [self.length, self.length, 0]'/2);
            w = null(checkersNorm); 
            [P,Q] = meshgrid([-self.length/2,self.length/2], ...
                [-self.length/2,self.length/2]); 
            X = centerPoint(1)+w(1,1)*P+w(1,2)*Q; 
            Y = centerPoint(2)+w(2,1)*P+w(2,2)*Q; 
            Z = centerPoint(3)+w(3,1)*P+w(3,2)*Q;
            s = surf(X,Y,Z);
            set(s, 'faceColor','k', ...
                'edgecolor', 'none');
            hold on
            checkersRot = t2r(self.base);
            checkersNorm = checkersRot(1:3,3)';
            self.plotIm('CheckersS.jpeg',400,checkersNorm,(transl(self.base)+ ...
                [self.length, self.length, 0]'/2), self.length, self.length);
            hold off
        end
    end
end
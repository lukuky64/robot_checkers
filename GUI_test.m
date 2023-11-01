%% Create two robots
baseTr_1 = transl(-0.4, 0, 0);
baseTr_2 = transl(0.4, 0, 0)*trotz(pi);

bot1 = DobotMagician(baseTr_1);
bot2 = MyCobot320(baseTr_2);

bots_ = {bot1, bot2};


%% Create interface instance
interface_ = JOG_GUI(bots_);

%% Create workspace visual objects


%%
hold on
base = transl(-0.175, -0.175, 0.05);
length = 0.35;
height = 0.05;
% Function to plot 3D textures on an object

% Utility function to extract rotation matrix
t2r = @(T) T(1:3, 1:3);

% Utility function to extract translation vector
transl = @(T) T(1:3, 4);

% top:
checkersRot = t2r(base);
checkersNorm = checkersRot(1:3,3)';
plotIm('CheckersS.jpeg', 400, checkersNorm, (transl(base) + [length, length, 0]'/2), length, length);

% sides y-y, then x-x:
sideRot = t2r(base);
sideNorm = sideRot(1:3,1)';
plotIm('Wood_Texture2.jpeg', 400, sideNorm, (transl(base) + [0 length -height]'/2), length, height);
plotIm('Wood_Texture2.jpeg', 400, sideNorm, (transl(base) + [length length/2 -height/2]'), length, height);

sideNorm = sideRot(1:3,2)';
plotIm('Wood_Texture2.jpeg', 400, sideNorm, (transl(base) + [length 0 -height]'/2), length, height);
plotIm('Wood_Texture2.jpeg', 400, sideNorm, (transl(base) + [length/2 length -height/2]'), length, height);

% bottom:
checkersRot = t2r(base);
checkersNorm = checkersRot(1:3,3)';
plotIm('Wood_Texture2.jpeg', 400, checkersNorm, (transl(base) + [length/2, length/2, -height]'), length, length);

hold off


%%
function [] = plotIm(imPath, N, normalVector, centerPoint, length, width)
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
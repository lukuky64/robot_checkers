classdef MyCobot320 < RobotBaseClass
    %% MyCobot280 - "Baby-Elephant Collaborative Robotic Arm"
    % URL: https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.1-280/2.1.4.1%20Introduction%20of%20product%20parameters.html
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!
    
    properties(Access = public)
        plyFileNameStem = 'MyCobot320';
        
        
    end
    methods (Access = public)
        %% Constructor
        function self = MyCobot320(baseTr)
            self.CreateModel();
            if nargin == 1
                self.model.base = self.model.base.T * baseTr;
            end
            
            % Overiding the default workspace for this small robot
            self.workspace = [-0.8 0.8 -0.8 0.8 -0.01 0.5];
            
            self.PlotAndColourRobot();
        end
        
        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d', 0.1739, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-165 165]));
            link(2) = Link('d', 0.08878, 'a', 0.135, 'alpha', 0,'offset',pi/2, 'qlim', deg2rad([-165 165]));
            link(3) = Link('d', -0.08878, 'a', 0.120, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(4) = Link('d', 0.08878, 'a', 0, 'alpha', pi/2,'offset',pi/2, 'qlim', deg2rad([-165 165]));
            link(5) = Link('d', 0.095, 'a', 0, 'alpha', pi/2,'offset',pi, 'qlim', deg2rad([-165 165]));
            link(6) = Link('d',0.0655, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-175 175]));
            
        % end effector parameters

<<<<<<< HEAD
             self.useTool = 1;
             self.toolFilename = 'MyCobot320EndEffector1.ply';
             self.toolTr = transl(0, 0.095, 0);
                       
=======
             self.useTool = 1;                                  % toggle on and off tool
             self.toolFilename = 'MyCobot320EndEffector1.ply';  %ply file that replaces final link
             self.toolTr = transl(0, 0.095, 0);                 % tip of the end effector                   
           
            
>>>>>>> 06a4e811960aa3d39c5e05afa77b7cba44cb5b3d
            self.model = SerialLink(link,'name',self.name);
        end
    end
end
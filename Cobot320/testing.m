            clf;
            clc

            % 4.1 and 4.2: Define the DH Parameters to create the Kinematic 
			% model
            link(1) = Link('d', 0.1739, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-165 165]));
            link(2) = Link('d', 0.08878, 'a', 0.135, 'alpha', 0,'offset',pi/2, 'qlim', deg2rad([-165 165]));
            link(3) = Link('d', -0.08878, 'a', 0.120, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(4) = Link('d', 0.08878, 'a', 0, 'alpha', pi/2,'offset',pi/2, 'qlim', deg2rad([-165 165]));
            link(5) = Link('d', 0.095, 'a', 0, 'alpha', pi/2,'offset',pi, 'qlim', deg2rad([-165 165]));
            link(6) = Link('d',0.0655, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-175 175]));

            

			% Generate the model
            robot = SerialLink([link(1), link(2), link(3), link(4), link(5), link(6)],'name','myRobot');         
            
            % Creates a vector of n joint angles at 0.
           q = [0 0 0 0 0 0];

            %(60,-45,35,-60,0)

           b = robot.fkine(q);

            % Set the size of the workspace when drawing the robot
             % Overiding the default workspace for this small robot
            workspace = [-0.5 0.5 -0.5 0.5 -0.01 0.8];   
         
             
            % Plot the robot
            robot.plot(q,'workspace',workspace); 
            robot.teach
        
            % 4.4 Get the current joint angles based on the position in the model
           % q = robot.getpos()  ;
            
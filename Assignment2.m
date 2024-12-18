classdef Assignment2
    properties
        % *********** NOTE! THIS FILE IS DEPRECIATED - EDIT THE GUITEST FILE! *****************
        robot1 = [];% Robot 1 model
        gripperBase1 = []; % Robot 1's gripper base model
        gripperLeft1 = cell(1,2); % stores (gripper claw 1, angle)
        gripperRight1 = cell(1,2); % stores (gripper claw 2, angle)

        robot2 = [];% Robot 2 model
        gripperBase2 = []; % Robot 2's gripper base model
        gripperLeft2 = cell(1,2); % stores (gripper claw 1, angle)
        gripperRight2 = cell(1,2); % stores (gripper claw 2, angle)

        bottles = cell(1,9); % Array to store bottle objects
        bottlePositions = []; % Array to store bottle positions
        holdbottle_ = [false, 1]; % stores whether the gripper is holding a bottle, and the index of the bottle (from array 'bottles')
        
        steps = 14; %should be an even number
        dt = 0.05;  % Discrete time step for simulation
        
        safeBound = [];
        elip1 = [];
        elip2 = [];
        cyl = [];

        % Shelf positions (x1 y1 z1; x2 y2 z2; etc)
        shelf_pos_arr = [0.8 2.15 1.48;
                         0.8 2.15 1.1;
                         0.6 2.15 1.48;
                         0.6 2.15 1.1];

        joy = [];
        joy_status = false;
        joyToggle = false;

        isEmergencyStopTriggered = false;

        

    end
    
    methods
        %% Constructor
        function self = Assignment2()
            Logger().write('\erase');
            try
                self.joy = vrjoystick(1);
                self.joy_status = true;
            catch ERR
                disp(ERR.message);
            end

        end

        %% Initialize Scene
        function self = initialiseScene(self) % Load and place objects in the scene
            

            
            %Plot the robot in it's initial configuration
            clf;
            self.robot1 = UR30(eye(4) * transl(0,1.45,1) * trotx(0) * troty(0));
            hold on;

         
            xlim([-5, 5]);
            ylim([-5, 5]);
            zlim([-5, 5]);
            axis equal;
            self.robot1.model.animate(deg2rad([-230 -60 100 -137 -5 90])); % Robot's initial joint configuration
            self.gripperBase1 = PlaceObject('gripperBase.ply',[0,0,0]);
            self.gripperLeft1{1} = PlaceObject('gripperClaw.ply',[0,0,0]);
            self.gripperLeft1{2} = deg2rad(20);
            self.gripperRight1{1} = PlaceObject('gripperClaw.ply',[0,0,0]);
            self.gripperRight1{2} = deg2rad(20);
            self = self.commandGrippers('update');
            
            
            % Plot objects around the envionment

            % Add wood to floor
            surf([-5,-5;5,5] ...        % x axis
            ,[-4,4;-4,4] ...            % y axis
            ,[0,0;0,0] ...          % z axis
            ,'CData',imread('floor.jpg') ...
            ,'FaceColor','texturemap');
          
        
            % Place the bar at specified location
            PlaceObject('Bar.PLY',[0,0,0]);

            % Place the fire extinguisher at specified location
            PlaceObject('fireExtinguisherElevated.ply',[2.5,2.25,0.465]);

            % Place the estop at specified location
            PlaceObject('emergencyStopButton.ply',[1.7,-1.7,1.1]);
        
            % Place the ingredients at specified locations
            t = hgtransform;
            self.bottles{1} = PlaceObject('Whisky.ply',[0,0,0]);
            set(self.bottles{1}, 'Parent', t);
            set(t, 'Matrix', (transl(self.shelf_pos_arr(1,:))));
            
            t = hgtransform;
            self.bottles{2} = PlaceObject('Vodka.ply',[0,0,0]);
            set(self.bottles{2}, 'Parent', t);
            set(t, 'Matrix', (transl(self.shelf_pos_arr(2,:))));
            
            t = hgtransform;
            self.bottles{3} = PlaceObject('Cognac.ply',[0,0,0]);
            set(self.bottles{3}, 'Parent', t);
            set(t, 'Matrix', (transl(self.shelf_pos_arr(3,:))));

            t = hgtransform;
            self.bottles{4} = PlaceObject('Glass.ply',[0,0,0]);
            set(self.bottles{4}, 'Parent', t);
            set(t, 'Matrix', (transl(self.shelf_pos_arr(4,:))));

           
            drawnow();
            Logger().write('Initialisation finished');
        end
        
        %% Control Loop
        function self = controlLoop(self)
           run = true;
           while run == true
                % Looped control code in here

                self = self.straightDrink1();
                self = self.straightDrink2();
                self = self.straightDrink3();



               run = false;
           end

           
           Logger().write('Program Completed');
        end

        %% Gui command to mix drink variant 1
        function self = mixerOption1(self)
            

        end

        %% Gui command to mix drink variant 2
        function self = mixerOption2(self)
            

        end

        %% Gui command to pour straight drink variant 1
        function self = straightDrink1(self)
            
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(1,1), self.shelf_pos_arr(1,2)-0.3, self.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
            disp('ESTOP pressed3:')
            disp(self.isEmergencyStopTriggered);
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(1,1), self.shelf_pos_arr(1,2)-0.12, self.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
            self.holdbottle_ = [true,1];
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(1,1), self.shelf_pos_arr(1,2)-0.3, self.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
            disp(self.isEmergencyStopTriggered);
            self = self.moveToPoseRMRC(self.getPoseT() * transl(0, 0, -0.5), [1 0 0 0 0 0]);
            self = self.moveToPoseRMRC(self.getPoseT() * transl(0, 0, 0.5), [1 0 0 0 0 0]);
            self.holdbottle_ = [false,1];
            self.SnapBottleTo(1, self.shelf_pos_arr(1,:), [0,0,0]);
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(1,1), self.shelf_pos_arr(1,2)-0.3, self.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
            
        end

        %% Gui command to pour straight drink variant 2
        function self = straightDrink2(self)
            
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(2,1), self.shelf_pos_arr(2,2)-0.3, self.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(2,1), self.shelf_pos_arr(2,2)-0.12, self.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            self.holdbottle_ = [true,2];
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(2,1), self.shelf_pos_arr(2,2)-0.3, self.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            self = self.moveToPoseRMRC(self.getPoseT() * transl(0, 0, -0.5), [1 0 0 0 0 0]);
            self = self.moveToPoseRMRC(self.getPoseT() * transl(0, 0, 0.5), [1 0 0 0 0 0]);
            self.holdbottle_ = [false,2];
            self.SnapBottleTo(2, self.shelf_pos_arr(2,:), [0,0,0]);
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(2,1), self.shelf_pos_arr(2,2)-0.3, self.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
        end

        %% Gui command to pour straight drink variant 3
        function self = straightDrink3(self)
            
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(3,1), self.shelf_pos_arr(3,2)-0.3, self.shelf_pos_arr(3,3)+0.08) * rpy2tr(-pi/2,0,0));
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(3,1), self.shelf_pos_arr(3,2)-0.12, self.shelf_pos_arr(3,3)+0.08) * rpy2tr(-pi/2,0,0));
            self.holdbottle_ = [true,3];
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(3,1), self.shelf_pos_arr(3,2)-0.3, self.shelf_pos_arr(3,3)+0.1) * rpy2tr(-pi/2,0,0));
            self = self.moveToPoseRMRC(self.getPoseT() * transl(0, 0, -0.5), [1 0 0 0 0 0]);
            self = self.moveToPoseRMRC(self.getPoseT() * transl(0, 0, 0.5), [1 0 0 0 0 0]);
            self.holdbottle_ = [false,3];
            self.SnapBottleTo(3, self.shelf_pos_arr(3,:), [0,0,0]);
            self = self.moveToPoseIkcon(transl(self.shelf_pos_arr(3,1), self.shelf_pos_arr(3,2)-0.3, self.shelf_pos_arr(3,3)+0.08) * rpy2tr(-pi/2,0,0));
        end

        

        %% get pose.T
        function output = getPoseT(self)
            output = self.robot1.model.fkine(self.robot1.model.getpos()).T;
            
        end

        %% get Joint State
        function output = getJointState(self, pose)
            output = self.robot1.model.ikcon(pose, self.robot1.model.getpos()); % joint values for target pose
        end


        %% Move Bottle
        function HoldBottle(self, bottleNum)
            % Update the position of a bottle based on the input handle and 
            % end effector pos, i.e. make the relative pose of the bottle 
            % 'sticky' to the pose of the end effector

            t = hgtransform;
            set(self.bottles{bottleNum}, 'Parent', t);
            set(t, 'Matrix', (self.getPoseT())* trotx(pi/2) * transl(0,0.12,-0.1));
        end
        
        %% Teleport Bottle
        function SnapBottleTo(self, bottleNum, trans, rpy)
            % Update the position of a bottle based on the input handle and transform + rotation values

            t = hgtransform;
            set(self.bottles{bottleNum}, 'Parent', t);
            set(t, 'Matrix', (transl(trans) * rpy2tr(rpy)));
        end

        %% Draw Grippers
        function self = commandGrippers(self, command)
            
                       
            switch(command)
                case 'update'
                    t1 = hgtransform;
                    set(self.gripperLeft1{1}, 'Parent', t1);
                    set(t1, 'Matrix', (self.getPoseT() * transl(-0.03, 0, 0.07) * trotx(-pi/2) * trotz(-pi/2) * trotx(pi) * trotz(self.gripperLeft1{2})));
                    
                    t2 = hgtransform;
                    set(self.gripperRight1{1}, 'Parent', t2);
                    set(t2, 'Matrix', (self.getPoseT() * transl(0.03, 0, 0.07) * trotx(-pi/2) * trotz(-pi/2) * trotz(self.gripperRight1{2})));
                    
                    t3 = hgtransform;
                    set(self.gripperBase1, 'Parent', t3);
                    set(t3, 'Matrix', (self.getPoseT() * transl(0, 0, 0) * trotx(-pi/2) * trotz(-pi/2)));
                    
    
                case 'open'
                    Logger().write('open claw');
                    Logger().write(' ');
                    self.steps = self.steps/2;
                    


                    self.steps = self.steps*2;
    
                case 'close'
                    Logger().write('close claw');
                    Logger().write(' ');
                    self.steps = self.steps/2;
                  


                    self.steps = self.steps*2;
            end
        
        end
        
        %% Update graphics
        function self = frameUpdate(self)
           
            self = self.commandGrippers('update');
            if self.holdbottle_(1) == true
                self.HoldBottle(self.holdbottle_(2));
            end
            

            if (self.joy_status == true)
                [axes, buttons, povs] = read(self.joy); %get controller values
                if (buttons(13) == 1) % if ps5 button is pressed, loop manual control code until ps5 button is released then pressed again
                    self.joyToggle = true; % manually control variable on
                    button_release = false;
                    
                    while (self.joyToggle == true) 
                        self = self.commandGrippers('update');
                        joystick_command();
                        
                        if (buttons(13) == 0)
                            button_release = true;
                        end
                        if (button_release == true && buttons(13) == 1)
                            self.joyToggle = false;
                        end
                    end
                    pause(2);
                end
            end

            drawnow();
            pause(self.dt);
            
        end

        %% Free move to given pose with ikcon
        function app = moveToPoseIkcon(app, pose)
            
            q0 = app.robot1.model.getpos(); % Initial joint values
            qf = app.robot1.model.ikcon(pose, q0); % joint values for target pose
           for q = 1:size(qf,2)
                while abs(qf(q)) > 2*pi
                    if qf(q) > 0
                        qf(q) = qf(q) - pi;
                    else
                        qf(q) = qf(q) + pi;
                    end
                end
            end

            qMatrix = jtraj(q0, qf, app.steps);

            Logger().write(['   Target Joint State: ',mat2str(qf)]);
            Logger().write(['   Target Pose: ',mat2str(pose)]);
            
            for frame = 1:app.steps

                if app.isEmergencyStopTriggered == true
                    CurrentPos = app.robot1.model.getpos();
                    app.robot1.model.animate(CurrentPos);
                 
                 % Wait until the emergency stop is released
                while app.isEmergencyStopTriggered
                    pause(0.1); % Brief pause to prevent busy waiting
                end
                
                disp('EMERGENCY STOP RELEASED. Resuming movement.');

                 else
                    app.robot1.model.animate(qMatrix(frame,:));
    
                    app = app.frameUpdate();
                end
            end
            ActPose = app.getPoseT();
            
            if sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                
                Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
                Logger().write(['   Δ = ',num2str(sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)),' Failed to achieve desired pose']);

            else
            
                Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
                Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
                Logger().write(' ');

            end
        end


        %% Free move to given pose with Resolved Motion Rate Control (RMRC) 
        function app = moveToPoseRMRC(app, pose, mask)
            
            q0 = app.robot1.model.getpos();

            T0 = app.getPoseT();     % Starting position
            Tf = pose;                              % Ending position

            x0 = transl(T0);                        % Starting position (x, y, z)
            xf = transl(Tf);                        % Target position (x, y, z)

            rpy0 = tr2rpy(T0);                      % Starting orientation (roll, pitch, yaw)
            rpyf = tr2rpy(Tf);                      % Target orientation (roll, pitch, yaw)

            s = lspb(0, 1, app.steps);             % LSPB interpolation scalar

            lambda = 0.2;
            
            % Initialize arrays for storing trajectory
            pos_trajectory = zeros(3, app.steps);     % Position trajectory (x, y, z)
            rpy_trajectory = zeros(3, app.steps);     % Orientation trajectory (roll, pitch, yaw)
            
            % Interpolate both position and orientation
            for i = 1:app.steps
                
                 if app.isEmergencyStopTriggered == true
                    CurrentPos = app.robot1.model.getpos();
                    app.robot1.model.animate(CurrentPos);
                 
                 % Wait until the emergency stop is released
                while app.isEmergencyStopTriggered
                    pause(0.1); % Brief pause to prevent busy waiting
                end
                
                disp('EMERGENCY STOP RELEASED. Resuming movement.');

                 else
                
                % Linear interpolation for position (3D space)
                pos_trajectory(:, i) = x0*(1-s(i)) + s(i)*xf;
                
                % Linear interpolation for orientation (RPY)
                rpy_trajectory(:, i) = rpy0*(1-s(i)) + s(i)*rpyf;
                 end
            end
            
            qMatrix = nan(app.steps,6);
            qMatrix(1,:) = app.robot1.model.ikine(T0, 'q0', q0, 'mask', mask);                 % Solve for joint angles

            for i = 1:app.steps-1

                 if app.isEmergencyStopTriggered == true
                    CurrentPos = app.robot1.model.getpos();
                    app.robot1.model.animate(CurrentPos);
                 
                 % Wait until the emergency stop is released
                while app.isEmergencyStopTriggered
                    pause(0.1); % Brief pause to prevent busy waiting
                end
                
                disp('EMERGENCY STOP RELEASED. Resuming movement.');

                else

                xdot_pos = (pos_trajectory(:, i+1) - pos_trajectory(:, i)) / app.dt;  % Linear velocity
                xdot_ori = (rpy_trajectory(:, i+1) - rpy_trajectory(:, i)) / app.dt;  % Angular velocity
                xdot = [xdot_pos; xdot_ori];   % Full twist (6x1 vector: [vx; vy; vz; wx; wy; wz])

                J = app.robot1.model.jacob0(qMatrix(i,:));                   % Get the Jacobian at the current state
                Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
                qdot = Jinv_dls*xdot;                                              % Solve velocitities via RMRC
                qMatrix(i+1,:) =  qMatrix(i,:) + app.dt*qdot';             % Update next joint state
                 end
            end

            for frame = 1:app.steps 
                
                if app.isEmergencyStopTriggered == true
                    CurrentPos = app.robot1.model.getpos();
                    app.robot1.model.animate(CurrentPos);
                 
                 % Wait until the emergency stop is released
                while app.isEmergencyStopTriggered
                    pause(0.1); % Brief pause to prevent busy waiting
                end
                
                disp('EMERGENCY STOP RELEASED. Resuming movement.');

                 else
                
                app.robot1.model.animate(qMatrix(frame,:));

                app = app.frameUpdate();
                end

            end
            ActPose = app.getPoseT();
            
            if sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
                Logger().write(['   Δ = ',num2str(sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)),' Failed to achieve desired pose']);
            
            else
            
                Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
                Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
                Logger().write(' ');

            end
        end

        
        %% Move to given Joint State
        function app = moveManual(app, qf)

            q0 = app.robot1.model.getpos(); % Initial joint values          

            qSuperMatrix = zeros(app.steps, size(qf,2));
            for i = 1:size(qf,2)
                if ~isnan(qf(i))
                    qMatrix = jtraj(q0(i), qf(i), app.steps);
                    qSuperMatrix(:,i) = qMatrix;
                else
                    qSuperMatrix(:,i) = q0(i);
                    qf(i) = q0(i);
                end
            end

            pose = app.robot1.model.fkine(qf).T;
            Logger().write(['   Moving to Joint State ',mat2str(qf)]);
            Logger().write(['   Target Pose ',mat2str(pose)]);
            
            for frame = 1:app.steps 
                
                if app.isEmergencyStopTriggered == true
                    CurrentPos = app.robot1.model.getpos();
                    app.robot1.model.animate(CurrentPos);
                 
                 % Wait until the emergency stop is released
                while app.isEmergencyStopTriggered
                    pause(0.1); % Brief pause to prevent busy waiting
                end
                
                disp('EMERGENCY STOP RELEASED. Resuming movement.');

                else

                app.robot1.model.animate(qSuperMatrix(frame,:));

                app = app.frameUpdate();
                end
            end
            ActPose = app.getPoseT();
            
            if sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                Logger().write(['   Δ = ',sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2),' Failed to achieve desired pose']);

            else

            Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
            Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
            Logger().write(' ');

            end
        end

        %% simultaneous movement demo
        % function simulMoveDemo(self)
            % self.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));
            % 
            % endAffectorTr = self.getPoseT() * troty(-pi/2);
            % self.gripperLeft1.model.base = endAffectorTr;
            % self.gripperRight1.model.base = endAffectorTr * troty(pi);
            % self.gripperLeft1.model.animate(deg2rad(190));
            % self.gripperRight1.model.animate(deg2rad(10));
            % 
            % q0 = self.robot1.model.getpos(); % Initial joint values
            % qf = self.robot1.model.ikcon(self.getPoseT * transl(0.2,0.2,0), q0); % joint values for target pose
            % 
            % for q = 1:size(qf,2)
            %     if abs(qf(q)) > 2*pi
            %         if qf(q) > 0
            %             qf(q) = qf(q) - pi;
            %         else
            %             qf(q) = qf(q) + pi;
            %         end
            %     end
            % end
            % 
            % qMatrixBase = jtraj(q0, qf, 100);
            % qMatrixRight  = jtraj(self.gripperLeft1.model.getpos(), deg2rad(170), 100);
            % qMatrixLeft  = jtraj(self.gripperRight1.model.getpos(), deg2rad(-10), 100);
            % 
            % for frame = 1:100 
            %     self.robot1.model.animate(qMatrixBase(frame,:));
            % 
            %     endAffectorTr = self.getPoseT() * troty(-pi/2);
            %     self.gripperLeft1.model.base = endAffectorTr;
            %     self.gripperRight1.model.base = endAffectorTr * troty(pi);
            %     self.gripperLeft1.model.animate(qMatrixLeft(frame,:));
            %     self.gripperRight1.model.animate(qMatrixRight(frame,:));
            % 
            %     self = self.frameUpdate();
            % end
            % 
            % pause(3);
            % self = self.commandGrippers('open');
            % self.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));
            % self = self.commandGrippers('update');
        % end
        %% Workspace Radius and Volume
        function self = plotRadAndVol(self)
            disp('The reach of the stationary UR3e can roughly be calculated as 0.9410m')
            disp('vertically from the base, and 0.7410m horizontally. The reach of the')
            disp('UR30 can be calculated to be XXXm vertically and XXXm horizontally.')
            disp(' ')
            disp('The robots cannot reach below their relative height Z=0, therefore the volumes of their')
            disp('reach is determined as half-ellipsoids centred on the robots base ');
            disp(' ')
            disp('All together, this represents a working volume of:')
            disp('(2π/3)*(0.9410 * 0.7410^2)')
            disp(['    = ',num2str((2*pi/3)*(0.9410 * 0.7410^2)),' cubic metres for the UR3e, and:']);
            disp('(2π/3)*(XXXX * XXXXX^2)')
            disp(['    = ',num2str((2*pi/3)*(0.9410 * 0.7410^2)),' cubic metres for the UR30.']);


            % NOTE: The basis of the following surf drawing code was adapted from code generated by AI 
            % Parameters for the ellipsoid 1
            [a, b, c] = deal(0.7410, 0.7410, 0.9410);  % Radii along the x, y, and z axes

            % Create the ellipsoid data for UR3e
            [xe, ye, ze] = ellipsoid(0, 0, 0.5, a, b, c, 100);
            % Mask to keep only the lower half of the ellipsoid
            zmask = ze >= 0.5;  % True for points at or above z = 0.5
            xmask = xe >= 0;
            mask = zmask & xmask;
            xe(~mask) = NaN;
            ye(~mask) = NaN;
            ze(~mask) = NaN;
            
            % Define a grid for the half-ellipsoid surface
            patchData = surf2patch(xe, ye, ze);
            self.elip1 = patch(patchData);
            set(self.elip1, 'FaceColor', [0, 1, 0]);  % RGB color (e.g., red)
            set(self.elip1, 'FaceAlpha', 0.2);        % Transparency (0 is fully transparent, 1 is fully opaque)
            set(self.elip1, 'EdgeColor', 'none');     % Remove edges for smoother appearance

            % Parameters for the ellipsoid 2
            [a, b, c] = deal(0.7410, 0.7410, 0.9410);  % Radii along the x, y, and z axes

            % Create the ellipsoid data
            [xe, ye, ze] = ellipsoid(-0.8, 0, 0.5, a, b, c, 100);
            % Mask to keep only the lower half of the ellipsoid
            zmask = ze >= 0.5;  % True for points at or above z = 0.5
            xmask = xe <= -0.8;
            mask = zmask & xmask;
            xe(~mask) = NaN;
            ye(~mask) = NaN;
            ze(~mask) = NaN;
            
            % Define a grid for the half-ellipsoid surface
            patchData = surf2patch(xe, ye, ze);
            self.elip2 = patch(patchData);
            set(self.elip2, 'FaceColor', [0, 1, 0]);  % RGB color (e.g., red)
            set(self.elip2, 'FaceAlpha', 0.2);        % Transparency (0 is fully transparent, 1 is fully opaque)
            set(self.elip2, 'EdgeColor', 'none');     % Remove edges for smoother appearance
            
            % Parameters for the middle cylinder
            radius_major = 0.9410; % Major radius (along y-axis)
            radius_minor = 0.7410; % Minor radius (along z-axis)

            [zc, yc, xc] = cylinder(1, 150);  % Minor radius for circular base
            zc = (zc * radius_major) + 0.5;
            yc = yc * radius_minor;
            xc = xc * -0.82;
            mask = zc >= 0.5;  % True for points at or above z = 0.5
            xc(~mask) = NaN;
            yc(~mask) = NaN;
            zc(~mask) = NaN;

            patchData = surf2patch(xc, yc, zc);
            self.cyl = patch(patchData);

            set(self.cyl, 'FaceColor', [0, 1, 0]);  % RGB color (e.g., red)
            set(self.cyl, 'FaceAlpha', 0.2);        % Transparency (0 is fully transparent, 1 is fully opaque)
            set(self.cyl, 'EdgeColor', 'none');     % Remove edges for smoother appearance
            
            self.moveManual([0 0 0 0 0 -pi/2 0 0]);
            self.moveManual([0 0 -pi/2 0 0 -pi/2 0 0]);
            self.moveManual([0 0 pi/2 0 0 -pi/2 0 0]);
            self.moveManual([0 0 pi/2 -pi/2 0 -pi/2 0 0]);
            self.moveManual([0 0 -pi/2 0 0 -pi/2 0 0]);
            self.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));
            self.commandGrippers('update');
            
            self.safeBound = [self.elip1 self.elip2 self.cyl];
            
        end
        
        %% Hide safe Zone
        function self = hideVol(self)
            delete(self.safeBound);
        end

        %% Run
        function self = runAll(self)
            self = self.initialiseScene();
            self = self.plotRadAndVol();
            self = self.hideVol();
            self = self.controlLoop();
        end

%% Setter function
        function self = EmergencyStopTriggeredTrue(self)

            self.isEmergencyStopTriggered = true;
            disp('Emergency stop triggered ');

        end

%% Setter function
        function self = EmergencyStopTriggeredFalse(self)

            self.isEmergencyStopTriggered = false;
            disp('Emergency stop reset ');

        end


    end
end
classdef Assignment2
    properties
        robot1 = [];% Robot model
        gripperBase1 = []; % stores 
        gripperLeft1 = cell(1,2); % stores (object, angle)
        gripperRight1 = cell(1,2); % stores (object, angle)
        bottles = cell(1,9); % Array to store bottle objects
        bottlePositions = []; % Array to store bottle positions
        jtraj_steps = 30; %should be even
        holdbottle_ = [false, 1];
        safeBound = [];
        elip1 = [];
        elip2 = [];
        cyl = [];

        % Shelf positions
        Shelf_pos_1 = [-0.7,2.2,1.5];
        Shelf_pos_2 = [-0.7,2.2,1.1];
        Shelf_pos_3 = [0.6,2.2,1.5];
        Shelf_pos_4 = [0.6,2.2,1.1];
    end
    
    methods
        %% Constructor
        function self = Assignment2()
            Logger().write('\erase');
        end
        
        %% Initialize Scene
        function self = initialiseScene(self) % Load and place objects in the scene
            

            
            %Plot the robot in it's initial configuration
            clf;
            self.robot1 = UR30(eye(4) * transl(0,1.5,1) * trotx(0) * troty(0));
            hold on;

         
            xlim([-5, 5]);
            ylim([-5, 5]);
            zlim([-0.1, 2]);
            axis equal;
            self.robot1.model.animate(deg2rad([0 0 0 -90 0 0]));
            
            self.gripperBase1 = PlaceObject('gripperBase.ply',[0,0,0]);
            self.gripperLeft1{1} = PlaceObject('gripperClaw.ply',[0,0,0]);
            self.gripperLeft1{2} = deg2rad(0);
            self.gripperRight1{1} = PlaceObject('gripperClaw.ply',[0,0,0]);
            self.gripperRight1{2} = deg2rad(0);
            self = self.commandGrippers('update');
            
            
            % Plot objects around the envionment

            % Add wood to floor
            surf([-5,-5;5,5] ...        % x axis
            ,[-4,4;-4,4] ...            % y axis
            ,[0,0;0,0] ...          % z axis
            ,'CData',imread('floor.jpg') ...
            ,'FaceColor','texturemap');
          
        
            % Place the bar at specified location
            PlaceObject('Bar.ply',[0,0,0]);
        
            % Place the ingredients at specified locations
            self.bottles{1} = PlaceObject('Whisky.ply',self.Shelf_pos_1);
            self.bottles{2} = PlaceObject('Vodka.ply',self.Shelf_pos_2);
            self.bottles{3} = PlaceObject('Cognac.ply',self.Shelf_pos_3);
            self.bottles{4} = PlaceObject('Glass.ply',self.Shelf_pos_4);

            % % Plot the table
            % table = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0, 0, 0]);
            % t = hgtransform;
            % set(table, 'Parent', t);
            % set(t, 'Matrix', (transl(-0.3, 0, 0)));
            % 
            % % Place emergency stop button
            % estop = PlaceObject('emergencyStopButton.ply', [0, 0, 0]);
            % verts = [get(estop, 'Vertices'), ones(size(get(estop, 'Vertices'), 1), 1)]; % Rotation
            % verts = verts * 0.2; % Scaling factor
            % set(estop, 'Vertices', verts(:, 1:3));
            % t = hgtransform;
            % set(estop, 'Parent', t);
            % set(t, 'Matrix', (transl(0.65, -0.6, 0.5) * trotz(pi/2)));
            % 
            % % Place emergency stop button
            % estop2 = PlaceObject('emergencyStopButton.ply', [0, 0, 0]);
            % verts = [get(estop2, 'Vertices'), ones(size(get(estop2, 'Vertices'), 1), 1)]; % Rotation
            % verts = verts * 0.2; % Scaling factor
            % set(estop2, 'Vertices', verts(:, 1:3));
            % t = hgtransform;
            % set(estop2, 'Parent', t);
            % set(t, 'Matrix', (transl(1.1, -1.1, 0.55) * trotx(pi/2)));
            % 
            % % Place fence
            % fence = PlaceObject('fence.ply', [0, 0, 0]);
            % verts = [get(fence, 'Vertices'), ones(size(get(fence, 'Vertices'), 1), 1)]; % Rotation
            % verts(:,1) = verts(:,1) * 0.4; % Scaling factor
            % verts(:,2) = verts(:,2) * 0.6; % Scaling factor
            % verts(:,3) = verts(:,3) * 0.3; % Scaling factor
            % set(fence, 'Vertices', verts(:, 1:3));
            % t = hgtransform;
            % set(fence, 'Parent', t);
            % set(t, 'Matrix', (transl(-0.1, 0.55, -0.3)));
            % 
            % % Place fire extinguisher
            % fireEx = PlaceObject('fireExtinguisher.ply', [0, 0, 0]);
            % t = hgtransform;
            % set(fireEx, 'Parent', t);
            % set(t, 'Matrix', (transl(0.5, -1.25, 0)));
            % 
            % % Place the bricks
            % lastPosition = [0.3, -0.4, 0.5];
            % for i = 1:9
            % 
            %     if mod(i,3) == 1 && i ~= 1
            %         self.brickPositions{i} = lastPosition + [0.1, -0.4, 0];  % Move bricks along -x axis
            %         lastPosition = self.brickPositions{i};
            %     else
            %         self.brickPositions{i} = lastPosition + [0, 0.2, 0];  % Move bricks along -x axis
            %         lastPosition = self.brickPositions{i};
            %     end
            %     self.bricks{i} = PlaceObject('HalfSizedRedGreenBrick.ply', [0,0,0]);
            %     t = hgtransform;
            %     set(self.bricks{i}, 'Parent', t);
            %     set(t, 'Matrix', (transl(self.brickPositions{i})));
            % end
            

            drawnow();
            Logger().write('Initialisation finished');
        end
        
        %% Control Loop
        function self = controlLoop(self)
           while run == true

           end

            Logger().write('Program Completed');
            pause;
        end

        %% get pose
        function output = getpose(self)
            output = self.robot1.model.fkine(self.robot1.model.getpos());
        end

        %% get pose.T
        function output = getposeT(self)
            output = self.robot1.model.fkine(self.robot1.model.getpos()).T;
        end

        %% get Joint State
        function output = getJointState(self, pose)
            output = self.robot1.model.ikcon(pose, self.robot1.model.getpos()); % joint values for target pose
        end


        %% Move Bottle
        function moveBottle(self, bottleNum)
            % Update the position of a bottle based on the input handle and end effector pos

            t = hgtransform;
            set(self.bottles{bottleNum}, 'Parent', t);
            set(t, 'Matrix', (self.getposeT()) * transl(0,0,0.14) * trotz(pi/2));
        end

        function moveBottleTo(self, bottleNum, trans, rpy)
            % Update the position of a bottle based on the input handle and transform + rotation values

            t = hgtransform;
            set(self.bottles{bottleNum}, 'Parent', t);
            set(t, 'Matrix', (self.getposeT()) * transl(trans) * rpy2tr(rpy));
        end

        %% Draw Grippers
        function self = commandGrippers(self, command)
            
                       
            switch(command)
                case 'update'
                    t = hgtransform;
                    set(self.gripperLeft1{1}, 'Parent', t);
                    set(t, 'Matrix', (self.getposeT() * transl(0, 0, -0.3) * trotx(pi/2)));

                    set(self.gripperRight1{1}, 'Parent', t);
                    set(t, 'Matrix', (self.getposeT() * transl(0, 0, -0.3) * trotx(pi/2)));

                    set(self.gripperBase1, 'Parent', t);
                    set(t, 'Matrix', (self.getposeT() * transl(0, 0, -0.3) * trotx(pi/2)));
                    
    
                case 'open'
                    Logger().write('open claw');
                    Logger().write(' ');
                    self.jtraj_steps = self.jtraj_steps/2;
                    qMatrix1  = jtraj(self.gripperLeft1.model.getpos(), deg2rad(10), self.jtraj_steps);
                    qMatrix2  = jtraj(self.gripperRight1.model.getpos(), deg2rad(190), self.jtraj_steps);
                    
                    for frame = 1:self.jtraj_steps
                        self.gripperLeft1.model.animate(qMatrix1(frame,:));
                        self.gripperRight1.model.animate(qMatrix2(frame,:));
                        
                        if (mod(frame,10))
                            drawnow();
                        end
                    end
                    self.jtraj_steps = self.jtraj_steps*2;
    
                case 'close'
                    Logger().write('close claw');
                    Logger().write(' ');
                    self.jtraj_steps = self.jtraj_steps/2;
                    qMatrix1  = jtraj(self.gripperLeft1.model.getpos(), deg2rad(3), self.jtraj_steps);
                    qMatrix2  = jtraj(self.gripperRight1.model.getpos(), deg2rad(183), self.jtraj_steps);
                
                    for frame = 1:self.jtraj_steps
                        self.gripperLeft1.model.animate(qMatrix1(frame,:));
                        self.gripperRight1.model.animate(qMatrix2(frame,:));
                        
                        if (mod(frame,10))
                            drawnow();
                        end
                    end
                    self.jtraj_steps = self.jtraj_steps*2;
            end
        
        end
        
        %% Update graphics
        function self = frameUpdate(self)
            self = self.commandGrippers('update');
            if self.holdbottle_(1) == true
                self.moveBottle(self.holdbottle_(2));
            end
            drawnow();
        end

        %% Free move to given pose with ikcon
        function self = moveToPose(self, pose)
            
            q0 = self.robot1.model.getpos(); % Initial joint values
            qf = self.robot1.model.ikcon(pose, q0); % joint values for target pose
            for q = 1:size(qf,2)
                if abs(qf(q)) > pi
                    if qf(q) > 0
                        qf(q) = qf(q) - pi;
                    else
                        qf(q) = qf(q) + pi;
                    end
                end
            end

            qMatrix = jtraj(q0, qf, self.jtraj_steps);

            Logger().write(['   Target Joint State: ',mat2str(qf)]);
            Logger().write(['   Target Pose: ',mat2str(pose)]);
            
            for frame = 1:self.jtraj_steps 
                self.robot1.model.animate(qMatrix(frame,:));
                if mod(frame,10)
                    self = self.frameUpdate();
                end
            end
            ActPose = self.getposeT();
            
            while sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                Logger().write(['   Δ = ',num2str(sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)),' trying again...']);
                q0 = self.robot1.model.getpos(); % Initial joint values
                qf = self.robot1.model.ikcon(pose * transl(0,0-0.01), q0); % joint values for target pose
                qMatrix = jtraj(q0, qf, self.jtraj_steps);

                for frame2 = 1:self.jtraj_steps 
                    self.robot1.model.animate(qMatrix(frame2,:));
                    if mod(frame2,10)
                        self = self.frameUpdate();
                    end
                end
                ActPose = self.getposeT();
            end
            
            Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
            Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
            Logger().write(' ');
        end
        
        %% Move to given Joint State
        function self = moveManual(self, qf)

            q0 = self.robot1.model.getpos(); % Initial joint values          

            qSuperMatrix = zeros(self.jtraj_steps, size(qf,2));
            for i = 1:size(qf,2)
                if ~isnan(qf(i))
                    qMatrix = jtraj(q0(i), qf(i), self.jtraj_steps);
                    qSuperMatrix(:,i) = qMatrix;
                else
                    qSuperMatrix(:,i) = q0(i);
                    qf(i) = q0(i);
                end
            end

            pose = self.robot1.model.fkine(qf).T;
            Logger().write(['   Moving to Joint State ',mat2str(qf)]);
            Logger().write(['   Target Pose ',mat2str(pose)]);
            
            for frame = 1:self.jtraj_steps 
                self.robot1.model.animate(qSuperMatrix(frame,:));
                if mod(frame,10)
                    self = self.frameUpdate();
                end
            end
            ActPose = self.getposeT();
            
            while sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                Logger().write(['   Δ = ',sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2),' trying again...']);
                q0 = self.robot1.model.getpos(); % Initial joint values
                qf = self.robot1.model.ikcon(pose * transl(0,0-0.01), q0); % joint values for target pose
                qMatrix = jtraj(q0, qf, self.jtraj_steps);

                for frame2 = 1:self.jtraj_steps 
                    self.robot1.model.animate(qMatrix(frame2,:));
                    if mod(frame2,10)
                        self = self.frameUpdate();
                    end
                end
                ActPose = self.getposeT();
            end

            Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
            Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
            Logger().write(' ');
        end

        %% simultaneous movement demo
        function simulMoveDemo(self)
            self.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));

            endAffectorTr = self.getposeT() * troty(-pi/2);
            self.gripperLeft1.model.base = endAffectorTr;
            self.gripperRight1.model.base = endAffectorTr * troty(pi);
            self.gripperLeft1.model.animate(deg2rad(190));
            self.gripperRight1.model.animate(deg2rad(10));

            q0 = self.robot1.model.getpos(); % Initial joint values
            qf = self.robot1.model.ikcon(self.getposeT * transl(0.2,0.2,0), q0); % joint values for target pose
            
            for q = 1:size(qf,2)
                if abs(qf(q)) > pi
                    if qf(q) > 0
                        qf(q) = qf(q) - pi;
                    else
                        qf(q) = qf(q) + pi;
                    end
                end
            end

            qMatrixBase = jtraj(q0, qf, 100);
            qMatrixRight  = jtraj(self.gripperLeft1.model.getpos(), deg2rad(170), 100);
            qMatrixLeft  = jtraj(self.gripperRight1.model.getpos(), deg2rad(-10), 100);
            
            for frame = 1:100 
                self.robot1.model.animate(qMatrixBase(frame,:));

                endAffectorTr = self.getposeT() * troty(-pi/2);
                self.gripperLeft1.model.base = endAffectorTr;
                self.gripperRight1.model.base = endAffectorTr * troty(pi);
                self.gripperLeft1.model.animate(qMatrixLeft(frame,:));
                self.gripperRight1.model.animate(qMatrixRight(frame,:));

                self = self.frameUpdate();
            end

            pause(3);
            self = self.commandGrippers('open');
            self.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));
            self = self.commandGrippers('update');
        end
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

    end
end
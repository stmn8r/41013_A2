classdef Assignment2
    properties
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
        
        jtraj_steps = 14; %should be an even number
        
        safeBound = [];
        elip1 = [];
        elip2 = [];
        cyl = [];

        % Shelf positions (x1 y1 z1; x2 y2 z2; etc)
        shelf_pos_arr = [0.8 2.15 1.48;
                         0.8 2.15 1.1;
                         0.6 2.15 1.48;
                         0.6 2.15 1.1];
    end
    
    methods
        %% Constructor
        function app = Assignment2()
            Logger().write('\erase');
        end
        
        %% Initialize Scene
        function app = initialiseScene(app) % Load and place objects in the scene
            

            
            %Plot the robot in it's initial configuration
            clf;
            app.robot1 = UR30(eye(4) * transl(0,1.45,1) * trotx(0) * troty(0));
            hold on;

         
            xlim([-5, 5]);
            ylim([-5, 5]);
            zlim([-5, 5]);
            axis equal;
            app.robot1.model.animate(deg2rad([-230 -60 100 -137 -5 90])); % Robot's initial joint configuration
            app.gripperBase1 = PlaceObject('gripperBase.ply',[0,0,0]);
            app.gripperLeft1{1} = PlaceObject('gripperClaw.ply',[0,0,0]);
            app.gripperLeft1{2} = deg2rad(20);
            app.gripperRight1{1} = PlaceObject('gripperClaw.ply',[0,0,0]);
            app.gripperRight1{2} = deg2rad(20);
            app = app.commandGrippers('update');
            
            
            % Plot objects around the envionment

            % Add wood to floor
            surf([-5,-5;5,5] ...        % x axis
            ,[-4,4;-4,4] ...            % y axis
            ,[0,0;0,0] ...          % z axis
            ,'CData',imread('floor.jpg') ...
            ,'FaceColor','texturemap');
          
        
            % Place the bar at specified location
            PlaceObject('Bar.PLY',[0,0,0]);
        
            % Place the ingredients at specified locations
            t = hgtransform;
            app.bottles{1} = PlaceObject('Whisky.ply',[0,0,0]);
            set(app.bottles{1}, 'Parent', t);
            set(t, 'Matrix', (transl(app.shelf_pos_arr(1,:))));
            
            t = hgtransform;
            app.bottles{2} = PlaceObject('Vodka.ply',[0,0,0]);
            set(app.bottles{2}, 'Parent', t);
            set(t, 'Matrix', (transl(app.shelf_pos_arr(2,:))));
            
            t = hgtransform;
            app.bottles{3} = PlaceObject('Cognac.ply',[0,0,0]);
            set(app.bottles{3}, 'Parent', t);
            set(t, 'Matrix', (transl(app.shelf_pos_arr(3,:))));

            t = hgtransform;
            app.bottles{4} = PlaceObject('Glass.ply',[0,0,0]);
            set(app.bottles{4}, 'Parent', t);
            set(t, 'Matrix', (transl(app.shelf_pos_arr(4,:))));

            drawnow();
            Logger().write('Initialisation finished');
            pause;
        end
        
        %% Control Loop
        function app = controlLoop(app)
           run = true;
           while run == true
                % Looped control code in here

                app = app.straightDrink1();
                app = app.straightDrink2();
                app = app.straightDrink3();



               run = false;
           end

           
           Logger().write('Program Completed');
        end

        %% Gui command to mix drink variant 1
        function app = mixerOption1(app)
            

        end

        %% Gui command to mix drink variant 2
        function app = mixerOption2(app)
            

        end

        %% Gui command to pour straight drink variant 1
        function app = straightDrink1(app)
            
            app = app.moveToPose(transl(app.shelf_pos_arr(1,1), app.shelf_pos_arr(1,2)-0.3, app.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
            app = app.moveToPose(transl(app.shelf_pos_arr(1,1), app.shelf_pos_arr(1,2)-0.12, app.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
            app.holdbottle_ = [true,1];
            app = app.moveToPose(transl(app.shelf_pos_arr(2,1), app.shelf_pos_arr(2,2)-0.3, app.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            app = app.moveToPose(app.getPoseT() * transl(0, 0, -1));
            app = app.moveToPose(app.getPoseT() * transl(0, 0, 1));
            app.holdbottle_ = [false,1];
            app.SnapBottleTo(1, app.shelf_pos_arr(1,:), [0,0,0]);
            app = app.moveToPose(transl(app.shelf_pos_arr(1,1), app.shelf_pos_arr(1,2)-0.3, app.shelf_pos_arr(1,3)+0.1) * rpy2tr(-pi/2,0,0));
        end

        %% Gui command to pour straight drink variant 2
        function app = straightDrink2(app)
            
            app = app.moveToPose(transl(app.shelf_pos_arr(2,1), app.shelf_pos_arr(2,2)-0.3, app.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            app = app.moveToPose(transl(app.shelf_pos_arr(2,1), app.shelf_pos_arr(2,2)-0.12, app.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            app.holdbottle_ = [true,2];
            app = app.moveToPose(transl(app.shelf_pos_arr(2,1), app.shelf_pos_arr(2,2)-0.3, app.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            app = app.moveToPose(app.getPoseT() * transl(0, 0, -1));
            app = app.moveToPose(app.getPoseT() * transl(0, 0, 1));
            app.holdbottle_ = [false,2];
            app.SnapBottleTo(2, app.shelf_pos_arr(2,:), [0,0,0]);
            app = app.moveToPose(transl(app.shelf_pos_arr(2,1), app.shelf_pos_arr(2,2)-0.3, app.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
        end

        %% Gui command to pour straight drink variant 3
        function app = straightDrink3(app)
            
            app = app.moveToPose(transl(app.shelf_pos_arr(3,1), app.shelf_pos_arr(3,2)-0.3, app.shelf_pos_arr(3,3)+0.08) * rpy2tr(-pi/2,0,0));
            app = app.moveToPose(transl(app.shelf_pos_arr(3,1), app.shelf_pos_arr(3,2)-0.12, app.shelf_pos_arr(3,3)+0.08) * rpy2tr(-pi/2,0,0));
            app.holdbottle_ = [true,3];
            app = app.moveToPose(transl(app.shelf_pos_arr(2,1), app.shelf_pos_arr(2,2)-0.3, app.shelf_pos_arr(2,3)+0.1) * rpy2tr(-pi/2,0,0));
            app = app.moveToPose(app.getPoseT() * transl(0, 0, -1));
            app = app.moveToPose(app.getPoseT() * transl(0, 0, 1));
            app.holdbottle_ = [false,3];
            app.SnapBottleTo(3, app.shelf_pos_arr(3,:), [0,0,0]);
            app = app.moveToPose(transl(app.shelf_pos_arr(3,1), app.shelf_pos_arr(3,2)-0.3, app.shelf_pos_arr(3,3)+0.08) * rpy2tr(-pi/2,0,0));
        end

        

        %% get pose.T
        function output = getPoseT(app)
            output = app.robot1.model.fkine(app.robot1.model.getpos()).T;
        end

        %% get Joint State
        function output = getJointState(app, pose)
            output = app.robot1.model.ikcon(pose, app.robot1.model.getpos()); % joint values for target pose
        end


        %% Move Bottle
        function HoldBottle(app, bottleNum)
            % Update the position of a bottle based on the input handle and 
            % end effector pos, i.e. make the relative pose of the bottle 
            % 'sticky' to the pose of the end effector

            t = hgtransform;
            set(app.bottles{bottleNum}, 'Parent', t);
            set(t, 'Matrix', (app.getPoseT())* trotx(pi/2) * transl(0,0.12,-0.1));
        end
        
        %% Teleport Bottle
        function SnapBottleTo(app, bottleNum, trans, rpy)
            % Update the position of a bottle based on the input handle and transform + rotation values

            t = hgtransform;
            set(app.bottles{bottleNum}, 'Parent', t);
            set(t, 'Matrix', (transl(trans) * rpy2tr(rpy)));
        end

        %% Draw Grippers
        function app = commandGrippers(app, command)
            
                       
            switch(command)
                case 'update'
                    t1 = hgtransform;
                    set(app.gripperLeft1{1}, 'Parent', t1);
                    set(t1, 'Matrix', (app.getPoseT() * transl(-0.03, 0, 0.07) * trotx(-pi/2) * trotz(-pi/2) * trotx(pi) * trotz(app.gripperLeft1{2})));
                    
                    t2 = hgtransform;
                    set(app.gripperRight1{1}, 'Parent', t2);
                    set(t2, 'Matrix', (app.getPoseT() * transl(0.03, 0, 0.07) * trotx(-pi/2) * trotz(-pi/2) * trotz(app.gripperRight1{2})));
                    
                    t3 = hgtransform;
                    set(app.gripperBase1, 'Parent', t3);
                    set(t3, 'Matrix', (app.getPoseT() * transl(0, 0, 0) * trotx(-pi/2) * trotz(-pi/2)));
                    
    
                case 'open'
                    Logger().write('open claw');
                    Logger().write(' ');
                    app.jtraj_steps = app.jtraj_steps/2;
                    qMatrix1  = jtraj(app.gripperLeft1.model.getpos(), deg2rad(10), app.jtraj_steps);
                    qMatrix2  = jtraj(app.gripperRight1.model.getpos(), deg2rad(190), app.jtraj_steps);
                    
                    for frame = 1:app.jtraj_steps
                        app.gripperLeft1.model.animate(qMatrix1(frame,:));
                        app.gripperRight1.model.animate(qMatrix2(frame,:));
                        
                        if (mod(frame,1))
                            drawnow();
                        end
                    end
                    app.jtraj_steps = app.jtraj_steps*2;
    
                case 'close'
                    Logger().write('close claw');
                    Logger().write(' ');
                    app.jtraj_steps = app.jtraj_steps/2;
                    qMatrix1  = jtraj(app.gripperLeft1.model.getpos(), deg2rad(3), app.jtraj_steps);
                    qMatrix2  = jtraj(app.gripperRight1.model.getpos(), deg2rad(183), app.jtraj_steps);
                
                    for frame = 1:app.jtraj_steps
                        app.gripperLeft1.model.animate(qMatrix1(frame,:));
                        app.gripperRight1.model.animate(qMatrix2(frame,:));
                        
                        if (mod(frame,1))
                            drawnow();
                        end
                    end
                    app.jtraj_steps = app.jtraj_steps*2;
            end
        
        end
        
        %% Update graphics
        function app = frameUpdate(app)
            app = app.commandGrippers('update');
            if app.holdbottle_(1) == true
                app.HoldBottle(app.holdbottle_(2));
            end
            drawnow();
        end

        %% Free move to given pose with ikcon
        function app = moveToPose(app, pose)
            
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

            qMatrix = jtraj(q0, qf, app.jtraj_steps);

            Logger().write(['   Target Joint State: ',mat2str(qf)]);
            Logger().write(['   Target Pose: ',mat2str(pose)]);
            
            for frame = 1:app.jtraj_steps 
                app.robot1.model.animate(qMatrix(frame,:));
                if mod(frame,10)
                    app = app.frameUpdate();
                end
            end
            ActPose = app.getPoseT();
            
            while sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
                Logger().write(['   Δ = ',num2str(sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)),' trying again...']);
                q0 = app.robot1.model.getpos(); % Initial joint values
                qf = app.robot1.model.ikcon(pose * transl(0,0,-0.01), q0); % joint values for target pose
                qMatrix = jtraj(q0, qf, app.jtraj_steps);

                for frame2 = 1:app.jtraj_steps 
                    app.robot1.model.animate(qMatrix(frame2,:));
                    if mod(frame2,10)
                        app = app.frameUpdate();
                    end
                end
                ActPose = app.getPoseT();
            end
            
            Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
            Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
            Logger().write(' ');
        end
        
        %% Move to given Joint State
        function app = moveManual(app, qf)

            q0 = app.robot1.model.getpos(); % Initial joint values          

            qSuperMatrix = zeros(app.jtraj_steps, size(qf,2));
            for i = 1:size(qf,2)
                if ~isnan(qf(i))
                    qMatrix = jtraj(q0(i), qf(i), app.jtraj_steps);
                    qSuperMatrix(:,i) = qMatrix;
                else
                    qSuperMatrix(:,i) = q0(i);
                    qf(i) = q0(i);
                end
            end

            pose = app.robot1.model.fkine(qf).T;
            Logger().write(['   Moving to Joint State ',mat2str(qf)]);
            Logger().write(['   Target Pose ',mat2str(pose)]);
            
            for frame = 1:app.jtraj_steps 
                app.robot1.model.animate(qSuperMatrix(frame,:));
                if mod(frame,10)
                    app = app.frameUpdate();
                end
            end
            ActPose = app.getPoseT();
            
            while sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2) > 0.05
                Logger().write(['   Δ = ',sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2),' trying again...']);
                q0 = app.robot1.model.getpos(); % Initial joint values
                qf = app.robot1.model.ikcon(pose * transl(0,0,-0.01), q0); % joint values for target pose
                qMatrix = jtraj(q0, qf, app.jtraj_steps);

                for frame2 = 1:app.jtraj_steps 
                    app.robot1.model.animate(qMatrix(frame2,:));
                    if mod(frame2,10)
                        app = app.frameUpdate();
                    end
                end
                ActPose = app.getPoseT();
            end

            Logger().write(['   Actual Pose: ',mat2str(ActPose)]);
            Logger().write(['   Δ = ',mat2str((sqrt((ActPose(1,4)-pose(1,4))^2+(ActPose(2,4)-pose(2,4))^2+(ActPose(3,4)-pose(3,4))^2)))]);
            Logger().write(' ');
        end

        %% simultaneous movement demo
        function simulMoveDemo(app)
            app.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));

            endAffectorTr = app.getPoseT() * troty(-pi/2);
            app.gripperLeft1.model.base = endAffectorTr;
            app.gripperRight1.model.base = endAffectorTr * troty(pi);
            app.gripperLeft1.model.animate(deg2rad(190));
            app.gripperRight1.model.animate(deg2rad(10));

            q0 = app.robot1.model.getpos(); % Initial joint values
            qf = app.robot1.model.ikcon(app.getPoseT * transl(0.2,0.2,0), q0); % joint values for target pose
            
            for q = 1:size(qf,2)
                if abs(qf(q)) > 2*pi
                    if qf(q) > 0
                        qf(q) = qf(q) - pi;
                    else
                        qf(q) = qf(q) + pi;
                    end
                end
            end

            qMatrixBase = jtraj(q0, qf, 100);
            qMatrixRight  = jtraj(app.gripperLeft1.model.getpos(), deg2rad(170), 100);
            qMatrixLeft  = jtraj(app.gripperRight1.model.getpos(), deg2rad(-10), 100);
            
            for frame = 1:100 
                app.robot1.model.animate(qMatrixBase(frame,:));

                endAffectorTr = app.getPoseT() * troty(-pi/2);
                app.gripperLeft1.model.base = endAffectorTr;
                app.gripperRight1.model.base = endAffectorTr * troty(pi);
                app.gripperLeft1.model.animate(qMatrixLeft(frame,:));
                app.gripperRight1.model.animate(qMatrixRight(frame,:));

                app = app.frameUpdate();
            end

            pause(3);
            app = app.commandGrippers('open');
            app.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));
            app = app.commandGrippers('update');
        end
        %% Workspace Radius and Volume
        function app = plotRadAndVol(app)
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
            app.elip1 = patch(patchData);
            set(app.elip1, 'FaceColor', [0, 1, 0]);  % RGB color (e.g., red)
            set(app.elip1, 'FaceAlpha', 0.2);        % Transparency (0 is fully transparent, 1 is fully opaque)
            set(app.elip1, 'EdgeColor', 'none');     % Remove edges for smoother appearance

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
            app.elip2 = patch(patchData);
            set(app.elip2, 'FaceColor', [0, 1, 0]);  % RGB color (e.g., red)
            set(app.elip2, 'FaceAlpha', 0.2);        % Transparency (0 is fully transparent, 1 is fully opaque)
            set(app.elip2, 'EdgeColor', 'none');     % Remove edges for smoother appearance
            
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
            app.cyl = patch(patchData);

            set(app.cyl, 'FaceColor', [0, 1, 0]);  % RGB color (e.g., red)
            set(app.cyl, 'FaceAlpha', 0.2);        % Transparency (0 is fully transparent, 1 is fully opaque)
            set(app.cyl, 'EdgeColor', 'none');     % Remove edges for smoother appearance
            
            app.moveManual([0 0 0 0 0 -pi/2 0 0]);
            app.moveManual([0 0 -pi/2 0 0 -pi/2 0 0]);
            app.moveManual([0 0 pi/2 0 0 -pi/2 0 0]);
            app.moveManual([0 0 pi/2 -pi/2 0 -pi/2 0 0]);
            app.moveManual([0 0 -pi/2 0 0 -pi/2 0 0]);
            app.robot1.model.animate(deg2rad([-0.1 0 90 -90 -90 -90 90 -90]));
            app.commandGrippers('update');
            
            app.safeBound = [app.elip1 app.elip2 app.cyl];
            
        end
        
        %% Hide safe Zone
        function app = hideVol(app)
            delete(app.safeBound);
        end

        %% Run
        function app = runAll(app)
            app = app.initialiseScene();
            app = app.plotRadAndVol();
            app = app.hideVol();
            app = app.controlLoop();
        end

    end
end
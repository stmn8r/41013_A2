%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information
[axes, buttons, povs] = read(joy);

%% Start "real-time" simulation
q = app.robot1.model.getpos();  % Set initial robot configuration 'q'

dt = 0.1;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
%while(buttons(13) == 1)

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % -------------------------------------------------------------
    % 1 - turn joystick input into an end-effector force measurement
    Kv = 0.3;
    Kw = 0.8;
    if (abs(axes(1)) > 0.05)
        vx = Kv*axes(1);
    else
        vx = 0;
    end
    if (abs(axes(2)) > 0.05)
        vy = Kv*-axes(2);
    else 
        vy = 0;
    end
    vz = Kv*(buttons(6)-buttons(5));
    
    if (abs(axes(3)) > 0.05)
        wx = Kw*axes(6);
    else
        wx = 0;
    end
    if (abs(axes(6)) > 0.05)
        wy = Kw*axes(3);
    else
        wy = 0;
    end
    wz = Kw*(axes(4)+1)-Kw*(axes(5)+1);

    dx = [vx;vy;vz;wx;wy;wz];
    % 2 - use simple admittance scheme to convert force measurement into
    % velocity command
    % 2 - use J inverse to calculate joint velocity
    J = app.robot1.model.jacob0(q);
    %dq = pinv(J)*dx;
    
    
    lambda = 0.1;

    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
    dq  =Jinv_dls*dx;
    

    % 3 - apply joint velocity to step robot joint angles 
    q = q + dq'*dt;

    % -------------------------------------------------------------
    
    % Update plot
    app.robot1.model.animate(q);
    pause(0.1);
%end
      

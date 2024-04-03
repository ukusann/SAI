%% IMPORTANT:
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2023
%   2023/10/31
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of a communication class object with the simulator
% Try to connect to simulator
%Output:
% sim - pointer to class simulator_interface
% error_sim = 1 - impossible to connect to simulator
[sim,error_sim] = simulator_interface();
if(error_sim==1)
    return;
end

% Creation of a communication class object with the robot
%Input:
% sim - pointer to class kuka_interface
%Output:
% vehicle - pointer to class kuka_interface
[vehicle,error_kuka] = kuka_interface(sim);
if(error_kuka==1)
    return;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This function returns relevant information about the mobile platform
[error,rob_W,rob_L,theta_obs] = vehicle.get_RobotCharacteristics();
%Output:
% error = 1 - error in function
% rob_W - robot width (cm)
% rob_L - robot lenght (cm)
% theta_obs - Vector with angle value for each sector of obstacle dynamic
% i (i = 1, . . . , 29) relative to the frontal direction (in rad)
if(error==1)
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize the mobile platform with zero speeds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vrobot_y = 0.0;     %cm/s
vrobot_x = 0.0;     %cm/s
wrobot = 0.0;       %rad/s
error = 0;
vel_front_left = 0.0;   %rad/s
vel_front_right = 0.0;  %rad/s
vel_rear_left = 0.0;    %rad/s
vel_rear_right = 0.0;   %rad/s

% Convert longitudinal speed, lateral speed and angular speed into wheel
% speed
[error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = vehicle.Kinematics_vehicle(wrobot, vrobot_y,vrobot_x);
%Input:
% wrobot - angular speed
% vrobot_x - longitudinal speed (cm/s)
% vrobot_y - lateral speed (cm/s)
%Output:
% error = 1 - error in function
% vel_front_left - front left wheel rotation speed (rad/s)
% vel_front_right - front right wheel rotation speed (rad/s)
% vel_rear_left - rear left wheel rotation speed (rad/s)
% vel_rear_right - rear right wheel rotation speed (rad/s)
if(error==1)
    return;
end

% Set wheels speeds
[error, ~, ~, phi] = vehicle.set_velocity(vel_front_left,vel_front_right,vel_rear_left,vel_rear_right);
%Input:
% vel_front_left - rad/s
% vel_front_right - rad/s
% vel_rear_left - rad/s
% vel_rear_right - rad/s
%Output:
% robot  (xrobot,yrobot) in cm
% phirobot - rad
% error = 1 - error in function
if(error==1)
    return;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Show robot bounding box in simulator
error = vehicle.show_robot_bounding_box(true);
if(error==1)
    return;
end

% Get time step value (normally dt=50ms)
[error,timestep] = sim.get_simulation_timestep();
%Output:
% timestep - information of time step (dt)
% error = 1 - error in function
if(error==1)
    return;
end

itarget=2; %initialize first target
start = tic;

%*==================================================
%*=================Parameters=======================
vrobot_des  = 100;
lambdaTarget = 2.3;
lambda_v = -12.7;
stop_time = 4;
vinit = 50;
min_d_limit = 2;
max_d_limit = 300;

%** Useful for Plots **
% x = -pi:pi/10:pi;
% x2 = 0:pi/10:2*pi;

% B1 = 190; % magnitude max de força de repulSão
% B2 = 20; % taxa de decaimento com o aumento da dist
B1 = 10; % magnitude max de força de repulSão
B2 = 30; % taxa de decaimento com o aumento da dist
Q = 0.001;

obsSensorNumber = 29;
    
lambda_obs  = zeros(obsSensorNumber, 1);
sigma       = zeros(obsSensorNumber, 1);
fobs        = zeros(obsSensorNumber, 1);
psi_obs     = zeros(obsSensorNumber, 1);

Fobs = 0;
f_stock = sqrt(Q)*rand(1,obsSensorNumber);


%*==================================================
%%%---------------------- Start Robot Motion Behavior -------------------
while itarget<=sim.TARGET_Number % until robot goes to last target (TARGET_Number)
    %% Robot interface
    % set and get information to/from CoppeliaSim
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();

    % Convert longitudinal speed, lateral speed and angular speed into wheel
    % speed
    [error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = vehicle.Kinematics_vehicle(wrobot, vrobot_y,vrobot_x);
    %Input:
    % wrobot - angular speed
    % vrobot_x - longitudinal speed (cm/s)
    % vrobot_y - lateral speed (cm/s)
    %Output:
    % error = 1 - error in function
    % vel_front_left - front left wheel rotation speed (rad/s)
    % vel_front_right - front right wheel rotation speed (rad/s)
    % vel_rear_left - rear left wheel rotation speed (rad/s)
    % vel_rear_right - rear right wheel rotation speed (rad/s)
    if(error==1)
        return;
    end

    % set robot linear velocity and angular displacement for steering
    % get also pose. it can be used vehicle.get_vehicle_pose() instead
    [error,xrobot, yrobot, phirobot] = vehicle.set_velocity(vel_front_left,vel_front_right,vel_rear_left,vel_rear_right);
    %Input:
    % vel_front_left - rad/s
    % vel_front_right - rad/s
    % vel_rear_left - rad/s
    % vel_rear_right - rad/s
    %Output:
    % robot  (xrobot,yrobot) in cm
    % phirobot - rad
    % error = 1 - error in function
    if(error==1)
        return;
    end

    %[x, y, phi_2pi] = vehicle.get_vehicle_pose2pi();

    % trigger obstacles data reception
    error = vehicle.trigger_obstacles();
    % error = 1 - error in function
    if(error==1)
        return;
    end

    %Get target position for target 1 or 2 (itarget)
    [error,XTARGET,YTARGET] = sim.get_TargetPosition(itarget);
    %Input:
    %itarget - selection of target to get location
    %Output:
    %XTARGET - location information on the x-axis of the target (cm)
    %YTARGET - location information on the y-axis of the target (cm)
    % error = 1 - error in function
    if(error==1)
        return;
    end

    %Get simulation time
    [error,sim_time] = sim.get_simulation_time();
    %Output:
    % sim_time - information of simulation time
    % error = 1 - error in function
    if(error==1)
        return;
    end

    %trigger simulation step
    sim.trigger_simulation();

    %% Processing step
    % Obtain the distances of the sectors of interest in obstacle avoidance
    %dynamics.
    [error,dist] = vehicle.get_DistanceSensorAquisition(true, false);
    % Output:
    % dist - array of distances for each sector of obstacle avoidance
    %dynamics (cm)
    % error = 1 - error in function
    if(error==1)
        return;
    end

    % compute new vehicle velocity...
    % the simulation timestep is stored in timestep (value is not changed
    % while simulation is running)
    % the simulation time is stored in sim_time.
    %*===============================================
    %*===============================================
    %*----------- BEGIN YOUR CODE HERE ----------- %
    %-------------Navigation Direction-------------%
    psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot); % Angle in radians
    ftar = -lambdaTarget*sin(phirobot - psitarget);

    %-----------------Speed Control----------------%
    distance = sqrt((YTARGET - yrobot)^2 + (XTARGET - xrobot)^2);
    vrobot_des = distance/stop_time;
    euler_pass = 1/(lambdaTarget*10);
    if  (distance >= min_d_limit) && (distance <= max_d_limit) 
        vrobot_x = vrobot_x + euler_pass*(lambda_v*(vrobot_x-vrobot_des))
    elseif (distance >= min_d_limit)
        vrobot_x = 100.0
    else
        vrobot_x = 0.0
    end

    %--------------Obstacle Avoidance--------------%
    deltaThetaObs = theta_obs(2) - theta_obs(1);
    for i = 1:obsSensorNumber
        lambda_obs(i)   = B1*exp(-dist(i)/B2);
        psi_obs(i)      = phirobot + theta_obs(i);
        sigma(i)        = atan(tan(deltaThetaObs/2) + (1.2*rob_W/2)/((rob_L/2) + dist(i)));
        fobs(i)         = lambda_obs(i)*(phirobot - psi_obs(i))*exp(-(phirobot - psi_obs(i))*(phirobot - psi_obs(i))/(2*sigma(i)*sigma(i)));
        
        Fobs = Fobs +  fobs(i);
    end
    f_stock = sqrt(Q)*randn(1,obsSensorNumber);
    wrobot = Fobs + f_stock + ftar;
    Fobs = 0;
    %*===============================================
    %*===============================================
    %*------------- END OF YOUR CODE -------------
%     f_stock = sqrt(Q)*randn(1,N);
%     wrobot = Fobs + f_stock + ftar;
%     Fobs = 0;
%     %--------------------Inits-------------------- %
%     dt = timestep;
%     tau_tar = 15*dt;
%     lambda_tar = 1/tau_tar;
     
%     delta_y = YTARGET - y;
%     delta_x = XTARGET - x;
%     psi_tar = atan2(delta_y, delta_x);

%    %--------------------ObsAv-------------------- %
%     %tau_obs = 1/10*dt;     
%     Too_far = 100;       
%     N = length(theta_obs)                   
%     beta1 = 10;                 
%     beta2 = 30;                
%     Dtheta = theta_obs(2) - theta_obs(1);      
%     Q = 0.001;
   

%     %--------------------Vector-------------------- %
%     psi_obs = zeros(N,1);
%     lambda_obs =zeros(N,1);
%     sigma = zeros(N,1);
%     alfa_pot = zeros(N,1);
%     f_tar=-lambda_tar*sin(phi-psi_tar);

%     f_stoch = sqrt(Q)*randn(1,1);
    
%     gamma_obs = zeros(N,1);
%     k = zeros(N,1);
%     Vpot = zeros(N,1);
%     alpha = zeros(N,1);

%     fobs = zeros(N,1);
%     Fobs = 0;
%     for i=1:N
%         psi_obs(i) = phi+theta_obs(i);
%         %if dist(i) <= Too_far
%         lambda_obs(i) = beta1*exp(-dist(i)/beta2);
%         %end
%         if dist(i) > Too_far
%         lambda_obs(i) = 0;
%         end    
%         sigma(i) = atan(tan(Dtheta/2) + (1.2*rob_W/2)/((rob_L/2) + dist(i)));
%         fobs(i) = lambda_obs(i)*(phi-psi_obs(i))*exp(-(phi-psi_obs(i))^2/(2*(sigma(i)^2)));
%         if fobs(i) <= 0.2
%             k(i) = (lambda_obs(i)*sigma(i)^2)/(sqrt(exp(1)));
%             Vpot(i) = lambda_obs(i)*(sigma(i)^2)*exp((-(phi-psi_obs(i))^2)/(2*sigma(i)^2)) - k(i);
%             alpha(i) = atan(10 * Vpot(i))/pi;
%             gamma_obs(i) = -((alpha(i)-1)/2);
%             Fobs = Fobs + (gamma_obs(i)*fobs(i));
%         else
%             Fobs = Fobs + fobs(i);
%         end
%     end
    
%     %--------------------------------------------- % 

%     for i=1:10
%         if fobs(i) >= 0.2 && fobs(30-i) <= -0.2

%         Fobs = Fobs - fobs(i) - fobs(30-i);

%         end
%     end

%     wrobot = Fobs + f_tar +f_stoch;
%     vrobot_x = 40;
   
%    %--------------------Target-------------------- %
%     d = sqrt((delta_x)^2+(delta_y)^2);
%     if(d<15)    
%         if(itarget==1)
%             itarget=2;
%         else
%             vrobot_x =0;
%         end

%     end
  

    %%------------- END OF YOUR CODE -------------

    %%----------------------------------------------------------------------
    %It allows to guarantee a minimum cycle time of 50ms for the
    %computation cycle in Matlab
    time = toc(start);
    if time<0.05
        pause(0.05-time);
    else
        pause(0.01);
    end
    start = tic;
    %----------------------------------------------------------------------
end
sim.terminate();


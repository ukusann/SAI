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

itarget=1; %initialize first target
start = tic;

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
    [error,x, y, phi] = vehicle.set_velocity(vel_front_left,vel_front_right,vel_rear_left,vel_rear_right);
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
    
    %%----------- BEGIN YOUR CODE HERE ----------- %
    
    dt = timestep;
    tau_tar = 25 * dt;
    lambda_tar = 1/tau_tar;
    tau_obs_min = 5*dt;
    beta_1 = 1/tau_obs_min;
    beta_2 = 27;
    Q = 0.001;
    Q_sqrt = sqrt(Q);
    N = length(theta_obs); % NUMERO DE SENSORES
    TOO_FAR = 50;
    Dtheta = theta_obs(2) - theta_obs(1); % Valores obtidos na função getcharacteristics

    psi_obs = zeros(N, 1);
    lambda_obs = zeros(N, 1);
    sigma = zeros(N, 1);
    f_obs_i = zeros(N, 1);
   
    f_stoch = Q_sqrt * randn(1,1);

    psi_tar = atan2(YTARGET - y,XTARGET-x); % utilizar atan2 devido ha existência de dois valores para cada par de dados
    
    ftar = -lambda_tar*sin(phi - psi_tar); 
    
    f_obs=0;

    U_robot = 0;
    
      
   for i=1:29

    psi_obs(i) = phi + theta_obs(i);
    lambda_obs(i) = beta_1*exp(-dist(i)/beta_2);
    sigma(i) = atan(tan(Dtheta/2) + (1.2*rob_W/2)/((rob_L/2) + dist(i)));  %6*

    if(dist(i)>=TOO_FAR)

        lambda_obs(i) = 0;

    end

    f_obs_i(i) = lambda_obs(i) * (phi - psi_obs(i)) * exp((-(phi - psi_obs(i))^2) / (2 * sigma(i)^2));
    f_obs = f_obs + f_obs_i(i); 

    if(i>=14 && i<=16) %só considerar os sensores da frente
        k = (lambda_obs(i)*(sigma(i))^2)/sqrt(exp(1));
        U_pot= lambda_obs(i)*(sigma(i)^2)*exp(-(phi - psi_obs(i))^2)/(2*(sigma(i))^2) - k;
        U_robot = U_pot + U_robot;
    end


   end

    wrobot= f_obs + ftar + f_stoch;

    dist_tar = sqrt((XTARGET-x)^2+(YTARGET - y)^2);

    dist_tar_final = pi/2 - phi;

    if(U_robot > 0) %está perto do obstáculo
        % vrobot_x = 0;
        % vrobot_y = 60;
        sensor_esq = dist(24) + dist(25) + dist(26)
        sensor_dir = dist(4) + dist(5) + dist(6)
        if( sensor_esq < sensor_dir) %sinal que está mais perto um obstaculo no lado esquerdo, pois soma das dist menor, logo andar em y para o sentido oposto
            vrobot_x = 0;
            vrobot_y = -60;
        else %sinal que está mais perto um obstaculo no lado direito, logo andar em y para o sentido oposto
            % e coloca por predefenição  o igual para o lado direto
            vrobot_x = 0;
            vrobot_y = 60;
        end
           
     
    else

        if(dist_tar < 3)

            vrobot_x = 0;
            vrobot_y = 0;
            wrobot = dist_tar_final;
            itarget = 2;
        else 

            % vrobot_x = 65*(1-exp(-dist_tar/25));
            vrobot_x = 40*(1-exp(-dist_tar/25));
            vrobot_y = 0; %desligar a velocidade de y sinal que já passou o obstaculo

        end 

    end
  


   %----------------------view_dynamics--------------------

    %    view_dynamics = 0;
    % 
    % if (view_dynamics)
    % 
    %     % Definição de phi_plot e outros parâmetros
    %     phi_plot = (0: 5 :360) * pi/180;
    %     phi_plot_deg = (0: 5 :360);
    % 
    %     % Cria uma nova figura
    %     figure(1)
    % 
    %     % Plot dos gráficos das observações
    %     subplot(2, 2, 1);
    %     hold on;
    %     cla;
    % 
    % 
    %     ftar_plot = -lambda_tar*sin(phi_plot- psi_tar); %lphi = lenght(phi_plot);
    % 
    %     plot(phi_plot_deg, ftar_plot, 'LineWidth',2);
    %     xlim([0 360])
    %     ylabel('dφ/dt')
    %     xline(rad2deg(phi+ 2*pi*(phi < 0)), 'LineWidth',2)
    %     grid on;
    %     title('ftar');
    % 
    % 
    %     % Plot dos gráficos das observações
    %     subplot(2, 2, 2);
    %     hold on;
    %     cla;
    %     Fobs_plot = 0; % Inicialização do vetor de soma
    %     U_robot_plot = 0;
    %     xline(rad2deg(phi+ 2*pi*(phi < 0)), 'LineWidth',2)
    % 
    %     for i=1:N
    % 
    %         fobs_plot = (lambda_obs(i)*(phi_plot - psi_obs(i)).*exp(-(phi_plot - psi_obs(i)).^2)/(2*sigma(i)^2));
    %         k_plot = (lambda_obs(i)*(sigma(i))^2)/sqrt(exp(1));
    %         U_pot_plot= lambda_obs(i)*((sigma(i))^2).*exp(-(phi_plot - psi_obs(i)).^2)/(2*(sigma(i))^2) - k_plot;
    %         U_robot_plot = U_pot_plot + U_robot_plot;
    % 
    %         % Plot dos gráficos dentro do loop
    %         grid on;
    %         title('Individual fobs');
    %         plot(phi_plot_deg, fobs_plot, 'LineWidth',2);
    %         xlim([0 360])
    %         ylabel('dφ/dt')
    % 
    %         Fobs_plot = Fobs_plot + fobs_plot; 
    % 
    %     end
    % 
    %     % Plot do gráfico somado dos obstáculos
    %     subplot(2, 2, 3);
    %     cla;
    %     plot(phi_plot_deg, Fobs_plot, 'LineWidth',2);
    %     hold on;
    %     plot(phi_plot_deg, U_robot_plot, 'LineWidth',2);
    %     xlim([0 360])
    %     ylabel('dφ/dt')
    %     xline(rad2deg(phi+ 2*pi*(phi < 0)), 'LineWidth',2)
    %     grid on;
    %     title('Fobs and Up,obs');
    % 
    %     Ftotal_plot = ftar_plot + Fobs_plot;
    % 
    %     % Plot do gráfico total
    %     subplot(2, 2, 4);
    %     cla;
    %     plot(phi_plot_deg, Ftotal_plot, 'LineWidth',2);
    %     xlim([0 360])
    %     ylabel('dφ/dt')
    %     xline(rad2deg(phi+ 2*pi*(phi < 0)), 'LineWidth',2)
    %     grid on;
    %     title('Ftotal');
    % 
    % end 

       
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

    
0%% IMPORTANT:
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

% need to choose the arm to control
robot_name = 'LBR_iiwa_14_R820';
% need to choose the gripper/hand
hand_name = '2FGP20';
%Creation of a communication class object with the manipulator arm
% Input:
% sim - pointer to class simulator_interface
% robot_name - name of arm CoppeliaSim model
% hand_name - name of hand CoppeliaSim model
% Output:
% robot_arm - pointer to class arm_interface
% error_sim = 1 - impossible to connect to simulator
[robot_arm,error_man] = arm_interface(sim,robot_name,hand_name);
if error_man == 1
    sim.terminate();
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

%This function returns relevant information about the robotic arm
[error,nJoints,Links,DistanceHand,MinPositionJoint,MaxPositionJoint] = robot_arm.get_RobotCharacteristics();
%nJoints - number of arm joints.
%Links - dimensions of the links between the axes of rotation
%DistanceHand - distance between the tip of the manipulator and the palm of
% the hand
%MinPositionJoint - array with minimum position for joint 1-6
%MaxPositionJoint - array with maximum position for joint 1-6
if error == 1
    sim.terminate();
    return;
end

error = robot_arm.open_hand(); %Initialize with hand opened
if error == 1
    sim.terminate();
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

%*==================================================
%*=================Parameters=======================
%******* Mobile Robot *******  
vrobot_des  = 100;
lambdaTarget = 2.3;
stop_time = 4;
vinit = 50;
min_d_limit = 150;
max_d_limit = 300;
%***********************

%***** Obstacle Avoidance *****
B1 = 20;
B2 = 30;
Q = 0.005;

obsSensorNumber = 29;
    
lambda_obs  = zeros(obsSensorNumber, 1);
sigma       = zeros(obsSensorNumber, 1);
fobs        = zeros(obsSensorNumber, 1);
psi_obs     = zeros(obsSensorNumber, 1);

Fobs = 0;
f_stock = sqrt(Q)*rand(1,obsSensorNumber);

changeTargetDist = 40;
%***********************

%***** Speed Control *****
lambda_v = 5;
euler_pass = 1/(lambdaTarget*10);
%***********************

%***** Parking System *****
phi_parking = [pi/2, pi, 0, pi/2];
delay_exitPark = 0;
%***********************

%***** Robotic Arm *****
close_gripper = 0;
open_gripper = 0;
isGripperClosed = 0;
delay_grip = 0;
armMoved = 0;
delay_movArm = 0;

%***********************

%***** System Flags *****
currentState = states.GoToTarget;
nextState = currentState;
move_arm = 0;
go_to_target = 1;
parkPositionReached = 0;
isParked = 0;
picked = 0;
transPosArm = 0;
defPosArm = 0;
exitPark = 0;

stopSim = 0;
placed = 0;
%*==================================================

%*---------------------- Initial Commands -------------------
sim.move_conveyorbelt();
robot_arm.set_joints_defPos();
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
    %% Begin Code
    %%? ----------------State Idle----------------
    %%? ------------------------------------------

    %%? ------------State GoToTarget--------------
    if(currentState == states.GoToTarget)
        %-------------Navigation Direction-------------%
        psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot); % Angle in radians
        ftar = -lambdaTarget*sin(phirobot - psitarget);

        %-----------------Speed Control----------------%
        distance = sqrt((YTARGET - yrobot)^2 + (XTARGET - xrobot)^2);

        if(distance >= max_d_limit) % Set desired speed to max value
            vrobot_des = 100.0;

        elseif((distance >= min_d_limit) && (distance <= max_d_limit)) % Calculate and set desired speed
            vrobot_des = distance/stop_time;

        elseif(distance <= min_d_limit) % Set desired speed to min value
            vrobot_des = 30.0;
            parkPositionReached = 1; %todo: Change State Aux Flag

        else
            vrobot_des = 0.0;
        end

        if(parkPositionReached == 0) % Update robot's speed if parking position has not been reached
            vrobot_y = 0.0; % Forces speed on y-axis to 0
            acc = -lambda_v*(vrobot_x - vrobot_des); % Calculates acceleration with current speed and desired value
            vrobot_x = vrobot_x + acc*euler_pass; % Uses euler pass as time variable for speed formula
        end
        %--------------Obstacle Avoidance--------------%
        deltaThetaObs = theta_obs(2) - theta_obs(1);
        for i = 1:obsSensorNumber
            lambda_obs(i)   = B1*exp(-dist(i)/B2);
            psi_obs(i)      = phirobot + theta_obs(i);
            sigma(i)        = atan(tan(deltaThetaObs/2) + (rob_L/2)/((rob_W/2) + dist(i)));
            fobs(i)         = lambda_obs(i)*(phirobot - psi_obs(i))*exp(-(phirobot - psi_obs(i))*(phirobot - psi_obs(i))/(2*sigma(i)*sigma(i)));
            Fobs = Fobs + fobs(i);
        end

        f_stock = sqrt(Q)*randn(1,obsSensorNumber);
        wrobot = Fobs + f_stock + ftar;
        Fobs = 0;
    end
    %%? ------------------------------------------

    %%? ---------- State Init Parking ------------
    if(currentState == states.InitParking)
        distance = sqrt((YTARGET - yrobot)^2 + (XTARGET - xrobot)^2);
        if(distance<changeTargetDist)    
            isParked = 1; %todo: Change State Aux Flag
        else
            vrobot_x = vrobot_des * cos(psitarget - phi_parking(itarget));
            vrobot_y = vrobot_des * sin(psitarget - phi_parking(itarget));
            ftar = -lambdaTarget*sin(phirobot - phi_parking(itarget));
            wrobot = ftar;
        end
    end
    %%? ------------------------------------------

    %%? -------------- State Pick ----------------
    if(currentState == states.PickBox)
        % Force the robot to stop movement
        vrobot_y = 0.0;
        vrobot_x = 0.0;
        % wrobot = 0.0;
        if(close_gripper == 0)

            if(itarget == 2)
                armJoints(1)=90*pi/180;
                armJoints(2)=0*pi/180;
                armJoints(3)=-90*pi/180;
                armJoints(4)=-90*pi/180;
                armJoints(5)=90*pi/180;
                armJoints(6)=0*pi/180;
                armJoints(7)=0*pi/180;
            elseif(itarget == 3)
                armJoints(1)=90*pi/180;
                armJoints(2)=0*pi/180;
                armJoints(3)=-90*pi/180;
                armJoints(4)=-90*pi/180;
                armJoints(5)=90*pi/180;
                armJoints(6)=0*pi/180;
                armJoints(7)=0*pi/180;
            else
                disp('Error in target to move robotic arm!');
                sim.terminate();
                return;
            end
            
            error = robot_arm.set_joints(armJoints); %send value for arm Joints in rad
            if error == 1
                sim.terminate();
                return;
            end
            close_gripper = 1;
        
        else
            delay_grip = delay_grip + toc(start);
            if(delay_grip > 3 && isGripperClosed == 0)
                error = robot_arm.close_hand();
                if error == 1
                    sim.terminate();
                    return;
                end
                isGripperClosed = 1;
                
            elseif(delay_grip > 6)
                close_gripper = 0;
                delay_grip = 0;
                picked = 1; %todo: Change State Aux Flag
            end
        end
    end
    %%? ------------------------------------------

    %%? -------------- State Place ---------------
    if(currentState == states.PlaceBox)
        % Force the robot to stop movement
        vrobot_y = 0.0;
        vrobot_x = 0.0;
        % wrobot = 0.0;
        if(open_gripper == 0)

            % if(itarget == 2)
            %     armJoints(1)=0*pi/180;
            %     armJoints(2)=0*pi/180;
            %     armJoints(3)=0*pi/180;
            %     armJoints(4)=0*pi/180;
            %     armJoints(5)=0*pi/180;
            %     armJoints(6)=0*pi/180;
            %     armJoints(7)=0*pi/180;
                
            % elseif(itarget == 3)
            %     armJoints(1)=0*pi/180;
            %     armJoints(2)=0*pi/180;
            %     armJoints(3)=0*pi/180;
            %     armJoints(4)=0*pi/180;
            %     armJoints(5)=0*pi/180;
            %     armJoints(6)=0*pi/180;
            %     armJoints(7)=0*pi/180;
            % else
            %     disp('Error in target to move robotic arm!');
            %     sim.terminate();
            %     return;
            % end
            armJoints(1)=0*pi/180;
            armJoints(2)=0*pi/180;
            armJoints(3)=0*pi/180;
            armJoints(4)=0*pi/180;
            armJoints(5)=0*pi/180;
            armJoints(6)=0*pi/180;
            armJoints(7)=0*pi/180;
            
            error = robot_arm.set_joints(armJoints); %send value for arm Joints in rad
            if error == 1
                sim.terminate();
                return;
            end
            open_gripper = 1;
        
        else
            delay_grip = delay_grip + toc(start);
            if(delay_grip > 3)
                error = robot_arm.open_hand();
                if error == 1
                    sim.terminate();
                    return;
                end
                delay_grip = 0;
                open_gripper = 0;
                picked = 0;
                placed = 1; %todo: Change State Aux Flag
            end
        end
    end
    %%? ------------------------------------------

    %%? -------------- State MoveArm -------------
    if(currentState == states.MoveArm)
        if(armMoved == 0)
            if(picked == 1) % Move to transport position
                armJoints(1)=-90*pi/180;
                armJoints(2)=0*pi/180;
                armJoints(3)=90*pi/180;
                armJoints(4)=90*pi/180;
                armJoints(5)=-90*pi/180;
                armJoints(6)=0*pi/180;
                armJoints(7)=0*pi/180;
                error = robot_arm.set_joints(armJoints); %send value for arm Joints in rad
                if error == 1
                    sim.terminate();
                    return;
                end
                armMoved = 1;
            elseif(placed == 1) % Move to default position
                error = robot_arm.set_joints_defPos(); %send value for arm Joints in rad
                if error == 1
                    sim.terminate();
                    return;
                end
                armMoved = 1;
                
            else
                disp('Error: Cannot identify if box was picked or placed!')
                sim.terminate();
                return;
            end
        else
            delay_movArm = delay_movArm + toc(start);
            if(delay_movArm > 5)
                delay_movArm = 0;
                if(picked == 1)
                    transPosArm = 1; %todo: Change State Aux Flag
                elseif(placed == 1)
                    defPosArm = 1; %todo: Change State Aux Flag
                end
            end
        end
    end
    %%? ------------------------------------------

    %%? ---------- State Exit Parking ------------
    if(currentState == states.ExitParking)
        delay_exitPark = delay_exitPark + toc(start);
        if(delay_exitPark > 5)
            exitPark = 1; %todo: Change State
            delay_exitPark = 0;
            vrobot_y = 0.0;
            exit_parking = 0;
            if(itarget == 1)
                itarget = 2;
      
            elseif(itarget == 2 && picked == 1)
                itarget = 4;
            elseif(itarget == 2 && placed == 1) 
                itarget = 1;   

            elseif(itarget == 3 && picked == 1)
                itarget = 4;
            elseif(itarget == 3 && placed == 1) 
                itarget = 1; 

            elseif(itarget == 4)
                vrobot_x = 0;
                vrobot_y = 0;
                stopSim = 1; %todo: Change State
            else
                vrobot_x = 0;
                vrobot_y = 0;
            end
        else
            if(itarget ~= 4)
                vrobot_x = vrobot_des * -cos(psitarget - phi_parking(itarget));
                vrobot_y = vrobot_des * -sin(psitarget - phi_parking(itarget));
                ftar = -lambdaTarget*sin(phirobot - phi_parking(itarget));
                wrobot = ftar;
            end
        end
    end
    %%? ------------------------------------------

    %%? ----------- State GoToDefPos -------------
    %%? ------------------------------------------

    %%? ------------ Update State ----------------
    if(currentState == states.GoToTarget)
        if(parkPositionReached == 1)
            parkPositionReached = 0; % Reset aux flag
            nextState = states.InitParking; 
        else
            nextState = states.GoToTarget;
        end

    elseif(currentState == states.InitParking)
        if(isParked == 1)
            isParked = 0; % Reset aux flag
            if(itarget == 1)
                nextState = states.ExitParking;
            elseif(picked == 0)
                nextState = states.PickBox;
            else
                nextState = states.PlaceBox;
            end
        else
            nextState = states.InitParking;   
        end

    elseif(currentState == states.PickBox)
        if(picked == 1)
            nextState = states.MoveArm;
        else
            nextState = states.PickBox;
        end

    elseif(currentState == states.MoveArm)
        if(transPosArm == 1 || defPosArm == 1)
            transPosArm = 0;
            defPosArm = 0;
            nextState = states.ExitParking;
        else
            nextState = states.MoveArm;
        end

    elseif(currentState == states.ExitParking)
        if(exitPark == 1)
            exitPark = 0;
            if(stopSim == 1)
                nextState = states.Idle;
            else 
                nextState = states.GoToTarget;
            end
        else
            nextState = states.ExitParking;
        end
    end

    currentState = nextState
    %%? ------------------------------------------
    %*===============================================
    %*===============================================
    %*------------- END OF YOUR CODE -------------
    %% End of Code

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


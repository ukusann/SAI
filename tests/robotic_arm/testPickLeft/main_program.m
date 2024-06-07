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

% [vision_rob, error_rob] = vision(sim);
% if (error_rob == 1)
%     return;
% end

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

[error,theta] = robot_arm.get_joints();
%theta - Values for each joint.
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
vrobot_min  = 30;
vrobot_des = 60;
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
phi_parking = [pi/2, pi, 0, pi/2, pi];
delay_exitPark = 0;
delay_rotate = 0;
%***********************

%***** Robotic Arm *****
close_gripper = 0;
open_gripper = 0;
isGripperClosed = 0;
isGripperOpened = 0;
delay_grip = 0;
armMoved = 0;
delay_movArm = 0;
startRotate = 0;
%* Joint limits
kuka_joint_lim_min = MinPositionJoint;
kuka_joint_lim_max = MaxPositionJoint;
%* Denavit-Hartenberg modified parameters: alpha(i-1), a(i-1), di, thetai
dh_alpha = pi/180*[0, -90, 90, 90, -90, -90, 90]';
dh_a = [0, 0, 0, 0, 0, 0, 0]';
dh_d = [Links(1), 0, Links(2), 0, Links(3), 0, Links(4)]';
dh_theta = pi/180*[0, 0, 0, 0, 0, 0, 0]';

%* Kinematics class object creation
kuka_kinematics = kinematics(kuka_joint_lim_min, kuka_joint_lim_max, Links);
alpha = 0*pi/180;

lastSolution = theta;
finalSolution = zeros(7,1);
%***********************

%***** Box Handler *****
boxesPose = zeros(6, 6);
%***********************

%***** System Flags *****
boxAvailable = 0;
retrieve_box = 0;
retShelfRight = 0;
parkPositionReached = 0;
isParked = 0;
picked = 0;
transPosArm = 0;
atEndEffector = 0;
exitPark = 0;
move_omni = 0;
placed = 0;
defPositionReached = 0;
%***** Temp Flags *****
box_high = 1;
box_low = 0;
tmpDelay = 0;
waitForBox = 0;
start_traj = 0;
closeGripper = 0;
waitToMove = 1;
exitShelf = 0;
%*==================================================

%*---------------------- Initial Commands -------------------
stateMachine = statesHandler(rob_L, rob_W, lambdaTarget, lambda_v, max_d_limit, min_d_limit, stop_time, euler_pass, obsSensorNumber, B1, B2, Q, changeTargetDist, phi_parking);
currentState = states.MoveArmEndEffector;
nextState = currentState;
sim.move_conveyorbelt(1);
robot_arm.set_joints_defPos();
%*==================================================
%%%---------------------- Start Robot Motion Behavior -------------------
while itarget<=sim.TARGET_Number % until robot goes to last target (TARGET_Number)
    %% Robot interface
    % set and get information to/from CoppeliaSim
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();

    % theta - get joint value (rad) for arm
    [error,theta] = robot_arm.get_joints();
    if error == 1
        return;
    end

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


    for i=1:sim.OBJECT_Number
        [error, boxesPose(i, :)] = sim.get_boxPose(i);
        if(error==1)
            return;
        end 
    end

    [error, robotPoseInWorldRef] = robot_arm.get_robot_pose();

    % compute new vehicle velocity...
    % the simulation timestep is stored in timestep (value is not changed
    % while simulation is running)
    % the simulation time is stored in sim_time.
    %*===============================================
    %*===============================================
    %*----------- BEGIN YOUR CODE HERE ----------- %
    %% Begin Code
    %%? ----------------State Idle----------------
    if(currentState == states.Idle)
        tmpDelay = tmpDelay + toc(start);
        if(tmpDelay > 5)
            tmpDelay = 0;
            boxesPose
            % [error, ~, frame] = vision_rob.getFrame();
            % if(error==1)
            %     % sim.terminate;
            %     return;
            % end
            % boxToPick = vision_rob.processFrame(frame);
        end
    end
    %%? ------------------------------------------

    %%? ------------State GoToTarget--------------
    if(currentState == states.GoToTarget)
       [vrobot_y, vrobot_x, wrobot, parkPositionReached] = stateMachine.handlerGoToTarget(YTARGET, XTARGET, yrobot, xrobot, vrobot_x, phirobot, lambda_obs, psi_obs, sigma, fobs, theta_obs, dist);
    end
    %%? ------------------------------------------

    %%? ---------- State Init Parking ------------
    if(currentState == states.InitParking)
        [vrobot_y, vrobot_x, wrobot, isParked, phi_parking(itarget)] = stateMachine.handlerInitParking(YTARGET, XTARGET, yrobot, xrobot, vrobot_min, phirobot, phi_parking(itarget), itarget, fobs);
    end
    %%? ------------------------------------------

    %%? -------------- State Pick ----------------
    if(currentState == states.PickBox)
        [error, close_gripper, delay_grip, isGripperClosed, armJoints, setJoints, closeHand, picked] = stateMachine.handlerPickBox(itarget, close_gripper, delay_grip, isGripperClosed, start);
        if error == 1
            sim.terminate();
            return;
        end


        if(setJoints == 1)
            error = robot_arm.set_joints(armJoints);
            if error == 1
                sim.terminate();
                return;
            end
        elseif(closeHand == 1)
            error = robot_arm.close_hand();
            if error == 1
                sim.terminate();
                return;
            end
        end
    end
    %%? ------------------------------------------

    %%? -------------- State Place ---------------
    if(currentState == states.PlaceBox)
        [error, open_gripper, delay_grip, isGripperOpened, armJoints, setJoints, openHand, picked, placed] = stateMachine.handlerPlaceBox(itarget, open_gripper, delay_grip, isGripperOpened, start);
        if error == 1
            sim.terminate();
            return;
        end

        if(setJoints == 1)
            error = robot_arm.set_joints(armJoints);
            if error == 1
                sim.terminate();
                return;
            end
        elseif(openHand == 1)
            error = robot_arm.open_hand();
            if error == 1
                sim.terminate();
                return;
            end
        end
    end
    %%? ------------------------------------------

    %%? -------------- State MoveArmEndEffector -------------
    if(currentState == states.MoveArmEndEffector)
        [delay_movArm, setJoints, calcInvKin, joints, poseHand, stopTraj] = stateMachine.handlerMoveArmEndEffector(lastSolution, finalSolution, start_traj, delay_movArm, start, boxesPose(1, :), robotPoseInWorldRef);
        if(setJoints == 1)
            setJoints = 0;
            error = robot_arm.set_joints(joints);
            if(error == 1)
                sim.terminate();
                return;
            end
        elseif(calcInvKin == 1)
            calcInvKin = 0;
            [error, solPossible, jointAnglesSol1, jointAnglesSol2, jointAnglesSol3, jointAnglesSol4] = kuka_kinematics.inverseKinematics(alpha, poseHand);
            if(error == 1)
                sim.terminate();
                return;
            end
            if(solPossible == 1234)
                solutions = [jointAnglesSol1'; jointAnglesSol2'; jointAnglesSol3'; jointAnglesSol4'];
            elseif(solPossible == 34)
                solutions = [jointAnglesSol3', jointAnglesSol4'];
            elseif(solPossible == 13)
                solutions = [jointAnglesSol1', jointAnglesSol3'];
            else
                solutions = jointAnglesSol3';
            end
            optimalSolution = kuka_kinematics.chooseInvKinSolution(solutions);
            finalSolution = optimalSolution';
            start_traj = 1;
        elseif(stopTraj == 1)
            lastSolution = finalSolution;
            delay_movArm = 0;
            start_traj = 0;
            conveyorPosReached = 1; %todo: Change State
        end
    end
    %%? ------------------------------------------------

    %%? -------------- State MoveArmTransp -------------
    %%? ------------------------------------------------
    %%? ---------- State Exit Parking ------------
    if(currentState == states.ExitParking)
        [vrobot_y, vrobot_x, wrobot, itarget, delay_exitPark, waitForBox, move_omni, exitPark] = stateMachine.handlerExitParking(YTARGET, XTARGET, yrobot, xrobot, vrobot_min, phirobot, phi_parking(itarget), box_low, box_high, picked, itarget, delay_exitPark, start);
    end
    %%? ------------------------------------------

    %%? ----------- State GoToDefPos -------------
    if(currentState == states.GoToDefPos)
        [vrobot_y, vrobot_x, wrobot, startRotate, delay_rotate, defPositionReached] = stateMachine.handlerGoToDefPos(YTARGET, XTARGET, yrobot, xrobot, startRotate, phirobot, phi_parking(itarget), delay_rotate, start);
    end
    %%? ------------------------------------------

    %%? ------------ Update State ----------------
    if(currentState == states.Idle)
        if(boxAvailable == 1)
            boxAvailable = 0;
            nextState = states.GoToTarget;
            if(retrieve_box == 1)
                retrieve_box = 0;
                if(retShelfRight == 1)
                    retShelfRight = 0;
                    itarget = 3;
                else
                    itarget = 2;
                end
            else
                itarget = 1;
            end 
        else
            nextState = states.Idle;
        end
    elseif(currentState == states.GoToTarget)
        sim.move_conveyorbelt(1);
        if(parkPositionReached == 1)
            parkPositionReached = 0; % Reset aux flag
            if(itarget == 5)
                nextState = states.GoToDefPos; 
            else
                nextState = states.InitParking;
            end
        else
            nextState = states.GoToTarget;
        end

    elseif(currentState == states.GoToDefPos)
        if(defPositionReached)
            defPositionReached = 0;
            nextState = states.ExitParking;
        end

    elseif(currentState == states.InitParking)
        if(isParked == 1)
            isParked = 0; % Reset aux flag
            if(itarget == 1 || itarget == 5)
                nextState = states.ExitParking;
            else
                nextState = states.MoveArmEndEffector;
            end
        else
            nextState = states.InitParking;   
        end

    elseif(currentState == states.MoveArmEndEffector)
        if(atEndEffector == 1)
             atEndEffector = 0;
             if(picked == 0)
                 nextState = states.PickBox;
             else
                 nextState = states.PlaceBox;
             end
         else
             nextState = states.MoveArmEndEffector;
         end

    elseif(currentState == states.PickBox)
        if(picked == 1)
            nextState = states.MoveArmTransp;
        else
            nextState = states.PickBox;
        end
    
    elseif(currentState == states.PlaceBox)
        if(placed == 1)
            nextState = states.MoveArmTransp;
        else
            nextState = states.PlaceBox;
        end

    elseif(currentState == states.MoveArmTransp)
        if(transPosArm == 1)
             transPosArm = 0;
         else
             nextState = states.GoToDefPos;
         end

    elseif(currentState == states.ExitParking)
        if(exitPark == 1)
            exitPark = 0;
            if(waitForBox == 1)
                waitForBox = 0;
                nextState = states.Idle;
            elseif(move_omni == 1) 
                move_omni = 0;
                nextState = states.InitParking;
            else
                nextState = states.GoToTarget;
            end
        else
            nextState = states.ExitParking;
        end
    end

    currentState = nextState;
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
disp(['---' newline 'Exiting simulation...' newline '---']);
sim.terminate();
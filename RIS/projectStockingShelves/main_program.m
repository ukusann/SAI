
%   main_program.m
%   Main program for the control of manipulators
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2024
%   2024/02/19
%--------------------------------------------------------------------------
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%--------------------------------------------------------------------------
clear
% need to choose the arm to control
robot_name = 'LBR_iiwa_14_R820';

% need to choose the gripper/hand
hand_name = 'RG2';

%Number of targets for each colour of box
salsageCan  = [1, 2, 3];
salsageCanPos = zeros(3, 3);
mushroomCan = [4, 5, 6];
mushroomCanPos = zeros(3, 3);

%Number of positions in the shelves
frontShelf1 = [ 1,  2,  3,  4,  5,  6,  7,  8];
frontShelf1Pos = zeros(8, 3);
frontShelf2 = [ 9, 10, 11, 12, 13, 14, 15, 16];
frontShelf2Pos = zeros(8, 3);
frontShelf3 = [17, 18, 19, 20, 21, 22, 23, 24];
frontShelf3Pos = zeros(8, 3);
frontShelf4 = [25, 26, 27, 28, 29, 30, 31, 32];
frontShelf4Pos = zeros(8, 3);
frontShelf5 = [33, 34, 35, 36, 37, 38, 39, 40];
frontShelf5Pos = zeros(8, 3);
frontShelf6 = [41, 42, 43, 44, 45, 46, 47, 48];
frontShelf6Pos = zeros(8, 3);


% Creation of a communication class object with the simulator
% Try to connect to simulator
% Output:
% sim - pointer to class simulator_interface
% error_sim = 1 - impossible to connect to simulator
[sim,error_sim] = simulator_interface();
if(error_sim==1)
    return;
end

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

[error,timestep] = sim.get_simulation_timestep();
%get time step value (normally dt=50ms)
if error == 1
    sim.terminate();
    return;
end

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

%--------------------------------------------------------------------------
error = sim.move_conveyorbelt(); %Put the conveyor belt in motion
if error == 1
    sim.terminate();
    return;
end

error = robot_arm.open_hand(); %Initialize with hand opened
if error == 1
    sim.terminate();
    return;
end
%*==================================================
%*=================Parameters=======================
%******* Default Variables ******* 
start = tic;
m=1;
stop=0;
%***********************


%********** Robotic Arm Variables ***********
%* Joint limits
kuka_joint_lim_min = MinPositionJoint;
kuka_joint_lim_max = MaxPositionJoint;

%* Speed limits
kuka_vel_lim = pi/180*[85, 85, 100, 75, 130, 135, 135];

%* Denavit-Hartenberg modified parameters: alpha(i-1), a(i-1), di, thetai
dh_alpha = pi/180*[0, -90, 90, 90, -90, -90, 90]';
dh_a = [0, 0, 0, 0, 0, 0, 0]';
dh_d = [Links(1), 0, Links(2), 0, Links(3), 0, Links(4)]';
dh_theta = pi/180*[0, 0, 0, 0, 0, 0, 0]';

%****************************************************

%********** Symbolic Matrices ***********
% syms L1 L2 L3 L4
% syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
% symb_links = [L1, L2, L3, L4]; 
% symb_transf01 = symbolicCalcTransf(0, dh_a(1), symb_links(1), theta1);
% symb_transf12 = symbolicCalcTransf(sym(-pi)/2, dh_a(2), 0, theta2);
% symb_transf23 = symbolicCalcTransf(sym(pi)/2, dh_a(3), symb_links(2), theta3);
% symb_transf34 = symbolicCalcTransf(sym(pi)/2, dh_a(4), 0, theta4);
% symb_transf45 = symbolicCalcTransf(sym(-pi)/2, dh_a(5), symb_links(3), theta5);
% symb_transf56 = symbolicCalcTransf(sym(-pi)/2, dh_a(6), 0, theta6);
% symb_transf67 = symbolicCalcTransf(sym(pi)/2, dh_a(7), symb_links(4), theta7);

% symb_transf07 = symb_transf01 * symb_transf12 * symb_transf23 * symb_transf34 * symb_transf45 * symb_transf56 * symb_transf67;
% symb_handPos = symb_transf07(1:3, 4)
% symb_transf03 = symb_transf01*symb_transf12*symb_transf23
% symb_transf47 = symb_transf45*symb_transf56*symb_transf67
%****************************************************

%********** Kinematics class object creation ***********
kuka_kinematics = kinematics(kuka_joint_lim_min, kuka_joint_lim_max, Links);

%****************************************************

%********** Inverse Kinematics Variables ***********
rpy_des_deg = [0, 90, 90];
rpy_des_deg2 = [0, 110, 0];
rpy_des_deg3 = [0, 120, 0];
rpy_des_deg4 = [0, 90, 90];
alpha = 40*pi/180;
alpha2 = 45*pi/180;


%****************************************************

%*************** Global Variables *******************
stateMachine = statesHandler(DistanceHand);
currentState = states.Idle;
nexState = currentState;
lastSolution = [-2.2985, -1.1855, 0.8030, 2.0662, -0.2730, 1.3041, 2.4532]';
finalSolution = zeros(7,1);
delay_dir = 0;
thresholdPosInX = -0.787;
listMushCansToPick = [1, 1, 1];
listSalsCansToPick = [1, 1, 1];
canToPick = 0;
shelfToPlace = zeros(3,1);

%* ------ States flags --------
canInPosition = 0;
conveyorPosReached = 0;
isPicked = 0;
shelfPosReached = 0;
isPlaced = 0;
defPosReached = 0;

%* ------ States aux flags --------
start_traj = 0;
closeGripper = 0;
waitToMove = 1;
exitShelf = 0;
%****************************************************

%*************** Initial Commands *******************
robot_arm.set_joints_defPos();
%*==================================================
while stop==0
    %----------------------------------------------------------------------
    %% Robot interface
    % set and get information to/from vrep
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();

    % theta - get joint value (rad) for arm
    [error,theta] = robot_arm.get_joints();
    if error == 1
        sim.terminate();
        return;
    end

    % armPosition - get robot position
    [error,armPosition] = robot_arm.get_robot_position();
    if error == 1
        sim.terminate();
        return;
    end

    % objectPosition - get object position
    for i = 1:3
        [error, salsageCanPos(i, :)] = sim.get_object_position(salsageCan(i));
        if(error == 1)
            sim.terminate();
            return;
        end

        [error, mushroomCanPos(i, :)] = sim.get_object_position(mushroomCan(i));
        if(error == 1)
            sim.terminate();
            return;
        end
    end

    % objectPosition - get target position
    for i = 1:8
        [error,frontShelf1Pos(i, :)]=sim.get_target_position(frontShelf1(i));        
        if(error == 1)
            sim.terminate();
            return;
        end

        [error,frontShelf2Pos(i, :)]=sim.get_target_position(frontShelf2(i));        
        if(error == 1)
            sim.terminate();
            return;
        end

        [error,frontShelf3Pos(i, :)]=sim.get_target_position(frontShelf3(i));        
        if(error == 1)
            sim.terminate();
            return;
        end

        [error,frontShelf4Pos(i, :)]=sim.get_target_position(frontShelf4(i));        
        if(error == 1)
            sim.terminate();
            return;
        end

        [error,frontShelf5Pos(i, :)]=sim.get_target_position(frontShelf5(i));        
        if(error == 1)
            sim.terminate();
            return;
        end

        [error,frontShelf6Pos(i, :)]=sim.get_target_position(frontShelf6(i));        
        if(error == 1)
            sim.terminate();
            return;
        end
    end

    %get simulation time
    [error,sim_time] = sim.get_simulation_time();
    if error == 1
        sim.terminate();
        return;
    end 

    %trigger simulation step
    sim.trigger_simulation();
    %----------------------------------------------------------------------
    % --- YOUR CODE --- %
    %%? ----------------State Idle----------------
    if(currentState == states.Idle)
        for i = 1:3
            if(mushroomCanPos(i, 1) > thresholdPosInX)
                if(listMushCansToPick(i) == 1)
                    listMushCansToPick(i) = 0;
                    canToPick = 10 + i;
                    shelfToPlace = frontShelf3Pos(4, 1:3);
                    canInPosition = 1; %todo: change state
                    break;
                end
            end
            if(salsageCanPos(i, 1) > thresholdPosInX)
                if(listSalsCansToPick(i) == 1)
                    listSalsCansToPick(i) = 0;
                    canToPick = 20 + i;
                    shelfToPlace = frontShelf3Pos(5, 1:3);
                    canInPosition = 1; %todo: change state
                    break;
                end
            end
        end
    end
    %%? ------------------------------------------
    
    %%? ----------State MoveArmConveyor-----------
    if(currentState == states.MoveArmConveyor)
        offset = DistanceHand + 0.012;
        if(canToPick > 10 && canToPick < 20)
            desPosition = mushroomCanPos(canToPick - 10, 1:3);
        else
            desPosition = salsageCanPos(canToPick - 20, 1:3);
        end
        [delay_dir, setJoints, calcInvKin, joints, poseHand, stopTraj] = stateMachine.handlerMoveArmConveyor(lastSolution, finalSolution, start_traj, delay_dir, start, desPosition, rpy_des_deg, offset);
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
            delay_dir = 0;
            start_traj = 0;
            conveyorPosReached = 1; %todo: Change State
        end
      
    end
    %%? ------------------------------------------

    %%? -------------- State Pick ----------------
    if(currentState == states.Pick)
        if(waitToMove == 1)
            delay_dir = delay_dir + toc(start);
            if(delay_dir > 1)
                waitToMove = 0;
                delay_dir = 0;
            end
        else
            if(canToPick > 10 && canToPick < 20)
                desPosition = mushroomCanPos(canToPick - 10, 1:3);
            else
                desPosition = salsageCanPos(canToPick - 20, 1:3);
            end
            [delay_dir, setJoints, calcInvKin, closeGripper, joints, poseHand] = stateMachine.handlerPick(lastSolution, finalSolution, start_traj, delay_dir, start, desPosition, rpy_des_deg);
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
            elseif(closeGripper == 1)
                error = robot_arm.close_hand();    %close
                if error == 1
                    sim.terminate();
                    return;
                end
                lastSolution = finalSolution;
                start_traj = 0;
                waitToMove = 1;
                isPicked = 1; %todo: Change State
            end
        end
    end
    %%? ------------------------------------------

    %%? ----------State MoveArmShelf--------------
    if(currentState == states.MoveArmShelf)
        offset_y = -0.12;
        offset_z = 0.15;
        if(waitToMove == 1)
            delay_dir = delay_dir + toc(start);
            if(delay_dir > 3)
                waitToMove = 0;
                delay_dir = 0;
            end
        else
            desPosition = shelfToPlace;
            [delay_dir, setJoints, calcInvKin, joints, poseHand, stopTraj] = stateMachine.handlerMoveArmShelf(lastSolution, finalSolution, start_traj, delay_dir, start, desPosition, rpy_des_deg2, offset_y, offset_z);
            if(setJoints == 1)
                setJoints = 0;
                error = robot_arm.set_joints(joints);
                if(error == 1)
                    sim.terminate();
                    return;
                end
            elseif(calcInvKin == 1)
                calcInvKin = 0;
                [error, solPossible, jointAnglesSol1, jointAnglesSol2, jointAnglesSol3, jointAnglesSol4] = kuka_kinematics.inverseKinematics(alpha2, poseHand);
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
                start_traj = 0;
                waitToMove = 1;
                shelfPosReached = 1; %todo: Change State
            end
        end
    end
    %%? ------------------------------------------

    %%? ------------- State Place ----------------
    if(currentState == states.Place)
        offset_y = -0.12;
        offset_z = 0.15;
        if(waitToMove == 1)
            delay_dir = delay_dir + toc(start);
            if(delay_dir > 0.5)
                waitToMove = 0;
                delay_dir = 0;
            end
        else
            desPosition = shelfToPlace;
            [delay_dir, setJoints, calcInvKin, openGripper, joints, poseHand] = stateMachine.handlerPlace(lastSolution, finalSolution, start_traj, delay_dir, start, desPosition, rpy_des_deg3, offset_y, offset_z);
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
            elseif(openGripper == 1)
                error = robot_arm.open_hand();    %open
                if error == 1
                    sim.terminate();
                    return;
                end
                lastSolution = finalSolution;
                start_traj = 0;
                waitToMove = 1;
                isPlaced = 1; %todo: Change State
            end
        end
    end
    %%? ------------------------------------------

    %%? ---------- State GoToDefPos --------------
    if(currentState == states.GoToDefPos)
        if(waitToMove == 1)
            delay_dir = delay_dir + toc(start);
            if(delay_dir > 1)
                waitToMove = 0;
                delay_dir = 0;
            end
        elseif(exitShelf == 0)
            offset_y = -0.25;
            offset_z = 0.15;
            desPosition = shelfToPlace;
            [delay_dir, setJoints, calcInvKin, joints, poseHand, stopTraj] = stateMachine.handlerGoToDefPos(lastSolution, finalSolution, start_traj, delay_dir, start, desPosition, rpy_des_deg2, offset_y, offset_z);
            if(setJoints == 1)
                setJoints = 0;
                error = robot_arm.set_joints(joints);
                if(error == 1)
                    sim.terminate();
                    return;
                end
            elseif(calcInvKin == 1)
                calcInvKin = 0;
                [error, solPossible, jointAnglesSol1, jointAnglesSol2, jointAnglesSol3, jointAnglesSol4] = kuka_kinematics.inverseKinematics(alpha2, poseHand);
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
                start_traj = 0;
                waitToMove = 1;
                exitShelf = 1;
            end
        else
            offset_y = 0.0;
            offset_z = 0.0;
            desPosition = [-0.522, 0.0, 0.90];
            [delay_dir, setJoints, calcInvKin, joints, poseHand, stopTraj] = stateMachine.handlerGoToDefPos(lastSolution, finalSolution, start_traj, delay_dir, start, desPosition, rpy_des_deg4, offset_y, offset_z);
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
                start_traj = 0;
                waitToMove = 1;
                exitShelf = 0;
                defPosReached = 1; %todo: Change State
            end
        end
    end
    %%? ------------------------------------------

    %%? ------------ Update State ----------------
    if(currentState == states.Idle)
        if(canInPosition == 1)
            canInPosition = 0;
            nexState = states.MoveArmConveyor;
        end
    elseif(currentState == states.MoveArmConveyor)
        if(conveyorPosReached == 1)
            conveyorPosReached = 0;
            nexState = states.Pick;
        end
    elseif(currentState == states.Pick)
        if(isPicked == 1)
            isPicked = 0;
            nexState = states.MoveArmShelf;
        end
    elseif(currentState == states.MoveArmShelf)
        if(shelfPosReached == 1)
            shelfPosReached = 0;
            nexState = states.Place;
        end
    elseif(currentState == states.Place)
        if(isPlaced == 1)
            isPlaced = 0;
            nexState = states.GoToDefPos;
        end
    elseif(currentState == states.GoToDefPos)
        if(defPosReached == 1)
            defPosReached = 0;
            nexState = states.Idle;
        end    
    end

    currentState = nexState;
    %%? ------------------------------------------

    m=m+1;
    %----------------------------------------------------------------------
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
error = sim.terminate();
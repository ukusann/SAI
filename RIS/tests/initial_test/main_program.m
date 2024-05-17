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
% robot_name = 'UR10';
%robot_name = 'Kuka_LBRiisy';
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


%*Variables of robotic arm
% Joint limits
kuka_joint_lim_min = MinPositionJoint;
kuka_joint_lim_max = MaxPositionJoint;

% Speed limits
kuka_vel_lim = pi/180*[85, 85, 100, 75, 130, 135, 135];

% Denavit-Hartenberg modified parameters: alpha(i-1), a(i-1), di, thetai
dh_alpha = pi/180*[0, -90, 90, 90, -90, -90, 90]';
dh_a = [0, 0, 0, 0, 0, 0, 0]';
dh_d = [Links(1), 0, Links(2), 0, Links(3), 0, Links(4)]';
dh_theta = pi/180*[0, 0, 0, 0, 0, 0, 0]';

%*Get symbolic matrices
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

%*Creation of kinematics class
kuka_kinematics = kinematics(kuka_joint_lim_min, kuka_joint_lim_max, Links);

% armJoints(1) =60*pi/180;
% armJoints(2) =-60*pi/180;
% armJoints(3) =60*pi/180;
% armJoints(4) =60*pi/180; % theta4 needs to be positive always
% armJoints(5) =60*pi/180;
% armJoints(6) =60*pi/180;
% armJoints(7) =-60*pi/180;
armJoints(1) =-40*pi/180;
armJoints(2) = 40*pi/180;
armJoints(3) = 40*pi/180;
armJoints(4) = 40*pi/180; % theta4 needs to be positive always
armJoints(5) =-40*pi/180;
armJoints(6) = 40*pi/180;
armJoints(7) = 40*pi/180;
disp('Armjoints to be send to arm:')
armJoints'
error = robot_arm.set_joints(armJoints); %send value for arm Joints in rad
if error == 1
    sim.terminate();
    return;
end

%************** Direct Kinematics ******************
poseHand = kuka_kinematics.directKinematics(dh_alpha, dh_a, dh_d, dh_theta, armJoints);
% poseHand = [0, 0, 0, 0, 0, 0]';
dir_flag = 2;
delay_dir = 0;
%****************************************************


%************** Inverse Kinematics ******************
alpha = 45*pi/180;

[error, solutionsNum, joingAnglesSol1, joingAnglesSol2, joingAnglesSol3, joingAnglesSol4] = kuka_kinematics.inverseKinematics(alpha, poseHand)
if(error == 1)
    sim.terminate();
    return;
end
% disp(['Inverse Kinematics Number of Solutions: ', num2str(solutionsNum)])
robot_arm.set_joints(joingAnglesSol3);


%****************************************************

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
    %Direct Kinematics calculation
    if(dir_flag == 1)
        delay_dir = delay_dir + toc(start);
        if(delay_dir > 5)
            poseHand = kuka_kinematics.directKinematics(dh_alpha, dh_a, dh_d, dh_theta, theta);
            delay_dir = 0;
            dir_flag = 0;
        end
    elseif(dir_flag == 0)
        [error, solutionsNum, joingAnglesSol1, joingAnglesSol2, joingAnglesSol3, joingAnglesSol4] = kuka_kinematics.inverseKinematics(alpha, poseHand)
        if(error == 1)
            sim.terminate();
            return;
        end
        dir_flag = 2;
        % disp(['Inverse Kinematics Number of Solutions: ', num2str(solutionsNum)]);
    end
    %Inverse Kinematics - Send values for joints
    %Write joints.
    %armJoints(1)=0*pi/180;
    %armJoints(2)=90*pi/180;
    %armJoints(3)=0*pi/180;
    %armJoints(4)=0*pi/180;
    %armJoints(5)=0*pi/180;
    %armJoints(6)=0*pi/180;
    %armJoints(7)=0*pi/180;
    %error = robot_arm.set_joints(armJoints) %send value for arm Joints in rad
    %if error == 1
    %    sim.terminate();
    %    return;
    %end

    %Functions that allows open/close the hand
    %error = robot_arm.open_hand();     %open
    %error = robot_arm.close_hand();    %close
    %if error == 1
    %    sim.terminate();
    %    return;
    %end

    %Function that allows you to put the conveyor belt in motion/stop
    %error = sim.move_conveyorbelt();     %motion
    %error = sim.stop_conveyorbelt();     %stop
    %if error == 1
    %    sim.terminate();
    %    return;
    %end

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
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
dh_alpha = pi/180*[0, -90, 90, -90, 90, -90, 90];
dh_a = pi/180*[0, 0, 0, 0, 0, 0, 0];
dh_d = [Links(1), 0, Links(2), 0, Links(3), 0, Links(4)];
dh_theta = pi/180*[0, 0, 0, 0, 0, 0, 0];


%*Creation of kinematics class
kuka_kinematics = kinematics(kuka_joint_lim_min, kuka_joint_lim_max, Links);

%* Compute individual transformation matrices
transf_01 = kuka_kinematics.dhTransfMatrix(dh_alpha(1), dh_a(1), dh_d(1), dh_theta(1) + theta(1));
transf_12 = kuka_kinematics.dhTransfMatrix(dh_alpha(2), dh_a(2), dh_d(2), dh_theta(2) + theta(2));
transf_23 = kuka_kinematics.dhTransfMatrix(dh_alpha(3), dh_a(3), dh_d(3), dh_theta(3) + theta(3));
transf_34 = kuka_kinematics.dhTransfMatrix(dh_alpha(4), dh_a(4), dh_d(4), dh_theta(4) + theta(4));
transf_45 = kuka_kinematics.dhTransfMatrix(dh_alpha(1), dh_a(5), dh_d(5), dh_theta(5) + theta(5));
transf_56 = kuka_kinematics.dhTransfMatrix(dh_alpha(6), dh_a(6), dh_d(6), dh_theta(6) + theta(6));
transf_67 = kuka_kinematics.dhTransfMatrix(dh_alpha(7), dh_a(7), dh_d(7), dh_theta(7) + theta(7));

%* Compute general transformation matrix T_BE
transf_07 = transf_01*transf_12*transf_23*transf_34*transf_45*transf_56*transf_67;

%* Cartesian coordinates of Tip {7} with respect to base {0}
xe_0=transf_07(1,4);
ye_0=transf_07(2,4);
ze_0=transf_07(3,4);
pe_0=[xe_0, ye_0, ze_0]';


%* Orientation of Tip {7} with respect to base {0}: Roll-Pitch-Yaw
%angles:
rotation_07 = transf_07(1:3,1:3); 
[yaw_x, pitch_y, roll_z] = kuka_kinematics.computeMatrixToRPY(rotation_07);
Tip_orientation_deg = [yaw_x, pitch_y, roll_z]'*180/pi;

%* Conclusion of Direct kinematics
% Tip pose:
Pose_Tip = [pe_0; Tip_orientation_deg]

%*==================================================
while stop==0
    %----------------------------------------------------------------------
    %% Robot interface
    % set and get information to/from vrep
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();

    % ReadArmJoints - get joint value (rad) for arm
    [error,ReadArmJoints] = robot_arm.get_joints();
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
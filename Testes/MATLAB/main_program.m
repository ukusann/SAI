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
robot_name = 'UR10';
%robot_name = 'Kuka_LBRiisy';
%robot_name = 'LBR_iiwa_14_R820';

% need to choose the gripper/hand
hand_name = 'RG2';

%Number of targets for each colour of box
LATA_SALSICHAS_1 = 1;
LATA_SALSICHAS_2 = 2;
LATA_SALSICHAS_3 = 3;
LATA_COGUMELOS_1 = 4;
LATA_COGUMELOS_2 = 5;
LATA_COGUMELOS_3 = 6;

%Number of positions in the shelfe
PRATELEIRA1_FRENTE1 = 1;
PRATELEIRA1_FRENTE2 = 2;
PRATELEIRA1_FRENTE3 = 3;
PRATELEIRA1_FRENTE4 = 4;
PRATELEIRA1_FRENTE5 = 5;
PRATELEIRA1_FRENTE6 = 6;
PRATELEIRA1_FRENTE7 = 7;
PRATELEIRA1_FRENTE8 = 8;
PRATELEIRA2_FRENTE1 = 9;
PRATELEIRA2_FRENTE2 = 10;
PRATELEIRA2_FRENTE3 = 11;
PRATELEIRA2_FRENTE4 = 12;
PRATELEIRA2_FRENTE5 = 13;
PRATELEIRA2_FRENTE6 = 14;
PRATELEIRA2_FRENTE7 = 15;
PRATELEIRA2_FRENTE8 = 16;
PRATELEIRA3_FRENTE1 = 17;
PRATELEIRA3_FRENTE2 = 18;
PRATELEIRA3_FRENTE3 = 19;
PRATELEIRA3_FRENTE4 = 20;
PRATELEIRA3_FRENTE5 = 21;
PRATELEIRA3_FRENTE6 = 22;
PRATELEIRA3_FRENTE7 = 23;
PRATELEIRA3_FRENTE8 = 24;
PRATELEIRA4_FRENTE1 = 25;
PRATELEIRA4_FRENTE2 = 26;
PRATELEIRA4_FRENTE3 = 27;
PRATELEIRA4_FRENTE4 = 28;
PRATELEIRA4_FRENTE5 = 29;
PRATELEIRA4_FRENTE6 = 30;
PRATELEIRA4_FRENTE7 = 31;
PRATELEIRA4_FRENTE8 = 32;
PRATELEIRA5_FRENTE1 = 33;
PRATELEIRA5_FRENTE2 = 34;
PRATELEIRA5_FRENTE3 = 35;
PRATELEIRA5_FRENTE4 = 36;
PRATELEIRA5_FRENTE5 = 37;
PRATELEIRA5_FRENTE6 = 38;
PRATELEIRA5_FRENTE7 = 39;
PRATELEIRA5_FRENTE8 = 40;
PRATELEIRA6_FRENTE1 = 41;
PRATELEIRA6_FRENTE2 = 42;
PRATELEIRA6_FRENTE3 = 43;
PRATELEIRA6_FRENTE4 = 44;
PRATELEIRA6_FRENTE5 = 45;
PRATELEIRA6_FRENTE6 = 46;
PRATELEIRA6_FRENTE7 = 47;
PRATELEIRA6_FRENTE8 = 48;

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

start = tic;
m=1;
stop=0;
while stop==0
    %----------------------------------------------------------------------
    %% Robot interface
    % set and get information to/from vrep
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();

    % ReadArmJoints - get joint value (rad) for arm
    [error,ReadArmJoints] = robot_arm.get_joints()
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
    [error,objectPosition]=sim.get_object_position(LATA_SALSICHAS_1)
    %[error,objectPosition]=sim.get_object_position(LATA_SALSICHAS_2);
    %[error,objectPosition]=sim.get_object_position(LATA_SALSICHAS_3);
    %[error,objectPosition]=sim.get_object_position(LATA_COGUMELOS_1);
    %[error,objectPosition]=sim.get_object_position(LATA_COGUMELOS_2);
    %[error,objectPosition]=sim.get_object_position(LATA_COGUMELOS_3);
    if error == 1
        sim.terminate();
        return;
    end

    % objectPosition - get target position
    [error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE1)
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE2);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE3);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE4);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE5);
    % etc
    if error == 1
        sim.terminate();
        return;
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
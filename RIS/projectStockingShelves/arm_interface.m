%   arm_interface.m
%   Interface to a simulated robot in CoppeliaSim simulator
%   2024/02/19
%   % Copyright (C) 2024
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef arm_interface < handle
    properties (Access = private)
        vrep
        clientID

        RobotHandle  % handle for the robot object
        jointHandle  % 1,2,... handle for each of the joints of the manipulator
        HandHandle   % handle for the hand object (BarrrettHand)
    end

    properties
        number_of_joints  % variable indicating the number of joints of the manipulator arm
        robot_name        % variable indicating the name of the handler in CoppeliaSim
        hand_name         % variable indicating the name of the handles in CoppeliaSim
        MinJointPos       % vector indicating the minimum position allowed for the joint i
        MaxJointPos       % vector indicating the maximun position allowed for the joint i
    end

    methods
        function [obj,error] = arm_interface(sim_obj,robot_name,hand_name)
            obj.robot_name = robot_name;
            obj.hand_name = hand_name;

            robot_name = ['/',robot_name];
            hand_name = ['/',hand_name];

            error = 0;

            [obj.vrep, obj.clientID] = sim_obj.get_connection();

            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                error(msg)
            end

            %% Get objects handles

            %Get Arm Handle
            [res, obj.RobotHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, robot_name, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting robot handle');
                error = 1;
                return;
            end

            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end

            %arm joint handles
            stop = 0;
            j=1;
            while stop==0
                jointName{j} = [robot_name,'_joint',num2str(j)];

                [res,obj.jointHandle{j}]=obj.vrep.simxGetObjectHandle(obj.clientID,jointName{j},obj.vrep.simx_opmode_blocking);

                if (res ~= obj.vrep.simx_return_ok)
                    stop=1;
                    obj.number_of_joints = j-1;
                    obj.jointHandle{:,j} = [];
                    jointName{:,j} = [];
                else
                    %obj.vrep.simxSetJointTargetVelocity (obj.clientID,obj.jointHandle{j},0*pi/180,obj.vrep.simx_opmode_oneshot);

                    [res,~]=obj.vrep.simxGetJointPosition(obj.clientID, obj.jointHandle{j},obj.vrep.simx_opmode_streaming);
                    if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                        disp('ERROR: Failed getting joint position information');
                        error = 1;
                        return;
                    end
                end
                j=j+1;
            end

            %Get Hand Handle
            [res, obj.HandHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, hand_name, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting hand handle');
                error = 1;
                return;
            end
        end

        %Function that allows you to get the arm characteristics
        function [error,nJoints,Link,DistanceHand,MinPositionJoint,MaxPositionJoint] = get_RobotCharacteristics(obj)
            error = 0;
            nJoints = obj.number_of_joints;
            name = obj.robot_name;
            if strcmp(name,'IRB140')==1
                MinPositionJoint(1)=-180*pi/180;
                MinPositionJoint(2)=-110*pi/180;
                MinPositionJoint(3)=-50*pi/180;
                MinPositionJoint(4)=-180*pi/180;
                MinPositionJoint(5)=-120*pi/180;
                MinPositionJoint(6)=-180*pi/180;
                MaxPositionJoint(1)=180*pi/180;
                MaxPositionJoint(2)=90*pi/180;
                MaxPositionJoint(3)=230*pi/180;
                MaxPositionJoint(4)=180*pi/180;
                MaxPositionJoint(5)=120*pi/180;
                MaxPositionJoint(6)=180*pi/180;
                Link(1)=0.352;
                Link(2)=0.07;
                Link(3)=0.177;
                Link(4)=0.36;
                Link(5)=0.177;
                Link(6)=0.38;
                Link(7)=0.065;
            elseif strcmp(name,'UR10')==1
                MinPositionJoint(1)=-360*pi/180;
                MinPositionJoint(2)=-360*pi/180;
                MinPositionJoint(3)=-360*pi/180;
                MinPositionJoint(4)=-360*pi/180;
                MinPositionJoint(5)=-360*pi/180;
                MinPositionJoint(6)=-360*pi/180;
                MaxPositionJoint(1)=360*pi/180;
                MaxPositionJoint(2)=360*pi/180;
                MaxPositionJoint(3)=360*pi/180;
                MaxPositionJoint(4)=360*pi/180;
                MaxPositionJoint(5)=360*pi/180;
                MaxPositionJoint(6)=360*pi/180;
                Link(1)=0.128;
                Link(2)=0.176;
                Link(3)=0.612;
                Link(4)=0.128;
                Link(5)=0.572;
                Link(6)=0.116;
                Link(7)=0.116;
                Link(8)=0.092;
            elseif strcmp(name,'LBR_iiwa_14_R820')==1
                MinPositionJoint(1)=-170*pi/180;
                MinPositionJoint(2)=-120*pi/180;
                MinPositionJoint(3)=-170*pi/180;
                MinPositionJoint(4)=-120*pi/180;
                MinPositionJoint(5)=-170*pi/180;
                MinPositionJoint(6)=-120*pi/180;
                MinPositionJoint(7)=-175*pi/180;
                MaxPositionJoint(1)=170*pi/180;
                MaxPositionJoint(2)=120*pi/180;
                MaxPositionJoint(3)=170*pi/180;
                MaxPositionJoint(4)=120*pi/180;
                MaxPositionJoint(5)=170*pi/180;
                MaxPositionJoint(6)=120*pi/180;
                MaxPositionJoint(7)=175*pi/180;
                Link(1)=0.360;
                Link(2)=0.42;
                Link(3)=0.4;
                Link(4)=0.126;
            elseif strcmp(name,'Kuka_LBRiisy')==1
                MinPositionJoint(1)=-185*pi/180;
                MinPositionJoint(2)=-230*pi/180;
                MinPositionJoint(3)=-150*pi/180;
                MinPositionJoint(4)=-180*pi/180;
                MinPositionJoint(5)=-110*pi/180;
                MinPositionJoint(6)=-220*pi/180;
                MaxPositionJoint(1)=185*pi/180;
                MaxPositionJoint(2)=50*pi/180;
                MaxPositionJoint(3)=150*pi/180;
                MaxPositionJoint(4)=180*pi/180;
                MaxPositionJoint(5)=110*pi/180;
                MaxPositionJoint(6)=220*pi/180;
                Link(1)=0.300;
                Link(2)=0.590;
                Link(3)=0.532;
                Link(4)=0.178;
            else
                disp('ERROR: No arm defined');
                error = 1;
                return;
            end

            if strcmp(obj.hand_name,'RG2')==1
                DistanceHand = 0.132+0.030;
                %ponta dos dedos aberto 0.174
                %ponta dos dedos fechados 0.213
            elseif strcmp(obj.hand_name,'BarrettHand')==1
                DistanceHand = 0.0755+0.018;
                %Palma da mão 0.0755
            else
                disp('ERROR: No hand defined');
                error = 1;
                return;
            end

            obj.MinJointPos = MinPositionJoint;
            obj.MaxJointPos = MaxPositionJoint;
        end

        %Function that allows you to get the position of the arm
        function [error,armPosition]=get_robot_position(obj)
            error = 0;
            [res,armPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end
        end

        %Function that allows you to set the joints from the arm
        function [error] = set_joints(obj, armJoints) %armJoints (1-7) in rad
            error = 0;
            for i=1:1:obj.number_of_joints
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)
                    disp('ERROR: value outside the limits of the joints');
                    error=1;
                    return;
                else
                    res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.jointHandle{i},armJoints(i), obj.vrep.simx_opmode_oneshot);

                    if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                        disp('ERROR: Failed sending Joint value!');
                        error=1;
                        return;
                    end
                end
            end
        end

        %Function that allows you to set the joints from the arm
        function [error] = set_joints_defPos(obj) %armJoints (1-7) in rad
            error = 0;
            armJoints(1)=-2.2985;
            armJoints(2)=-1.1855;
            armJoints(3)=0.8030;
            armJoints(4)=2.0662;
            armJoints(5)=-0.2730;
            armJoints(6)=1.3041;
            armJoints(7)=2.4532;
            for i=1:1:obj.number_of_joints
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)
                    disp('ERROR: value outside the limits of the joints');
                    error=1;
                    return;
                else
                    res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.jointHandle{i},armJoints(i), obj.vrep.simx_opmode_oneshot);

                    if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                        disp('ERROR: Failed sending Joint value!');
                        error=1;
                        return;
                    end
                end
            end
        end

        %Function that allows you to get the joints values from the arm
        %armJoints - joint values for arm (in rad)
        function [error,armJoints] = get_joints(obj)
            % Get pose
            % x and y in cm
            % phi in rad
            error = 0;
            for i = 1:obj.number_of_joints
                %Read from joints
                [res,armJoints(i)]=obj.vrep.simxGetJointPosition(obj.clientID, obj.jointHandle{i},obj.vrep.simx_opmode_buffer);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed read Joint value!');
                    error = 1;
                    return;
                end
            end
        end

        %Functions that allows close the hand
        function error = close_hand(obj)
            error = 0;
            if strcmp(obj.hand_name,'RG2')==1
                value = 0;      %Close Hand = 0
                [res] = obj.vrep.simxSetIntegerSignal(obj.clientID, 'RG2_open', value, obj.vrep.simx_opmode_oneshot);
            else
                value = 1;      %Close Hand = 1
                [res] = obj.vrep.simxSetIntegerSignal(obj.clientID, 'BarrettHandClose', value, obj.vrep.simx_opmode_oneshot);
            end
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed close hand!');
                error=1;
                return;
            end
        end

        %Functions that allows open the hand
        function error = open_hand(obj)
            error = 0;
            if strcmp(obj.hand_name,'RG2')==1
                value = 1;  %Open Hand = 1
                [res] = obj.vrep.simxSetIntegerSignal(obj.clientID, 'RG2_open', value, obj.vrep.simx_opmode_oneshot);
            else
                value = 0;  %Open Hand = 0
                [res] = obj.vrep.simxSetIntegerSignal(obj.clientID, 'BarrettHandClose', value, obj.vrep.simx_opmode_oneshot);
            end
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed open hand!');
                error = 1;
                return;
            end
        end
    end
end


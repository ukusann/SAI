classdef kuka_interface < handle
    %kuka_interface CoppeliaSim connection interface for a youBot Robot
    %   The class provides the communication service and information
    %   pre-processing for a MATLAB based control interface with a
    %   developed robot in CoppeliaSim.
    %   2023/10/20
    %   % Copyright (C) 2023, by Lu�s Louro
    %   Author: Lu�s Louro  e-mail: luislouro@algoritmi.uminho.pt
    
    properties (Access = private)
        vrep
        clientID
        motor_front_left_handle
        motor_front_right_handle
        motor_rear_left_handle
        motor_rear_right_handle
        S1_VS1_handle
        S1_VS2_handle
        S2_VS1_handle
        S2_VS2_handle
        
        S1_VS1_ignore_steps
        S1_VS2_ignore_steps
        S2_VS1_ignore_steps
        S2_VS2_ignore_steps
        
        S1_maxDist
        S2_maxDist
        
        unknown_sector_method
        
        world_ref_handle
        
        robot_shape_handle
        omni_ref_handle
        obstacles_lim
        nr_sectors
        theta_obs
        x_sensor_final
        y_sensor_final
        b_sensor_i
        in_veh_dist
        robot_distance
        min_sensor
        max_sensor
        L
        W
        H
        displc
        %distance from steering wheel to the vehicle rotation center

        
        m0      %transform from WORLD frame to frame 0(robot rear axle center)
        m1      %transform from WORLD frame to frame 1(S1 VS1 frame)
        m2      %transform from WORLD frame to frame 2(S1 VS2 frame)
        m3      %transform from WORLD frame to frame 3(S2 VS1 frame)
        m4      %transform from WORLD frame to frame 4(S2 VS2frame)
        
        m1_pos
        m1_ori
        m2_pos
        m2_ori
        m3_pos
        m3_ori
        m4_pos
        m4_ori
        
        m01  %transform from frame 0 to frame 1
        m02  %transform from frame 0 to frame 2
        m03  %transform from frame 0 to frame 3
        m04  %transform from frame 0 to frame 4
    end
    
    properties
        name
        LONGITUDINAL_MAX_LINEAR_SPEED = 3.6;
        LATERAL_MAX_LINEAR_SPEED = 2.0;
        wheel_radius = (0.25)/2;
        lx = 0.28;
        ly = 0.1825;
        auxDataS1VS1
        auxDataS1VS2
        auxDataS2VS1
        auxDataS2VS2
        DEFAULT_DETECTION_RANGE = 2*pi;
    end
    
    methods
        % Create a connection handler to a SAMU Robot
        % sim_obj The simulation connection object
        function [obj,erro] = kuka_interface(sim_obj)
            erro = 0;
           
            [obj.vrep, obj.clientID] = sim_obj.get_connection();
            
            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                erro = 1;
                error(msg)
            end
            
            %% Get objects handles
            obj.world_ref_handle = -1;

            [res, obj.robot_shape_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'robot_bounding_box', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting robot_bounding_box handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.motor_front_left_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, './Omnirob_FLwheel_motor', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting front left motor handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.motor_front_right_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./Omnirob_FRwheel_motor', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting front right motor handle ';
                erro = 1;
                error(msg);
            end

            [res, obj.motor_rear_left_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, './Omnirob_RLwheel_motor', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting rear left motor handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.motor_rear_right_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./Omnirob_RRwheel_motor', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting rear right motor handle ';
                erro = 1;
                error(msg);
            end
			
	
            [res, obj.S1_VS1_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./SICK_S300_sensor1_front', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting SICK_S300_sensor1 handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.S1_VS2_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./SICK_S300_sensor2_front', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting SICK_S300_sensor2 handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.S2_VS1_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./SICK_S300_sensor1_rear', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting SICK_S300_sensor1#0 handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.S2_VS2_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./SICK_S300_sensor2_rear', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting SICK_S300_sensor2#0 handle ';
                erro = 1;
                error(msg);
            end
            
            [res, obj.omni_ref_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'Omnirob_ref', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting omni_ref handle ';
                erro = 1;
                error(msg);
            end
     
            %% Parameters
            % Get vehicle size:
            localxMinMax = [0,0];
            localyMinMax = [0,0];
            localzMinMax = [0,0];
            [res, localxMinMax(1)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_min_x, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                erro = 1;
                error(msg);
            end
            [res, localyMinMax(1)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_min_y, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                erro = 1;
                error(msg);
            end
            [res, localzMinMax(1)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_min_z, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                erro = 1;
                error(msg);
            end
            [res, localxMinMax(2)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_max_x, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                erro = 1;
                error(msg);
            end
            [res, localyMinMax(2)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_max_y, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to robot size parameter';
                erro = 1;
                error(msg);
            end
            [res, localzMinMax(2)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_max_z, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to robot size parameter';
                erro = 1;
                error(msg);
            end
            xSize = localxMinMax(2) - localxMinMax(1);
            ySize = localyMinMax(2) - localyMinMax(1);
            zSize = localzMinMax(2) - localzMinMax(1);

            % get displacement in xx
           [res, pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot_shape_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
           if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to robot size parameter';
                erro = 1;
                error(msg);
            end

            center_xx_disp = abs(pos(1));

            % ensure x size is the Len of the vehicle
            if xSize < ySize
                temp = ySize;
                ySize = xSize;
                xSize = temp;
            end    
            
            obj.L = xSize;      %in m
            obj.W = ySize;      %in m
            obj.H = zSize;      %in m
            
            obj.displc = center_xx_disp;
            
            [res, obj.S1_maxDist] = obj.vrep.simxGetFloatSignal(obj.clientID,'SICK_S300_rear_maxDist', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get max scan distance for SICK_S300 rear';
                erro = 1;
                error(msg);
            end
            
            [res, obj.S2_maxDist] = obj.vrep.simxGetFloatSignal(obj.clientID,'SICK_S300_front_maxDist', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get max scan distance for SICK_S300 front';
                erro = 1;
                error(msg);
            end
            
            if obj.S1_maxDist ~= obj.S2_maxDist
                msg = 'Scan distance for laser scanner are not equal. Undefined behavior!';
                warning(msg);
            end
        
            [res, obj.m1_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.S1_VS1_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            [res, obj.m1_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.S1_VS1_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            [res, obj.m2_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.S1_VS2_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1; 
                error(msg);
            end
            
            [res, obj.m2_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.S1_VS2_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            [res, obj.m3_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.S2_VS1_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            [res, obj.m3_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.S2_VS1_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            [res, obj.m4_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.S2_VS2_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            [res, obj.m4_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.S2_VS2_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                erro = 1;
                error(msg);
            end
            
            % get ignore steps for laser scanners
            [res, packed_ignore_steps] = obj.vrep.simxGetStringSignal(obj.clientID, 'SICK_S300_front_VS1', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get ignore steps for SICK_S300_VS1';
                erro = 1;
                error(msg);
            end
            
            ignore_steps = obj.vrep.simxUnpackInts(packed_ignore_steps);
            if ignore_steps(1) ~= 0
                obj.S1_VS1_ignore_steps = ignore_steps;
            end
            
            [res, packed_ignore_steps] = obj.vrep.simxGetStringSignal(obj.clientID, 'SICK_S300_front_VS2', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get ignore steps for SICK_S300_VS2';
                erro = 1;
                error(msg);
            end
            
           ignore_steps = obj.vrep.simxUnpackInts(packed_ignore_steps);
            if ignore_steps(1) ~= 0
                obj.S1_VS2_ignore_steps = ignore_steps;
            end
            
            [res, packed_ignore_steps] = obj.vrep.simxGetStringSignal(obj.clientID, 'SICK_S300_rear_VS1', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get ignore steps for SICK_S300#0_VS1';
                erro = 1;
                error(msg);
            end
            
            ignore_steps = obj.vrep.simxUnpackInts(packed_ignore_steps);
            if ignore_steps(1) ~= 0
                obj.S2_VS1_ignore_steps = ignore_steps;
            end
            
            [res, packed_ignore_steps] = obj.vrep.simxGetStringSignal(obj.clientID, 'SICK_S300_rear_VS2', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get ignore steps for SICK_S300#0_VS2';
                erro = 1;
                error(msg);
            end
            
            ignore_steps = obj.vrep.simxUnpackInts(packed_ignore_steps);
            if ignore_steps(1) ~= 0
                obj.S2_VS2_ignore_steps = ignore_steps;
            end
        
            
            %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for simulation time';
                erro = 1;
                error(msg);
            end
            
            % Vision sensors for obstacles
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS1_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for S1_VS1_handle';
                erro = 1;
                error(msg);
            end
            
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS2_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for S1_VS2_handle';
                erro = 1;
                error(msg);
            end
            
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS1_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for S2_VS1_handle';
                erro = 1;
                error(msg);
            end
            
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS2_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for S2_VS2_handle';
                erro = 1;
                error(msg);
            end
            
            % Vehicle pose
            [res , ~] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for robot position';
                erro = 1;
                error(msg);
            end
            [res , ~] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for robot orientation';
                erro = 1;
                error(msg);
            end

            % Get Joints velocities
            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front left motor';
                erro = 1;
                error(msg);
            end

            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front right motor';
                erro = 1;
                error(msg);
            end

            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear left motor';
                erro = 1;
                error(msg);
            end

            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear right motor';
                erro = 1;
                error(msg);
            end

            
        end

        function [erro, vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = Kinematics_vehicle(obj, wrobot, v_roboty, v_robotx)
            erro = 0;

            v_robot_x = v_robotx*0.01;     %convert cm/s to m/s
            v_robot_y = v_roboty*0.01;     %convert cm/s to m/s

            if v_robot_x > obj.LONGITUDINAL_MAX_LINEAR_SPEED
                v_robot_x = obj.LONGITUDINAL_MAX_LINEAR_SPEED;
            end

            if v_robot_y > obj.LATERAL_MAX_LINEAR_SPEED
                v_robot_y = obj.LATERAL_MAX_LINEAR_SPEED;
            end

            vel_front_left=(v_robot_x-v_robot_y-(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
            vel_front_right=(v_robot_x+v_robot_y+(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
            vel_rear_left=(v_robot_x+v_robot_y-(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
            vel_rear_right=(v_robot_x-v_robot_y+(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;   
        end
         
        function  [erro, x, y, phi] = set_velocity(obj, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)
            erro = 0;
            %% Set vehicle velocity    
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_front_left_handle, vel_front_left, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end
            
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_front_right_handle, vel_front_right, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end

            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_rear_left_handle, vel_rear_left, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end

            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_rear_right_handle, vel_rear_right, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end
            if erro==0
                % Get pose
                [erro, x, y, phi] = get_vehicle_pose(obj);
                %convers�o para cm j� realizada
            end
            
        end
        
        function [erro, x, y, phi] = get_vehicle_pose(obj)
            erro = 0;
            % Get pose
            [res, robot_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'Failed getting robot position!';
                erro = 1;
                error(msg);
            end
            
            [res, robot_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'Failed getting robot orientation!';
                erro = 1;
                error(msg);
            end
            
            x = robot_pos(1)*100;   %cm
            y = robot_pos(2)*100;   %cm
            phi = robot_ori(3);     %rad/s

        end
        
         function [erro, x, y, phi] = get_vehicle_pose2pi(obj)
             erro = 0;
             [erro, x, y, phi] = get_vehicle_pose(obj);
             phi = rem(phi + 2*pi, 2*pi);
         end
         
         % Get current power wheel velocities:
         % v_steer current linear velocity of power wheel
         % theta_steer current angular position of steering
         % w_theta_steer current velocity of steering
         function [erro,vel_front_left, vel_front_right, vel_rear_left, vel_rear_right] = get_current_state(obj)
            erro = 0;
            % Get Joints velocities
            [res , vel_front_left] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front left motor';
                erro = 1;
                error(msg);
            end

            [res , vel_front_right] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front right motor';
                erro = 1;
                error(msg);
            end

            [res , vel_rear_left] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear left motor';
                erro = 1;
                error(msg);
            end

            [res , vel_rear_right] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear right motor';
                erro = 1;
                error(msg);
            end
         end

        function [erro,rob_W,rob_L,theta_obs] = get_RobotCharacteristics(obj)
            erro = 0;
            % Setup number of sectors for obstacles detection.
            %   Set the method of dealing with occluded sectors (either 'inf' or 'max')
            Nsensors = 29;              
            range = (180+30.0)*pi/180;  
            [erro, theta_obs,delta_obs,in_veh_dist] = setup_obstacles(obj,Nsensors, range, 'inf');
            
            if erro==0

                [erro,rob_W, rob_L, rob_H] = get_robot_dimension(obj);
                %Convertido para cm no get_robot_dimension
            end
        end

        function [erro, theta_obs, delta_theta_obs, in_veh_dist] = setup_obstacles(obj, nr_sectors, detection_range, occluded_method)
            erro = 0;
            obj.unknown_sector_method = inf;
            if nargin >= 3
                if ~isfloat(detection_range)
                    erro = 1;
                    error('detection_range paramter must be a double or single value, e.g. 2*pi. Leave empty for default');
                end
                detection_range_ = detection_range;
            else
                detection_range_ = obj.DEFAULT_DETECTION_RANGE;
            end
            if nargin >= 3
                if strcmp(occluded_method, 'inf')
                     obj.unknown_sector_method = inf;
                elseif strcmp(occluded_method, 'max')
                     obj.unknown_sector_method = obj.S1_maxDist * 1.1;
                else
                    erro = 1;
                    error(['unknown argument: ' occluded_method]);
                end
            end
            
            % enforce some options
            if nr_sectors <= 0 
                erro = 1;
                error('Invalid number of sectors. Value should be higher than 0');
            end
            if rem(nr_sectors, 2) == 0
                erro = 1;
                error('Invalid number of sectors. Value should be odd');
            end
           
            % angular difference between to sectors
            % consider detetion range to be the full circle, or given
            % parameter
            %detection_range_ = 2*pi;   % see parameter and default value
            delta_theta_obs = detection_range_/nr_sectors;
            obj.nr_sectors = nr_sectors;

            %store sectors angles limits (start angle and end angle )
            obj.obstacles_lim = -detection_range_/2.0 + (0:1:nr_sectors) * delta_theta_obs;
            obj.obstacles_lim = obj.obstacles_lim';
            
            % sectors center
            theta_obs = -detection_range_/2.0 + delta_theta_obs/2.0 + (0:1:nr_sectors-1) * delta_theta_obs;
            theta_obs = round(theta_obs,6)';
            obj.theta_obs = theta_obs;
            
            obj.x_sensor_final = 30*cos(obj.theta_obs);
            obj.y_sensor_final = 30*sin(obj.theta_obs);
            obj.b_sensor_i = 0;
            
            %precompute transform matrices     
           % R = rotx(alpha) * roty( beta ) * rotz(gamma);  
            R = rotx(obj.m1_ori(1)) * roty( obj.m1_ori(2) ) * rotz(obj.m1_ori(3)); 
            obj.m01 = [ R obj.m1_pos'; 0 0 0 1];
            
            R = rotx(obj.m2_ori(1)) * roty( obj.m2_ori(2) ) * rotz(obj.m2_ori(3)); 
            obj.m02 = [ R obj.m2_pos'; 0 0 0 1];
            
            R = rotx(obj.m3_ori(1)) * roty( obj.m3_ori(2) ) * rotz(obj.m3_ori(3)); 
            obj.m03 = [ R obj.m3_pos'; 0 0 0 1];
            
            R = rotx(obj.m4_ori(1)) * roty( obj.m4_ori(2) ) * rotz(obj.m4_ori(3)); 
            obj.m04 = [ R obj.m4_pos'; 0 0 0 1];
            
            % compute distance from center to vehicle's margins
            % compute only for one side, the other can be mirrored
            psi = theta_obs( theta_obs < 0 );
            
            % 'a' stores the distances from center of rotation
            % perpendicular to robot margins
            % 'theta' are the angles between theta_obs and the same
            % perpendicular
            % 'd' is the effective distance from center of rotation to
            % robot margin, at the angle given by the theta_obs
            a = zeros(size(psi));
            theta = a;
            %d = a;
            
            % rear perpendicular distance
            a1 = obj.L/2 - obj.displc;
            % side perpendicular distance
            a2 = obj.W/2;
            
            % angle of the rear right vehicle corner
            alfa1 = atan(a2/a1);
            beta1 = - (pi - alfa1);
            
            % angle of the front right corner
            alfa2 = atan( (obj.L/2 + obj.displc ) / a2);
            beta2 = - (pi/2 - alfa2);
            
            % Indexs of the several branches
            cond_1_idx = psi < beta1;
            cond_2_idx = (beta1 <= psi) & (psi < beta2);
            cond_3_idx = psi >= beta2;
            
            % Fill up 'a'
            a(cond_1_idx) = obj.L/2 - obj.displc; %same as a1
            a(cond_2_idx) = obj.W/2;    %same as a2
            a(cond_3_idx) = obj.L/2 + obj.displc;
            
            % compute thetas
            theta(cond_1_idx) = pi - abs(psi(cond_1_idx));
            theta(cond_2_idx) = abs( abs(psi(cond_2_idx)) - pi/2 );
            theta(cond_3_idx) = abs( psi(cond_3_idx) );
            
            % finally 'd'
            d = a./cos(theta);
            
            % properly fill the array to use in future computations
            obj.in_veh_dist = zeros(size(obj.theta_obs));
            % right side
            
            
            aux = (obj.nr_sectors-1)/2;
            
            obj.in_veh_dist(1:aux) = d;
            
            % front center
            obj.in_veh_dist((obj.nr_sectors-1)/2 + 1) = obj.L/2 + obj.displc;
            
            % right side
            obj.in_veh_dist((obj.nr_sectors-1)/2 + 2 : end) = flipud(d);
            
            in_veh_dist = obj.in_veh_dist;

            obj.robot_distance = in_veh_dist;

            obj.min_sensor = 2;
            obj.max_sensor = nr_sectors-1;
            
%             value_max = max(in_veh_dist);
%             
%             for i=1:nr_sectors
%                 if theta_obs(i)>-45*pi/180 && theta_obs(i)<45*pi/180
%                     in_veh_dist(i)=value_max;
%                 end
%             end
%             
%             obj.in_veh_dist = in_veh_dist;
            
        end

        function erro = trigger_obstacles(obj)
            erro = 0;
            [res , ~, obj.auxDataS1VS1, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS1_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting sensor data for S1_VS1! \n';
                erro = 1;
                error(msg);
            end
            [res , ~, obj.auxDataS1VS2, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS2_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting sensor data for S1_VS2! \n';
                erro = 1;
                error(msg);
            end
            [res , ~, obj.auxDataS2VS1, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS1_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting sensor data for S2_VS1! \n';
                erro = 1;
                error(msg);
            end
            [res , ~, obj.auxDataS2VS2, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS2_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting sensor data for S2_VS2! \n';
                erro = 1;
                error(msg);
            end
        end
        
        function [erro,distances] = get_DistanceSensorAquisition(obj, show_obs, show_limits)
            erro = 0;
            show_obstacles = false;
            show_sectors_limits = false;
            if nargin >= 2
                if show_obs ~= 0
                    show_obstacles = true;
                end
            end
            if nargin >= 3
                if show_limits ~= 0
                    show_sectors_limits = true;
                end
            end
            
            % Inf was returned when a sector was ocluded by the physical disposition of the laser scanners in the vehicle
            % and some obstacle did not allow the scanner to detect obstacles in other sectors.
            % NOTICE: with this configuration, there is no way to tell whether there is an obstacle or not. We can't know.

            % For now, a finit value will be returned. Nevertheless, behavior might change in future (returning again inf) if we
            % see that we must know that in such sectors it is impossible to tell whether there is an obstacle or not.

            distances = ones(obj.nr_sectors, 1) * obj.unknown_sector_method;
                % times 1.1 to ensure distance higher than the laser
                % scanner reach
            
            % need for error check: all sensors must be 1 pixel height
            % all sensors must have same width
     
            % size of incoming data, i.e. the number of steps provided by
            % sensors
            vs_size = obj.auxDataS1VS1(1);        
            
            % get the obstacle position (relative to its own sensor)
            vect1 = reshape(obj.auxDataS1VS1(3:end), [4,vs_size]);
            vect2 = reshape(obj.auxDataS1VS2(3:end), [4,vs_size]);
            vect3 = reshape(obj.auxDataS2VS1(3:end), [4,vs_size]);
            vect4 = reshape(obj.auxDataS2VS2(3:end), [4,vs_size]);
            
            % positions [x y z 1]' for all obstacles (merge of sensors)
            pos = zeros(4, vs_size*4);
            
            % compute the obstacle position relative to rear axle
%              for i = 1:vs_size
%                  pos( :, (i-1)*4 + 1)  = obj.m01 * [vect1(1:3, i);1];
%                  pos( :, (i-1)*4 + 2)  = obj.m02 * [vect2(1:3, i);1];
%                  pos( :, (i-1)*4 + 3)  = obj.m03 * [vect3(1:3, i);1];
%                  pos( :, (i-1)*4 + 4)  = obj.m04 * [vect4(1:3, i);1];
%              end

             for i = 1:vs_size
                 pos( :, i)  = obj.m01 * [vect1(1:3, i);1];
                 pos( :, i+vs_size)  = obj.m02 * [vect2(1:3, i);1];
                 pos( :, i+2*vs_size)  = obj.m03 * [vect3(1:3, i);1];
                 pos( :, i+3*vs_size)  = obj.m04 * [vect4(1:3, i);1];
             end
  
            % convert to polar coordinates
            [theta, rho] = cart2pol(pos(1,:), pos(2,:));

            % ensure values are within [-pi pi]
            theta = theta - 2*pi*floor( (theta+pi)/(2*pi) );
         
%             % ensure constante obstacle steps are ignored
%             rho( (obj.S1_VS1_ignore_steps-1)*4 + 1 ) = obj.unknown_sector_method;
%             rho( (obj.S1_VS2_ignore_steps-1)*4 + 2 ) = obj.unknown_sector_method;
%             rho( (obj.S2_VS1_ignore_steps-1)*4 + 3 ) = obj.unknown_sector_method;
%             rho( (obj.S2_VS2_ignore_steps-1)*4 + 4 ) = obj.unknown_sector_method;
           
            middle = floor(size(theta,2)/2);
            
            IgnoredStepsInitS1 = size(obj.S1_VS1_ignore_steps,2);
            IgnotedStepsEndS1 = size(obj.S1_VS2_ignore_steps,2);
            
            IgnoredStepsInitS2 = size(obj.S2_VS1_ignore_steps,2);
            IgnotedStepsEndS2 = size(obj.S2_VS2_ignore_steps,2);

            
             % map each obstacle to the pre-define sectors
            for i = IgnoredStepsInitS1+2:middle-IgnotedStepsEndS1
                if(theta(i-1)<theta(i))
                    angle_min = theta(i-1);
                    angle_max = theta(i);
                else
                    angle_min = theta(i);
                    angle_max = theta(i-1);
                end
                
                Index = find(obj.theta_obs<angle_max & obj.theta_obs>angle_min);
                
                for k=1:size(Index)
                    j = Index(k);
                    if(pos(1,i)-pos(1,i-1)==0) %recta vertical do obst�culo x1-x0=0
                        m_sensor_i= (obj.y_sensor_final(j)-0)/(obj.x_sensor_final(j)-0);
                        b_recta_i = pos(1,i);

                        x_interception = b_recta_i;
                        y_interception = (m_sensor_i*x_interception)+obj.b_sensor_i;
                    elseif(obj.x_sensor_final(j)-0==0) %sensor da frente em que x1-x0=0
                        m_recta_i = (pos(2,i)-pos(2,i-1))/(pos(1,i)-pos(1,i-1));
                        b_recta_i = pos(2,i)-m_recta_i*pos(1,i);

                        x_interception = 0;
                        y_interception = (m_recta_i*x_interception)+b_recta_i;    
                    else  %outros casos
                        m_sensor_i= (obj.y_sensor_final(j)-0)/(obj.x_sensor_final(j)-0);
                        m_recta_i = (pos(2,i)-pos(2,i-1))/(pos(1,i)-pos(1,i-1));
                        b_recta_i = pos(2,i)-m_recta_i*pos(1,i);

                        x_interception = (b_recta_i-obj.b_sensor_i)/(m_sensor_i-m_recta_i);
                        y_interception = (m_sensor_i*x_interception)+obj.b_sensor_i;

                    end
                    distance = sqrt((x_interception)^2+(y_interception)^2);

                    if distance < distances(j)
                        distances(j) = distance;
                    end
                end
            end
            
            IgnoredStepsInitS2 = middle + 342;
            
            
            for i = IgnoredStepsInitS2+2:size(theta,2)-IgnotedStepsEndS2
                if(theta(i-1)<theta(i))
                    angle_min = theta(i-1);
                    angle_max = theta(i);
                else
                    angle_min = theta(i);
                    angle_max = theta(i-1);
                end
                
                Index = find(obj.theta_obs<angle_max & obj.theta_obs>angle_min);
                
                for k=1:size(Index)
                   j= Index(k);
                   if(pos(1,i)-pos(1,i-1)==0) %recta vertical do obst�culo x1-x0=0
                        m_sensor_i= (obj.y_sensor_final(j)-0)/(obj.x_sensor_final(j)-0);
                        b_recta_i = pos(1,i);

                        x_interception = b_recta_i;
                        y_interception = (m_sensor_i*x_interception)+obj.b_sensor_i;
                    elseif(obj.x_sensor_final-0==0) %sensor da frente em que x1-x0=0
                        m_recta_i = (pos(2,i)-pos(2,i-1))/(pos(1,i)-pos(1,i-1));
                        b_recta_i = pos(2,i)-m_recta_i*pos(1,i);

                        x_interception = 0;
                        y_interception = (m_recta_i*x_interception)+b_recta_i;    
                    else  %outros casos
                        m_sensor_i= (obj.y_sensor_final(j)-0)/(obj.x_sensor_final(j)-0);
                        m_recta_i = (pos(2,i)-pos(2,i-1))/(pos(1,i)-pos(1,i-1));
                        b_recta_i = pos(2,i)-m_recta_i*pos(1,i);

                        x_interception = (b_recta_i-obj.b_sensor_i)/(m_sensor_i-m_recta_i);
                        y_interception = (m_sensor_i*x_interception)+obj.b_sensor_i; 
                    end
                    distance = sqrt((x_interception)^2+(y_interception)^2);

                    if distance < distances(j)
                        distances(j) = distance;
                    end
                end
            end
     
            
            if show_obstacles 
                [X,Y] = pol2cart(obj.theta_obs, distances);
                [X1,Y1] = pol2cart(obj.theta_obs, obj.in_veh_dist);
                       
                dat = [X,Y,X1,Y1,zeros(obj.nr_sectors,1)];
                dat = reshape(dat', 1, numel(dat));
                [res, ~, ~, ~, ~] = obj.vrep.simxCallScriptFunction(obj.clientID,'SICK_S300_front', 1, 'show_sectors_lines_function',[],dat',[],[],obj.vrep.simx_opmode_oneshot);
                if (res~=obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                     error('show_sectors_lines_function! \n');
                end
            end
            
            if show_sectors_limits
                [X1,Y1] = pol2cart(obj.obstacles_lim, ones(numel(obj.obstacles_lim),1)*30);

                dat = [X1,Y1,zeros(numel(obj.obstacles_lim),1)];
                dat = reshape(dat', 1, numel(dat));
                [res, ~, ~, ~, ~] = obj.vrep.simxCallScriptFunction(obj.clientID,'SICK_S300_front', 1, 'show_sectors_lim_lines_function',[],dat',[],[],obj.vrep.simx_opmode_oneshot);
                if (res~=obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                     error('show_sectors_lim_lines_function! \n');
                end
            end
            
            % return the distances from vehicle bounding box
            distances = (distances - obj.in_veh_dist)*100;        %cm
            % ensure distances are not smaller than vehicle itself
            distances( distances < 0 ) = 0;
        end
        
        function erro = show_robot_bounding_box(obj, show)
            erro = 0;
            show_box = 1;
            if nargin >= 2
                if isinteger(show) || isa(show,'logical')
                    if show > 0
                        show_box = 1;
                    else
                        show_box = 0;
                    end
                else
                    erro = 1;
                    error('show parameter is invalid');
                end
            end
            
            [res, ~, ~, ~, ~] = obj.vrep.simxCallScriptFunction(obj.clientID,'kuka', 1, 'show_bounding_box_function',show_box, [], [], [], obj.vrep.simx_opmode_oneshot);
            if (res~=obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                erro = 1; 
                error('show_bounding_box_function!');
            end
        end
        
        function erro = show_robot_path(obj, show)
            erro = 0;
            % show_path = 1;
            % if nargin >= 2
            %     if isinteger(show) || isa(show,'logical')
            %         if show > 0
            %             show_path = 1;
            %         else
            %             show_path = 0;
            %         end
            %     else
            %          error('show parameter is invalid');
            %     end
            % end
            % 
            % [res, ~, ~, ~, ~] = obj.vrep.simxCallScriptFunction(obj.clientID,'Path', 1, 'show_path',show_path, [], [], [], obj.vrep.simx_opmode_oneshot);
            % if (res~=obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
            %      error('show_robot_path_function!');
            % end
        end
        
        % Gets robot dimension
        % W robot width
        % L robot lenght
        % H robot height
        function [erro, W, L, H] = get_robot_dimension(obj)
            erro = 0;
            W = obj.W * 100;    %cm
            L = obj.L * 100;    %cm
            H = obj.H * 100;    %cm
        end
        
        function terminate(obj)
           %% Stop data streaming
            
            % Vision sensors for obstacles
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS1_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S1_VS1_handle';
                error(msg);
            end
            
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS2_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S1_VS2_handle';
                error(msg);
            end
            
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS1_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S2_VS1_handle';
                error(msg);
            end
            
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS2_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S2_VS2_handle';
                error(msg);
            end
            
        end
    end
    
end


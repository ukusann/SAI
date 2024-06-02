%   simulator_interface.m
%   Interface to simulator in CoppeliaSim simulator
%   2024/02/19
%   % Copyright (C) 2024
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef simulator_interface < handle

    properties (Access = private)
        vrep
        clientID
        TargetHandle        % handle for the target object (48 different position in the shelf)
        TargetNames         % list of target names
        ObjectHandle        % handle for the object (6 differents)
        ObjectNames         % list of object names
        ConveyorBeltHandle  % handle for the conveyor object
    end

    properties
        TARGET_Number       % Number of targets
        OBJECT_Number       % Number of objects

    end

    methods
        % Create a connection handler to a SAMU Simulation scene
        % remote_ip_address The remote IP address
        % remote_port The remote port
        function [obj,error] = simulator_interface(remote_ip_address, remote_port)
            error = 0;
            ip_address = '127.0.0.1';
            port = 19997;
            if nargin >= 1
                % TODO: Add safe guarding for well formated ip
                ip_address = remote_ip_address;
            end
            if nargin >= 2
                % TODO: Check for valid number
                port = remote_port;
            end

            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections

            obj.clientID = obj.vrep.simxStart(ip_address, port, true, true, 5000, 5);

            if (obj.clientID > -1)
                fprintf('\n');
                disp('INFO: Connected successfully to CoppeliaSim!');
                fprintf('\n');

            else
                clear obj;
                disp('ERROR: Failed connecting to remote API server. Ensure CoppeliaSim is running and a scenario is loaded.');
                error = 1;
                return;
            end

            %% Synchronous mode
            obj.vrep.simxSynchronous(obj.clientID, true); % Enable the synchronous mode
            obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);

            obj.vrep.simxSynchronousTrigger(obj.clientID); % Trigger next simulation step (Blocking function call)

            % The first simulation step is now being executed
            %ensure it is finished so we can access signals
            obj.vrep.simxGetPingTime(obj.clientID);

            % Six different objects
            obj.OBJECT_Number = 6;
            obj.ObjectNames{1} = 'lata_salsichas_1';
            obj.ObjectNames{2} = 'lata_salsichas_2';
            obj.ObjectNames{3} = 'lata_salsichas_3';
            obj.ObjectNames{4} = 'lata_cogumelos_1';
            obj.ObjectNames{5} = 'lata_cogumelos_2';
            obj.ObjectNames{6} = 'lata_cogumelos_3';

            

            %Get Targets Handle
            for a=1:obj.OBJECT_Number
                object_name = ['/',obj.ObjectNames{a}];
                [res, obj.ObjectHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, obj.ObjectNames{a}, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting object handle');
                    error = 1;
                    return;
                end
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.ObjectHandle{a},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting object position information');
                    error = 1;
                    return;
                end
            end

            % 48 different targets
            obj.TARGET_Number = 48;
            obj.TargetNames{1} = 'Prat1_frente1';
            obj.TargetNames{2} = 'Prat1_frente2';
            obj.TargetNames{3} = 'Prat1_frente3';
            obj.TargetNames{4} = 'Prat1_frente4';
            obj.TargetNames{5} = 'Prat1_frente5';
            obj.TargetNames{6} = 'Prat1_frente6';
            obj.TargetNames{7} = 'Prat1_frente7';
            obj.TargetNames{8} = 'Prat1_frente8';
            obj.TargetNames{9} = 'Prat2_frente1';
            obj.TargetNames{10} = 'Prat2_frente2';
            obj.TargetNames{11} = 'Prat2_frente3';
            obj.TargetNames{12} = 'Prat2_frente4';
            obj.TargetNames{13} = 'Prat2_frente5';
            obj.TargetNames{14} = 'Prat2_frente6';
            obj.TargetNames{15} = 'Prat2_frente7';
            obj.TargetNames{16} = 'Prat2_frente8';
            obj.TargetNames{17} = 'Prat3_frente1';
            obj.TargetNames{18} = 'Prat3_frente2';
            obj.TargetNames{19} = 'Prat3_frente3';
            obj.TargetNames{20} = 'Prat3_frente4';
            obj.TargetNames{21} = 'Prat3_frente5';
            obj.TargetNames{22} = 'Prat3_frente6';
            obj.TargetNames{23} = 'Prat3_frente7';
            obj.TargetNames{24} = 'Prat3_frente8';
            obj.TargetNames{25} = 'Prat4_frente1';
            obj.TargetNames{26} = 'Prat4_frente2';
            obj.TargetNames{27} = 'Prat4_frente3';
            obj.TargetNames{28} = 'Prat4_frente4';
            obj.TargetNames{29} = 'Prat4_frente5';
            obj.TargetNames{30} = 'Prat4_frente6';
            obj.TargetNames{31} = 'Prat4_frente7';
            obj.TargetNames{32} = 'Prat4_frente8';
            obj.TargetNames{33} = 'Prat5_frente1';
            obj.TargetNames{34} = 'Prat5_frente2';
            obj.TargetNames{35} = 'Prat5_frente3';
            obj.TargetNames{36} = 'Prat5_frente4';
            obj.TargetNames{37} = 'Prat5_frente5';
            obj.TargetNames{38} = 'Prat5_frente6';
            obj.TargetNames{39} = 'Prat5_frente7';
            obj.TargetNames{40} = 'Prat5_frente8';
            obj.TargetNames{41} = 'Prat6_frente1';
            obj.TargetNames{42} = 'Prat6_frente2';
            obj.TargetNames{43} = 'Prat6_frente3';
            obj.TargetNames{44} = 'Prat6_frente4';
            obj.TargetNames{45} = 'Prat6_frente5';
            obj.TargetNames{46} = 'Prat6_frente6';
            obj.TargetNames{47} = 'Prat6_frente7';
            obj.TargetNames{48} = 'Prat6_frente8';

            %Get Targets Handle
            for a=1:obj.TARGET_Number
                target_name = ['/',obj.TargetNames{a}];
                [res, obj.TargetHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, target_name, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting target handle');
                    error = 1;
                    return;
                end
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{a},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
            end


            %Get ConveyorBelt Handle
            [res, obj.ConveyorBeltHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'ConveyorBelt', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting conveyorbelt handle');
                error = 1;
                return;
            end

            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', 0, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed set conveyorbelt velocity!');
                error=1;
                return;
            end

             %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed to setup data streaming for simulation time');
                error = 1;
                return;
            end

            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed to setup data streaming for simulation time step');
                error = 1;
                return;
            end
        end

        function [vrep, clientID] = get_connection(obj)
            vrep = obj.vrep;
            clientID = obj.clientID;
        end

        % call before setting velocity and getting data
        function ensure_all_data(obj)
            obj.vrep.simxGetPingTime(obj.clientID);
        end

        % call just after all data have been collected.
        function trigger_simulation(obj)
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end

        %Function that allows you to get the position of the target
        function [error,targetPosition]=get_target_position(obj,iTarget)
            error = 0;
            [res,targetPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{iTarget},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting target position information');
                error = 1;
                return;
            end
        end

         %Function that allows you to get the position of the target
        function [error,objectPosition]=get_object_position(obj,iObject)
            error = 0;
            [res,objectPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.ObjectHandle{iObject},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting object position information');
                error = 1;
                return;
            end
        end

        %Function that allows you to put the conveyor belt stopped
        function error = stop_conveyorbelt(obj)
            value = 0;
            error = 0;
            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', value, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed stopping belt!');
                error=1;
                return;
            end
        end

        %Function that allows you to put the conveyor belt in motion
        function error = move_conveyorbelt(obj)
            value = 0.08;
            error = 0;
            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', value, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed moving belt!');
                error = 1;
                return;
            end
        end

        function [error,sim_time] = get_simulation_time(obj)
            error = 0;
            [res, sim_time] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTime', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: The simulation stopped in CoppeliaSim');
                error = 1;
                return;
            end
        end

        function [error,sim_timestep] = get_simulation_timestep(obj)
            error = 0;
            [res, sim_timestep] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed getting simulation time step!');
                error = 1;
                return;
            end
        end

        function error = terminate(obj)
            error = 0;
            res = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed stopping simulation!');
                error = 1;
                return;
            end
            obj.vrep.simxGetPingTime(obj.clientID);

            % Now close the connection to CoppeliaSim:
            obj.vrep.simxFinish(obj.clientID);

            obj.vrep.delete(); % call the destructor!
        end
    end

end


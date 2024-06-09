classdef simulator_interface < handle
%   simulator_interface CoppeliaSim connection interface for a Robot scene
%   The class provides the communication service and information
%   pre-processing for a MATLAB based control interface
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2022
%   2022/03/17
    
    properties (Access = private)
        vrep
        clientID
        TargetHandle
        ObjectHandle
        ObjectNames
        ConveyorInHandle
        ConveyorOutHandle % handle for the conveyor out object
        sensorHandle
    end
    
    properties
        TARGET_Number
        targetName
        OBJECT_Number
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


            %Objects 
            % Six different objects
            obj.OBJECT_Number = 6;
            obj.ObjectNames{1} = 'caixa_600_400_1';
            obj.ObjectNames{2} = 'caixa_400_300_1';
            obj.ObjectNames{3} = 'caixa_600_400_2';
            obj.ObjectNames{4} = 'caixa_400_300_2';
            obj.ObjectNames{5} = 'caixa_600_400_3';
            obj.ObjectNames{6} = 'caixa_400_300_3';

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
            
            
            %TODO: Change here to support more targets
            obj.TARGET_Number=5;
            
            % target handle
            
            for t=1:obj.TARGET_Number
                targetName{t} = ['Target',num2str(t)];
                [res,obj.TargetHandle{t}]=obj.vrep.simxGetObjectHandle(obj.clientID,targetName{t},obj.vrep.simx_opmode_blocking);
                
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting target handle ');
                    error = 1;
                    return;
                end
                
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{t},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
            end
            
            % %Get ConveyorBelt Handle
            % [res, obj.ConveyorOutHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, '/conveyor_out', obj.vrep.simx_opmode_blocking);
            % if (res ~= obj.vrep.simx_return_ok)
            %     disp('ERROR: Failed getting conveyor out handle');
            %     error = 1;
            %     return;
            % end
            % 
            % [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', 0, obj.vrep.simx_opmode_oneshot);
            % if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
            %     disp('ERROR: Failed set conveyorbelt velocity!');
            %     error=1;
            %     return;
            % end
            %Get ConveyorOut Handle
            [res, obj.ConveyorOutHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, '/conveyor_out', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting conveyor out handle');
                error = 1;
                return;
            end

            %Get ConveyorIn Handle
            [res, obj.ConveyorInHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, '/conveyor_in', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting conveyor in handle');
                error = 1;
                return;
            end
            
            %Set ConveyorOut velocity to 0
            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocityOut', 0, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed set conveyorbelt velocity!');
                error=1;
                return;
            end

            %Set ConveyorIn velocity to 0
            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocityIn', 0, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed set conveyorbelt_in velocity!');
                error=1;
                return;
            end


             %Get Sensor Handle 
            [res, obj.sensorHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, '/conveyor_in/Proximity_sensor', obj.vrep.simx_opmode_blocking);
            if res == obj.vrep.simx_return_ok
                disp('Sensor Handle is ok');
            else  
                 disp('ERROR:Failed to get sensor handle');
            end 


            %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed to setup data streaming for simulation time');
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
        
        function [error,XTARGET,YTARGET] = get_TargetPosition( obj, t)
            %UNTITLED2 Summary of this function goes here
            %   Detailed explanation goes here
            %XTARGET and YTARGET in cm
            error = 0;
            [res,tposition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{t},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting target position!');
                error = 1;
                return;
            end
            
            XTARGET=tposition(1)*100;      %cm
            YTARGET=tposition(2)*100;      %cm
        end
        
        %Function that allows you to put the conveyor belt stopped
        % function error = stop_conveyorbelt(obj)
        %     value = 0;
        %     error = 0;
        %     [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', value, obj.vrep.simx_opmode_oneshot);
        %     if (res ~= obj.vrep.simx_return_ok )
        %         disp('ERROR: Failed stopping belt!');
        %         error=1;
        %         return;
        %     end
        % end
        % 
        % %Function that allows you to put the conveyor belt in motion
        % function error = move_conveyorbelt(obj)
        %     value = 0.1;
        %     error = 0;
        %     [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', value, obj.vrep.simx_opmode_oneshot);
        %     if (res ~= obj.vrep.simx_return_ok )
        %         disp('ERROR: Failed moving belt!');
        %         error = 1;
        %         return;
        %     end
        % end
         %Function that allows you to put the conveyor belt stopped
        function error = stop_conveyorbelt(obj,sel_conv)
            value = 0;
            error = 0;

             if(sel_conv==1)
                [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocityIn', value, obj.vrep.simx_opmode_oneshot);
            elseif(sel_conv==2)
                [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocityOut', value, obj.vrep.simx_opmode_oneshot);
            else 
                disp('ERROR: Failed selecting conveyor!');
                error = 1;
                return ;
            end 
         
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed stopping conveyor!');
                error=1;
                return;
            end
        end

        %Function that allows you to put the conveyor belt in motion
        function error = move_conveyorbelt(obj,sel_conv)
            value = 0.1;
            error = 0;
            if(sel_conv==1)
                [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocityIn', value, obj.vrep.simx_opmode_oneshot);
            elseif(sel_conv==2)
                [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocityOut', value, obj.vrep.simx_opmode_oneshot);
            else 
                disp('ERROR: Failed selecting conveyor!');
                error = 1;
                return ;
            end 

            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed moving conveyor!');
                error = 1;
                return;
            end
        end

        function [error,detectionState]=ReadProximitySensor(obj)
            error=0; 
            [res,detectionState,~,~,~]  = obj.vrep.simxReadProximitySensor(obj.clientID, '/conveyor_in/Proximity_sensor', obj.vrep.simx_opmode_streaming);
                 if(detectionState)  
                    disp('Object detected: ', num2str(detectionState))
                 else 
                    disp(['No object detected : ',num2str(detectionState)])
                end 
      
        end 

        function [error,sim_time] = get_simulation_time(obj)
            error = 0;
            [res, sim_time] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTime', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: The simulation stopped in CoppeliaSim');
                error = 1;
                obj.terminate();
                return;
            end
        end
        
        function [error,sim_timestep] = get_simulation_timestep(obj)
            error = 0;
            [res, sim_timestep] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed getting simulation time step!');
                error = 1;
                obj.terminate();
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
            
            % Now close the connection to V-REP:
            obj.vrep.simxFinish(obj.clientID);
            
            obj.vrep.delete(); % call the destructor!
        end
    end
    
end


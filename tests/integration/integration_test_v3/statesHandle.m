% States Handlers 

classdef statesHandle < handle
    properties
        rob_L
        rob_W
        lambdaTarget
        lambda_v
        max_d_limit
        min_d_limit
        stop_time
        euler_pass
        obsSensorNumber
        beta1
        beta2
        Q
    end

    methods
        %%********* Constructor method  *********%%
        function obj = statesHandle(rob_L, rob_W, lambdaTarget, lambda_v, max_d_limit, min_d_limit, stop_time, euler_pass, obsSensorNumber, beta1, beta2, Q)
            obj.rob_L = rob_L;
            obj.rob_W = rob_W;
            obj.lambdaTarget = lambdaTarget;
            obj.lambda_v = lambda_v;
            obj.max_d_limit = max_d_limit;
            obj.min_d_limit = min_d_limit;
            obj.stop_time = stop_time;
            obj.euler_pass = euler_pass;
            obj.obsSensorNumber = obsSensorNumber;
            obj.beta1 = beta1;
            obj.beta2 = beta2;
            obj.Q = Q;
        end
        % Method for handling the Idle state
        % function result = handleIdle(obj, arg1, arg2)
        %     result = arg1 + arg2; % Example computation
        % end
        
        %%********* Method for handling the GoToTarget state *********%%
        function [vrobot_y_out, vrobot_x_out, wrobot_out, parkPositionReached] = handleGoToTarget(obj, YTARGET, XTARGET, yrobot, xrobot, vrobot_x_in, phirobot, lambda_obs, psi_obs, sigma, fobs, theta_obs, dist)
            parkPositionReached = 0;
            %-------------Navigation Direction-------------%
            psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot); % Angle in radians
            ftar = -obj.lambdaTarget*sin(phirobot - psitarget);

            %-----------------Speed Control----------------%
            distance = sqrt((YTARGET - yrobot)^2 + (XTARGET - xrobot)^2);

            if(distance >= obj.max_d_limit) % Set desired speed to max value
                vrobot_des = 100.0;

            elseif((distance >= obj.min_d_limit) && (distance <= obj.max_d_limit)) % Calculate and set desired speed
                vrobot_des = distance/obj.stop_time;

            elseif(distance <= obj.min_d_limit) % Set desired speed to min value
                vrobot_des = 30.0;
                parkPositionReached = 1; %todo: Change State Aux Flag

            else
                vrobot_des = 0.0;
            end

            vrobot_y_out = 0.0; % Forces speed on y-axis to 0
            acc = -obj.lambda_v*(vrobot_x_in - vrobot_des); % Calculates acceleration with current speed and desired value
            vrobot_x_out = vrobot_x_in + acc*obj.euler_pass; % Uses euler pass as time variable for speed formula
    
            %--------------Obstacle Avoidance--------------%
            deltaThetaObs = theta_obs(2) - theta_obs(1);
            Fobs = 0;
            for i = 1:obj.obsSensorNumber
                lambda_obs(i)   = obj.beta1*exp(-dist(i)/(obj.beta2));
                psi_obs(i)      = phirobot + theta_obs(i);
                sigma(i)        = atan(tan(deltaThetaObs/2) + (obj.rob_L/2)/((obj.rob_W/2) + dist(i)));
                fobs(i)         = lambda_obs(i)*(phirobot - psi_obs(i))*exp(-(phirobot - psi_obs(i))*(phirobot - psi_obs(i))/(2*sigma(i)*sigma(i)));
                Fobs = Fobs + fobs(i);
            end
            f_stock = sqrt(obj.Q)*randn(1,obj.obsSensorNumber);
            wrobot_out = Fobs + f_stock + ftar;
        end
        
        % Define methods for other states similarly...
    end
end
% States Handlers 

classdef statesHandler < handle
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
        changeTargetDist
        phi_parking

    end

    methods
        %%********* Constructor method  *********%%
        function obj = statesHandler(rob_L, rob_W, lambdaTarget, lambda_v, max_d_limit, min_d_limit, stop_time, euler_pass, obsSensorNumber, beta1, beta2, Q, changeTargetDist, phi_parking)
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
            obj.changeTargetDist = changeTargetDist;
            obj.phi_parking = phi_parking;

        end
        % Method for handling the Idle state
        % function result = handleIdle(obj, arg1, arg2)
        %     result = arg1 + arg2; % Example computation
        % end
        
        %%********* Method for handling the GoToTarget state *********%%
        function [vrobot_y_out, vrobot_x_out, wrobot_out, parkPositionReached] = handlerGoToTarget(obj, YTARGET, XTARGET, yrobot, xrobot, vrobot_x_in, phirobot, lambda_obs, psi_obs, sigma, fobs, theta_obs, dist)
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
        %%************************************************************%%

        %%******** Method for handling the InitParking state *********%%
        function [vrobot_y_out, vrobot_x_out, wrobot_out, isParked, phi_parking_out] = handlerInitParking(obj, YTARGET, XTARGET, yrobot, xrobot, vrobot_des, phirobot, phi_parking_in, itarget, fobs)
            isParked = 0;
            phi_parking_out = phi_parking_in;
            ftar = -obj.lambdaTarget*sin(phirobot - phi_parking_in);
            distance = sqrt((YTARGET - yrobot)^2 + (XTARGET - xrobot)^2);
            psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot); % Angle in radians
            if(distance > obj.changeTargetDist)    
                vrobot_x_out = vrobot_des * cos(psitarget - phi_parking_in);
                vrobot_y_out = vrobot_des * sin(psitarget - phi_parking_in);
                wrobot_out = ftar;
                if(itarget == 2 || itarget == 3)
                    for i = 16:20
                        if fobs (i) > 0
                            k = (lambda_obs(i) * (sigma(i))^2) / sqrt(exp(1));
                            U_pot = lambda_obs(i) * (sigma(i)^2) * exp(-(phi - psi_obs(i))^2) / (2 * (sigma(i))^2) - k;
                            U_robot = U_pot + U_robot;
                            phi_parking_out = phi_parking_in + U_robot - k_phi;
                        end
                    end
                end
            else
                isParked = 1; %todo: Change State Aux Flag
                phi_parking_out = obj.phi_parking(itarget);
                vrobot_x_out = 0;
                vrobot_y_out = 0;
                wrobot_out = ftar;
            end
        end
        %%************************************************************%%

        %%******** Method for handling the PickBox state *********%%
        function [error, close_gripper_out, delay_grip_out, isGripperClosed_out, armJoints, setJoints, closeHand, picked] = handlerPickBox(~, itarget, close_gripper_in, delay_grip_in, isGripperClosed_in, start)
            error = 0;
            close_gripper_out = close_gripper_in;
            delay_grip_out = 0;
            isGripperClosed_out = isGripperClosed_in;
            armJoints = 0;
            setJoints = 0;
            closeHand = 0;
            picked = 0; 
            if(close_gripper_in == 0)
                if(itarget == 2)
                    armJoints(1)=90*pi/180;
                    armJoints(2)=0*pi/180;
                    armJoints(3)=-90*pi/180;
                    armJoints(4)=-90*pi/180;
                    armJoints(5)=90*pi/180;
                    armJoints(6)=0*pi/180;
                    armJoints(7)=0*pi/180;
                elseif(itarget == 3)
                    armJoints(1)=90*pi/180;
                    armJoints(2)=0*pi/180;
                    armJoints(3)=-90*pi/180;
                    armJoints(4)=-90*pi/180;
                    armJoints(5)=90*pi/180;
                    armJoints(6)=0*pi/180;
                    armJoints(7)=0*pi/180;
                else
                    disp('Error in target to move robotic arm!');
                    error = 1;
                    return;
                end
                
                setJoints = 1;
                close_gripper_out = 1;
            else
                delay_grip_out = delay_grip_in + toc(start);
                if(delay_grip_out > 3 && isGripperClosed_in == 0)
                    closeHand = 1;
                    isGripperClosed_out = 1;
                    
                elseif(delay_grip_out > 6)
                    isGripperClosed_out = 0;
                    close_gripper_out = 0;
                    delay_grip_out = 0;
                    picked = 1; %todo: Change State Aux Flag
                end
            end
        end
        %%************************************************************%%

        %%******** Method for handling the PlaceBox state *********%%
        function [error, open_gripper_out, delay_grip_out, isGripperOpened_out, armJoints, setJoints, openHand, picked, placed] = handlerPlaceBox(~, itarget, open_gripper_in, delay_grip_in, isGripperOpened_in, start)
            error = 0;
            open_gripper_out = open_gripper_in;
            delay_grip_out = 0;
            isGripperOpened_out = isGripperOpened_in;
            armJoints = 0;
            setJoints = 0;
            openHand = 0;
            picked = 1;
            placed = 0; 
            if(open_gripper_in == 0)
                if(itarget ~= 1)
                    armJoints(1)=90*pi/180;
                    armJoints(2)=0*pi/180;
                    armJoints(3)=-90*pi/180;
                    armJoints(4)=-90*pi/180;
                    armJoints(5)=90*pi/180;
                    armJoints(6)=0*pi/180;
                    armJoints(7)=0*pi/180;
                else
                    disp('Error in target to move robotic arm!');
                    error = 1;
                    return;
                end
                
                setJoints = 1;
                open_gripper_out = 1;
            else
                delay_grip_out = delay_grip_in + toc(start);
                if(delay_grip_out > 3 && isGripperOpened_in == 0)
                    openHand = 1;
                    isGripperOpened_out = 1;
                    
                elseif(delay_grip_out > 6)
                    isGripperOpened_out = 0;
                    open_gripper_out = 0;
                    delay_grip_out = 0;
                    picked = 0; 
                    placed = 1; %todo: Change State Aux Flag
                end
            end
        end
        %%************************************************************%%   
        
        %%******** Method for handling the MoveArm state *********%%
        function [error, armMoved_out, delay_movArm_out, armJoints, setJoints, transPosArm, defPosArm] = handlerMoveArm(~, armMoved_in, picked, placed, delay_movArm_in, start)
            error = 0;
            armMoved_out = armMoved_in;
            delay_movArm_out = 0;
            armJoints = 0;
            setJoints = 0;
            transPosArm = 0;
            defPosArm = 0;
            if(armMoved_in == 0)
                if(picked == 1) % Move to transport position
                    armJoints(1)=-90*pi/180;
                    armJoints(2)=0*pi/180;
                    armJoints(3)=90*pi/180;
                    armJoints(4)=90*pi/180;
                    armJoints(5)=-90*pi/180;
                    armJoints(6)=0*pi/180;
                    armJoints(7)=0*pi/180;

                    setJoints = 1;
                    armMoved_out = 1;
                
                elseif(placed == 1) % Move to default position
                    setJoints = 1;
                    armMoved_out = 1;
                else
                    disp('Error: Cannot identify if box was picked or placed!')
                    error = 1;
                    return;
                end
            else
                delay_movArm_out = delay_movArm_in + toc(start);
                if(delay_movArm_out > 5)
                    armMoved_out = 0;
                    delay_movArm_out = 0;
                    if(picked == 1)
                        transPosArm = 1; %todo: Change State Aux Flag
                    elseif(placed == 1)
                        defPosArm = 1; %todo: Change State Aux Flag
                    end
                end
            end
        end
        %%************************************************************%%  

        %%******** Method for handling the ExitParking state *********%%
        function [vrobot_y_out, vrobot_x_out, wrobot_out, itarget_out, delay_exitPark_out, waitForBox, move_omni, exitPark] = handlerExitParking(obj, YTARGET, XTARGET, yrobot, xrobot, vrobot_des, phirobot, phi_parking, box_low, box_high, picked, itarget_in, delay_exitPark_in, start)
            exitPark = 0;
            move_omni = 0;
            waitForBox = 0;
            itarget_out = itarget_in;
            vrobot_x_out = 0;
            vrobot_y_out = 0;
            wrobot_out = 0;
            delay_exitPark_out = delay_exitPark_in + toc(start);
            if(delay_exitPark_out > 7)
                exitPark = 1; %todo: Change State
                delay_exitPark_out = 0;
                vrobot_y_out = 0.0;
                if(itarget_in ~= 4)
                    itarget_out = 5;

                else
                    vrobot_x_out = 0;
                    vrobot_y_out = 0;
                end
            else
                if(itarget_in == 5) 
                    exitPark = 1; %todo: Change State
                    if(picked == 1)
                        itarget_out = 4;
                    else
                        move_omni = 1; %todo: Change State
                        if(box_high == 1)
                            itarget_out = 2;
                        elseif(box_low == 1)
                            itarget_out = 3;
                        else
                            waitForBox = 1;
                        end
                    end
    
                elseif(itarget_in ~= 4)
                    psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot);
                    vrobot_x_out = vrobot_des * -cos(psitarget - phi_parking);
                    vrobot_y_out = vrobot_des * -sin(psitarget - phi_parking);
                    ftar = -obj.lambdaTarget*sin(phirobot - phi_parking);
                    wrobot_out = ftar;
                else
                    vrobot_x_out = 0;
                    vrobot_y_out = 0;
                end
            end
        end
        %%************************************************************%%

        %%******** Method for handling the GoToDefPos state *********%%
        function [vrobot_y_out, vrobot_x_out, wrobot_out, startRotate_out, delay_rotate_out, defPositionReached] = handlerGoToDefPos(obj, YTARGET, XTARGET, yrobot, xrobot, startRotate_in, phirobot, phi_parking, delay_rotate_in, start)
            startRotate_out = startRotate_in;
            delay_rotate_out = 0;
            defPositionReached = 0;
            psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot);
            distance = sqrt((YTARGET - yrobot)^2 + (XTARGET - xrobot)^2);
            if(distance <= obj.changeTargetDist || startRotate_in == 1)
                startRotate_out = 1;    
                vrobot_x_out = 5 * cos(psitarget - phi_parking);
                vrobot_y_out = 5 * sin(psitarget - phi_parking);
                ftar = -obj.lambdaTarget*sin(phirobot - phi_parking);
                wrobot_out = ftar;
                delay_rotate_out = delay_rotate_in + toc(start);
                if(delay_rotate_out > 5)
                    delay_rotate_out = 0;
                    startRotate_out = 0;
                    defPositionReached = 1; %todo: Change State Aux Flag
                end
            elseif(startRotate_in == 0)
                vrobot_y_out = 0.0;
                vrobot_x_out = 30;
                psitarget = atan2(YTARGET - yrobot, XTARGET - xrobot);
                ftar = -obj.lambdaTarget*sin(phirobot - psitarget);
                wrobot_out = ftar;
            end
    end
    %%************************************************************%%
    end
end
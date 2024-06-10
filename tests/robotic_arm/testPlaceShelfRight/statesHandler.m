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
        stopTimeTrasp
        stopTimeEndEffector
        stopTimePick
        stopTimePlace
        distanceHand
    end

    methods
        %%********* Constructor method  *********%%
        function obj = statesHandler(rob_L, rob_W, lambdaTarget, lambda_v, max_d_limit, min_d_limit, stop_time, euler_pass, obsSensorNumber, beta1, beta2, Q, changeTargetDist, phi_parking, distanceHand)
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
            obj.stopTimeTrasp = 10;
            obj.stopTimeEndEffector = 20; 
            obj.stopTimePick = 8;
            obj.stopTimePlace = 8;
            obj.distanceHand = distanceHand;
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

        %%******** Method for handling the MoveArmEndEffector state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, desPoseHand, stopTraj] = handlerMoveArmEndEffector(obj, initJoints, finalJoints, startTraj, delay, start, poseObj, poseRobot, box_high, itarget)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            desPoseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            stopTraj = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimeEndEffector);
                setJoints = 1;
                if(delay_out >= obj.stopTimeEndEffector)
                    delay_out = 0;
                    setJoints = 0;
                    stopTraj = 1;
                end  
            else
                [poseObjInBaseRef, rotObj] = (refWorldToBase(poseRobot, poseObj));
                poseObjInBaseRef = poseObjInBaseRef';
                if(itarget == 4)
                    rot07 = [-rotObj(1:3, 1), rotObj(1:3, 2), -rotObj(1:3, 3)];
                    % rot07 = [-rotObj(1:3, 2), -rotObj(1:3, 1), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1), poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.15, des_yaw_x, des_pitch_y, des_roll_z]';
                elseif(itarget == 1)
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1)-0.12, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z]';
                elseif(itarget == 6)
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1)-0.1, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z]';
                elseif(itarget == 7)
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1), poseObjInBaseRef(2)-0.2, poseObjInBaseRef(3)+obj.distanceHand+0.20, des_yaw_x, des_pitch_y, des_roll_z]';
                else
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    if(box_high == 1)
                        desPoseHand = [poseObjInBaseRef(1)-0.1, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z]';
                    else
                        desPoseHand = [poseObjInBaseRef(1), poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z]';
                    end
                end
                
                calcInvKin = 1;
            end
        end
        %%************************************************************%%  

        %%******** Method for handling the PickBox state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, desPoseHand, closeGripper] = handlerPickBox(obj, initJoints, finalJoints, startTraj, delay, start, poseObj, poseRobot, box_high, itarget)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            desPoseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            closeGripper = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimePick);
                setJoints = 1;
                if(delay_out >= obj.stopTimePick)
                    delay_out = 0;
                    setJoints = 0;
                    closeGripper = 1;
                end  
            else
                [poseObjInBaseRef, rotObj] = (refWorldToBase(poseRobot, poseObj));
                poseObjInBaseRef = poseObjInBaseRef';
                rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                if(itarget == 1)
                    if(box_high == 1)
                        desPoseHand = [poseObjInBaseRef(1)-0.001, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand-0.1, des_yaw_x, des_pitch_y, des_roll_z]';
                    else
                        desPoseHand = [poseObjInBaseRef(1)-0.1, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand, des_yaw_x, des_pitch_y, des_roll_z]';
                    end
                else
                    if(box_high == 1)
                        desPoseHand = [poseObjInBaseRef(1)-0.12, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand-0.1, des_yaw_x, des_pitch_y, des_roll_z]';
                    else
                        desPoseHand = [poseObjInBaseRef(1), poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand-0.1, des_yaw_x, des_pitch_y, des_roll_z]';
                    end
                end
                calcInvKin = 1;
            end
        end
        %%************************************************************%%

        %%******** Method for handling the PlaceBox state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, desPoseHand, openGripper] = handlerPlaceBox(obj, initJoints, finalJoints, startTraj, delay, start, poseObj, poseRobot, itarget)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            desPoseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            openGripper = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimePlace);
                setJoints = 1;
                if(delay_out >= obj.stopTimePlace)
                    delay_out = 0;
                    setJoints = 0;
                    openGripper = 1;
                end  
            else
                [poseObjInBaseRef, rotObj] = (refWorldToBase(poseRobot, poseObj));
                poseObjInBaseRef = poseObjInBaseRef';
                if(itarget == 4)
                    % rot07 = [-rotObj(1:3, 3), rotObj(1:3, 2), rotObj(1:3, 1)];
                    rot07 = [-rotObj(1:3, 1), rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1), poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand-0.1, des_yaw_x, des_pitch_y, des_roll_z]';
                elseif(itarget == 6)
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1)+0.05, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.15, des_yaw_x, des_pitch_y+20*pi/180, des_roll_z-10*pi/180]';
                elseif(itarget == 7)
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1)+0.05, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.15, des_yaw_x, des_pitch_y-20*pi/180, des_roll_z]';
                else
                    rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                    [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                    desPoseHand = [poseObjInBaseRef(1)+0.05, poseObjInBaseRef(2), poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z]';
                end
                calcInvKin = 1;
            end
        end
        %%************************************************************%%   
        
        %%******** Method for handling the MoveArmTransp state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, desPoseHand, stopTraj] = handlerMoveArmTransp(obj, initJoints, finalJoints, startTraj, delay, start, poseObj, poseRobot, step)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            desPoseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            stopTraj = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimeTrasp);
                setJoints = 1;
                if(delay_out >= obj.stopTimeTrasp)
                    delay_out = 0;
                    setJoints = 0;
                    stopTraj = 1;
                end  
            else
                [poseObjInBaseRef, rotObj] = (refWorldToBase(poseRobot, poseObj));
                poseObjInBaseRef = poseObjInBaseRef';
                rot07 = [rotObj(1:3, 1), -rotObj(1:3, 2), -rotObj(1:3, 3)];
                [des_yaw_x, des_pitch_y, des_roll_z] = kinematics.computeMatrixToRPY(rot07);
                if(step == 0)
                    desPoseHand = [poseObjInBaseRef(1)+0.2, poseObjInBaseRef(2)-0.5, poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z+pi/2]';
                elseif(step == 1)
                    desPoseHand = [poseObjInBaseRef(1)-0.1, poseObjInBaseRef(2)-0.5, poseObjInBaseRef(3)+obj.distanceHand+0.25, des_yaw_x, des_pitch_y, des_roll_z+pi/2]';
                elseif(step == 2)
                    desPoseHand = [poseObjInBaseRef(1)-0.1, poseObjInBaseRef(2)-0.5, poseObjInBaseRef(3)+obj.distanceHand, des_yaw_x, des_pitch_y, des_roll_z+pi/2]';
                end
                calcInvKin = 1;
            end
        end
        %%************************************************************%%  

        %%******** Method for handling the ExitParking state *********%%
        function [vrobot_y_out, vrobot_x_out, wrobot_out, itarget_out, delay_exitPark_out, waitForBox, move_omni, exitPark] = handlerExitParking(obj, YTARGET, XTARGET, yrobot, xrobot, vrobot_des, phirobot, phi_parking, box_low, box_high, picked, itarget_in, delay_exitPark_in, start, retrieve_box)
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
                if(itarget_in == 5) % Central Position
                    exitPark = 1; %todo: Change State
                    if(retrieve_box == 1)
                        if(picked == 1)
                            itarget_out = 4; % Conveyor_out
                        else
                            move_omni = 1; %todo: Change State
                            if(box_high == 1)
                                itarget_out = 2; % ShelfLeft
                            elseif(box_low == 1)
                                itarget_out = 3; % ShelfRight
                            else
                                waitForBox = 1;
                            end
                        end
                    else
                        if(picked == 1)
                            move_omni = 1; %todo: Change State
                            if(box_high == 1)
                                itarget_out = 6; % ShelfLeft
                            elseif(box_low == 1)
                                itarget_out = 7; % ShelfRight
                            end
                        else
                            itarget_out = 1;
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
% States Handlers 

classdef statesHandler < handle
    properties
        distanceHand
        stopTimeConveyor
        stopTimePick
        stopTimeShelf
        stopTimePlace
        stopTimeDefPos
    end 
    
    methods
        %%********* Constructor method  *********%%
        function obj = statesHandler(distanceHand)
            obj.distanceHand = distanceHand;
            obj.stopTimeConveyor = 3;
            obj.stopTimePick = 3;
            obj.stopTimeShelf = 7;
            obj.stopTimePlace = 1;
            obj.stopTimeDefPos = 5;

        end

        %%******** Method for handling the MoveArmConveyor state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, poseHand, stopTraj] = handlerMoveArmConveyor(obj, initJoints, finalJoints, startTraj, delay, start, objPosition, rpy_des_deg, offset)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            poseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            stopTraj = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimeConveyor);
                setJoints = 1;
                if(delay_out >= obj.stopTimeConveyor)
                    delay_out = 0;
                    setJoints = 0;
                    stopTraj = 1;
                end  
            else
                desPosition = objPosition;
                desPosition(1, 1) = desPosition(1, 1) + offset;
                poseHand = refWorldToBase(desPosition', rpy_des_deg');
                calcInvKin = 1;
            end
        end
        %%************************************************************%%

        %%******** Method for handling the Pick state *********%%
        function [delay_out, setJoints, calcInvKin, closeGripper, joints_out, poseHand] = handlerPick(obj, initJoints, finalJoints, startTraj, delay, start, objPosition, rpy_des_deg)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            closeGripper = 0;
            poseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            
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
                desPosition = objPosition;
                desPosition(1, 1) = desPosition(1, 1) + obj.distanceHand;
                poseHand = refWorldToBase(desPosition', rpy_des_deg');
                calcInvKin = 1;
            end
        end
        %%************************************************************%%
        
        %%******** Method for handling the MoveArmShelf state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, poseHand, stopTraj] = handlerMoveArmShelf(obj, initJoints, finalJoints, startTraj, delay, start, shelfPosition, rpy_des_deg, offset_y, offset_z)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            poseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            stopTraj = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimeShelf);
                setJoints = 1;
                if(delay_out >= obj.stopTimeShelf)
                    delay_out = 0;
                    setJoints = 0;
                    stopTraj = 1;
                end  
            else
                desPosition = shelfPosition;
                desPosition(1, 2) = desPosition(1, 2) + offset_y;
                desPosition(1, 3) = desPosition(1, 3) + offset_z;
                poseHand = refWorldToBase(desPosition', rpy_des_deg');
                calcInvKin = 1;
            end
        end
        %%************************************************************%%

        %%******** Method for handling the Place state *********%%
        function [delay_out, setJoints, calcInvKin, openGripper, joints_out, poseHand] = handlerPlace(obj, initJoints, finalJoints, startTraj, delay, start, shelfPosition, rpy_des_deg, offset_y, offset_z)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            openGripper = 0;
            poseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            
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
                desPosition = shelfPosition;
                desPosition(1, 2) = desPosition(1, 2) + offset_y;
                desPosition(1, 3) = desPosition(1, 3) + offset_z;
                poseHand = refWorldToBase(desPosition', rpy_des_deg');
                calcInvKin = 1;
            end
        end
        %%************************************************************%%   
        
        %%******** Method for handling the GoToDefPos state *********%%
        function [delay_out, setJoints, calcInvKin, joints_out, poseHand, stopTraj] = handlerGoToDefPos(obj, initJoints, finalJoints, startTraj, delay, start, defPosition, rpy_des_deg, offset_y, offset_z)
            delay_out = 0;
            setJoints = 0;
            calcInvKin = 0;
            poseHand = zeros(1, 6);
            joints_out = zeros(7, 1);
            stopTraj = 0;

            if(startTraj == 1)
                delay_out = delay + toc(start);
                joints_out = trajectoryGen(initJoints, finalJoints, delay_out, obj.stopTimeDefPos);
                setJoints = 1;
                if(delay_out >= obj.stopTimeDefPos)
                    delay_out = 0;
                    setJoints = 0;
                    stopTraj = 1;
                end  
            else
                desPosition = defPosition;
                desPosition(1, 2) = desPosition(1, 2) + offset_y;
                desPosition(1, 3) = desPosition(1, 3) + offset_z;
                poseHand = refWorldToBase(desPosition', rpy_des_deg');
                calcInvKin = 1;
            end
        end
    end
    %%************************************************************%%
end
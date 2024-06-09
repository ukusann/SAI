%   refWorldToBase.m
%   Function to transfer an object in coppelia ref to robot base ref
%   2024/06/07
%   % Copyright (C) 2024
%   Author: Matheus Costa, pg50649@alunos.uminho.pt

function [poseInBaseRef, rotObj] = refWorldToBase(robotPoseInWorldRef, objPoseInWorldRef)
    yaw_x_robot = robotPoseInWorldRef(4);
    pitch_y_robot = robotPoseInWorldRef(5);
    roll_z_robot = robotPoseInWorldRef(6);
    RPYmatrix_robot = kinematics.RPYTransfMatrix(yaw_x_robot, pitch_y_robot, roll_z_robot);
    % RPYmatrix_robot = [
    %                 cos(roll_z_robot)*cos(pitch_y_robot)    -sin(roll_z_robot)*cos(yaw_x_robot)+cos(yaw_x_robot)*sin(pitch_y_robot)*sin(yaw_x_robot)      sin(roll_z_robot)*sin(yaw_x_robot)+cos(roll_z_robot)*sin(pitch_y_robot)*cos(yaw_x_robot);
    %                 sin(roll_z_robot)*cos(pitch_y_robot)     cos(roll_z_robot)*cos(yaw_x_robot)+sin(yaw_x_robot)*sin(pitch_y_robot)*sin(yaw_x_robot)     -cos(roll_z_robot)*sin(yaw_x_robot)+sin(roll_z_robot)*sin(pitch_y_robot)*cos(yaw_x_robot);
    %                -sin(pitch_y_robot)                       cos(pitch_y_robot)*sin(yaw_x_robot)                                                          cos(pitch_y_robot)*cos(yaw_x_robot)
    %                 ];

    % Transformation matrix calculated previously from world {W} to Base {B} ({B} === {0}) 
    transf_w0 = [RPYmatrix_robot, robotPoseInWorldRef(1:3)'; zeros(1, 3), 1];
    
    % Compute transf_0w
    rotation_w0 = transf_w0(1:3, 1:3);
    orig_w0     = transf_w0(1:3, 4);
    rotation_0w = rotation_w0';
    orig_0w     = -rotation_0w*orig_w0;
    transf_0w   = [rotation_0w, orig_0w; zeros(1, 3), 1];
    
    yaw_x_obj = objPoseInWorldRef(4);
    pitch_y_obj = objPoseInWorldRef(5);
    roll_z_obj = objPoseInWorldRef(6);
    RPYmatrix_obj = kinematics.RPYTransfMatrix(yaw_x_obj, pitch_y_obj, roll_z_obj);
    % RPYmatrix_obj = [
    %                 cos(roll_z_obj)*cos(pitch_y_obj)    -sin(roll_z_obj)*cos(yaw_x_obj)+cos(yaw_x_obj)*sin(pitch_y_obj)*sin(yaw_x_obj)      sin(roll_z_obj)*sin(yaw_x_obj)+cos(roll_z_obj)*sin(pitch_y_obj)*cos(yaw_x_obj);
    %                 sin(roll_z_obj)*cos(pitch_y_obj)     cos(roll_z_obj)*cos(yaw_x_obj)+sin(yaw_x_obj)*sin(pitch_y_obj)*sin(yaw_x_obj)     -cos(roll_z_obj)*sin(yaw_x_obj)+sin(roll_z_obj)*sin(pitch_y_obj)*cos(yaw_x_obj);
    %                -sin(pitch_y_obj)                     cos(pitch_y_obj)*sin(yaw_x_obj)                                                    cos(pitch_y_obj)*cos(yaw_x_obj)
    %                 ];

    % Transformation matrix calculated previously from world {W} to Object {Obj}  
    transf_wobj = [RPYmatrix_obj, objPoseInWorldRef(1:3)'; zeros(1, 3), 1];
    
    % Compute transformation from Base to Obj
    transf_0obj = transf_0w*transf_wobj;

    % Get obj RPY in Base ref
    [yaw_x_objBase, pitch_y_objBase, roll_z_objBase] = kinematics.computeMatrixToRPY(transf_0obj(1:3, 1:3)); 

    % Get RotRPY matrix
    rotObj = transf_0obj(1:3, 1:3);

    % Compute pose in Base Ref 
    poseInBaseRef = [transf_0obj(1:3, 4); [yaw_x_objBase; pitch_y_objBase; roll_z_objBase]];
end
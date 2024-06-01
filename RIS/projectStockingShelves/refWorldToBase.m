%   refWorldToBase.m
%   Function to transfer an object in coppelia ref to robot base ref
%   Change the values according to simulation values
%   2024/05/26
%   % Copyright (C) 2024
%   Author: Matheus Costa, pg50649@alunos.uminho.pt

function [poseInBaseRef] = refWorldToBase(positionInWorldRef, rpy_des_deg)
    % Transformation matrix calculated previously from world {W} to Base {B} ({B} === {0}) 
    transf_w0 = [
            0      -1       0         0.00005;
            1       0      -0.01067  -0.00726; 
            0.01067 0       1         0.680;
            0       0       0         1 
            ];

    % Compute transf_0w
    rotation_w0 = transf_w0(1:3, 1:3);
    orig_w0     = transf_w0(1:3, 4);
    rotation_0w = rotation_w0';
    orig_0w     = -rotation_0w*orig_w0;
    transf_0w   = [rotation_0w, orig_0w; zeros(1, 3), 1];
    
    % Create the homogeneous position of the desired value
    positionInWorldRef_h = [positionInWorldRef; 1];

    % Get position in Base ref with transf_0w and the given position
    positionInBaseRef_h = transf_0w*positionInWorldRef_h;

    % Compute pose in Base Ref 
    poseInBaseRef = [positionInBaseRef_h(1:3); rpy_des_deg*pi/180];
end
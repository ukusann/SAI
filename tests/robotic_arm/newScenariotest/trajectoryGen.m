%   trajectoryGen.m
%   Function to calculate arm joints according to desired trajectory
%   The restrictions implemented are 
%   theta(0)  = jointsInit;          theta(T)  = jointsFinal;
%   theta'(0) = 0;                   theta'(T) = 0;
%   2024/05/30
%   % Copyright (C) 2024
%   Author: Matheus Costa, pg50649@alunos.uminho.pt

function [jointsArm] = trajectoryGen(jointsInit, jointsFinal, currentTime, stopTime)
    jointsArm = zeros(7, 1);
    for i = 1:size(jointsInit, 1)
        jointsArm(i, 1) = jointsInit(i, 1) + currentTime^2*(jointsFinal(i, 1) - jointsInit(i, 1))*3/(stopTime^2) - currentTime^3*(jointsFinal(i, 1) - jointsInit(i, 1))*2/(stopTime^3); 
    end
end
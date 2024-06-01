%   kinematics.m
%   Interface for direct and inverse kinematics of robotic manipulator
%   2024/05/04
%   % Copyright (C) 2024
%   Author: Matheus Costa, pg50649@alunos.uminho.pt

classdef kinematics < handle
    properties
        kuka_joint_lim_min  % Array with size equal to number of joints
        kuka_joint_lim_max  % Array with size equal to number of joints 
        Link                % Array with size equal to number of joints
        transf_01
        transf_12
        transf_23
        transf_34
        transf_45
        transf_56
        transf_67
        transf_07           
    end
    methods
        function obj = kinematics(kuka_joint_lim_min, kuka_joint_lim_max, Link)
            obj.kuka_joint_lim_min = kuka_joint_lim_min;
            obj.kuka_joint_lim_max = kuka_joint_lim_max;
            obj.Link = Link;
            obj.transf_01 = zeros(4,4);
            obj.transf_12 = zeros(4,4);
            obj.transf_23 = zeros(4,4);
            obj.transf_34 = zeros(4,4);
            obj.transf_45 = zeros(4,4);
            obj.transf_56 = zeros(4,4);
            obj.transf_67 = zeros(4,4);
            obj.transf_07 = zeros(4,4);
        end

        %*********************Direct Kinematics *************************
        function poseHand = directKinematics(obj, dh_alpha, dh_a, dh_d, dh_theta, theta)
            %* Compute individual transformation matrices
            local_transf_01 = obj.dhTransfMatrix(dh_alpha(1), dh_a(1), dh_d(1), dh_theta(1) + theta(1));
            local_transf_12 = obj.dhTransfMatrix(dh_alpha(2), dh_a(2), dh_d(2), dh_theta(2) + theta(2));
            local_transf_23 = obj.dhTransfMatrix(dh_alpha(3), dh_a(3), dh_d(3), dh_theta(3) + theta(3));
            local_transf_34 = obj.dhTransfMatrix(dh_alpha(4), dh_a(4), dh_d(4), dh_theta(4) + theta(4));
            local_transf_45 = obj.dhTransfMatrix(dh_alpha(5), dh_a(5), dh_d(5), dh_theta(5) + theta(5));
            local_transf_56 = obj.dhTransfMatrix(dh_alpha(6), dh_a(6), dh_d(6), dh_theta(6) + theta(6));
            local_transf_67 = obj.dhTransfMatrix(dh_alpha(7), dh_a(7), dh_d(7), dh_theta(7) + theta(7));
            obj.transf_01 = local_transf_01; 
            obj.transf_12 = local_transf_12;
            obj.transf_23 = local_transf_23;
            obj.transf_34 = local_transf_34;
            obj.transf_45 = local_transf_45;
            obj.transf_56 = local_transf_56;
            obj.transf_67 = local_transf_67;
            
            %* Compute general transformation matrix T_BE
            obj.transf_07 = obj.transf_01 * obj.transf_12 * obj.transf_23 * obj.transf_34 * obj.transf_45 * obj.transf_56 * obj.transf_67;
            % disp(num2str(obj.transf_07))
            %* Cartesian coordinates of Tip {7} with respect to base {0}
            xh_0=obj.transf_07(1,4);
            yh_0=obj.transf_07(2,4);
            zh_0=obj.transf_07(3,4);
            ph_0=[xh_0, yh_0, zh_0]';


            %* Orientation of Tip {7} with respect to base {0}: Roll-Pitch-Yaw
            %angles:
            rotation_07 = obj.transf_07(1:3,1:3); 
            [yaw_x, pitch_y, roll_z] = obj.computeMatrixToRPY(rotation_07)
            handOrientation = [yaw_x, pitch_y, roll_z]'

            %* Conclusion of Direct kinematics
            % Tip pose:
            poseHand = [ph_0; handOrientation];
        end
        %*****************************************************************

        %*********************Inverse Kinematics *************************
        function [error, solPossible, joingAngles_sol1, joingAngles_sol2, joingAngles_sol3, joingAngles_sol4] = inverseKinematics(obj, alpha, desPoseHand)
            joingAngles_sol1 = zeros(7, 1);
            joingAngles_sol2 = zeros(7, 1);
            joingAngles_sol3 = zeros(7, 1);
            joingAngles_sol4 = zeros(7, 1);
            error = 0;
            solPossible = 0; 

            solutionsNum = 4;
            % Get hand position and RPY from PoseHand desired
            desHandPos = desPoseHand(1:3); %todo: Hand position
            yaw_x   = desPoseHand(4);
            pitch_y = desPoseHand(5);
            roll_z  = desPoseHand(6);

            % desHandPos = obj.transf_07(1:3, 4);

            % Create the rotation matrix from RPY 
            rotation_07 = obj.RPYTransfMatrix(yaw_x, pitch_y, roll_z);
            % rotation_07_2 = obj.transf_07(1:3,1:3) 
            
            % Get transfromation from Base {0} to Hand {7} with rotation matrix and hand position 
            desTransf_07 = [rotation_07, desHandPos; zeros(1, 3), 1];

            % Get the z_07 vector
            z_vec_07 = desTransf_07(1:3, 3);

            % Calculate desired Wrist Position from Hand Postion, Hand Link and  rotation in Z axis
            desWristPos = desHandPos - obj.Link(4)*z_vec_07; %todo: Wrist position

            %*Compute Shoulder Position
            desShoulderPos = [0, 0, obj.Link(1)]';  %todo: Shoulder position
            % desShoulderPos = obj.transf_01(1:3, 4);

            %*****

            %*Compute theta_4 Elbow Angle
            Lsw = norm(desWristPos - desShoulderPos);
            % Lsw2 = sqrt((desShoulderPos(1) - desWristPos(1))^2 + (desShoulderPos(2) - desWristPos(2))^2 + (desShoulderPos(3) - desWristPos(3))^2)
            
            argTheta4 = (-obj.Link(2)^2 -obj.Link(3)^2 + Lsw^2)/(2*obj.Link(2)*obj.Link(3));
            
            % Check if argument is inside valid limits, i.e., existence condition (sice argument is cos(theta4))
            if(argTheta4 > 1) || (argTheta4 < -1)
                disp(['---' newline 'Cannot compute theta4!' newline 'Robot desired position outside workspace!' newline 'Exiting simulation...' newline '---']);
                error = 1;
                return;
            end

            % Since we do not consider the elbow bending backwards, we force only one solution
            theta4 = acos(argTheta4); %todo: Theta4
            if((theta4 > obj.kuka_joint_lim_max(4)) || (theta4 < obj.kuka_joint_lim_min(4)))
                disp(['---' newline 'Error in theta4!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                error = 1;
                return;  
            end
            %*****

            %*Compute elbow desired position from alpha value
            sw = desWristPos - desShoulderPos; % direction vectorfrom shoulder to wrist

            sw_vec = sw / Lsw; % unit vector from shoulder to wrist, ^sw
            u = [sw_vec(2), -sw_vec(1), 0]'; % n _|_ u
            u_vec = u / norm(u);
            
            v_vec = cross(u_vec , sw_vec); % v = n X u cross product
            
            cosbeta = (Lsw^2 + obj.Link(2)^2 - obj.Link(3)^2) / (2 * Lsw * obj.Link(2)); % co-sines law
            
            C = desShoulderPos + cosbeta*obj.Link(2) * sw_vec; % center of the circle described by the elbow
            
            R = sqrt(1 - cosbeta^2) * obj.Link(2); % radius of the circle described by the elbow
            
            % Depends on alpha
            E = C + R*(cos(alpha)*u_vec + sin(alpha)*v_vec); % desired elbow position
            
            desElbowPos = E; %todo: Elbow position
            %*****

            %* Compute unit vectors of arm links
            % SE_vec = (desElbowPos - desShoulderPos) / obj.Link(2); % unit vector v_SE = (E - S) / norm(E - S)
            EW_vec = (desWristPos - desElbowPos) / norm(desWristPos - desElbowPos); % unit vector v_EW = (WE) / norm(W - E)
            % ew2 = (desWristPos - desElbowPos)/obj.Link(3)
            SW_vec = sw_vec; % unit vector from Shoulder to wrist
            CE_vec = (desElbowPos - C) / norm(desElbowPos - C);
            % ce2 = (desElbowPos - C) / R
            %*****

            %* Compute elbow's reference frame {4}
            z_vec_04 = -cross(SW_vec, CE_vec);   % z4 = SW_vec X CE_vec cross product
            y_vec_04 = EW_vec;
            x_vec_04 = cross(y_vec_04, z_vec_04);
            %*****

            %* Compute theta1, theta2, theta3
            rotation_04 = [x_vec_04, y_vec_04, z_vec_04];
            rotation_34 = [
                        cos(theta4)     -sin(theta4)   0.0;
                        0.0             0.0           -1.0;     
                        sin(theta4)     cos(theta4)    0.0
                        ];
            
            rotation_03 = rotation_04 * rotation_34';
            
            % 
            % transf03 =
            % [cos(theta1)*cos(theta2)*cos(theta3) - sin(theta1)*sin(theta3), - cos(theta3)*sin(theta1) - cos(theta1)*cos(theta2)*sin(theta3), cos(theta1)*sin(theta2), L2*cos(theta1)*sin(theta2)]
            % [cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1),   cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3), sin(theta1)*sin(theta2), L2*sin(theta1)*sin(theta2)]
            % [                                     -cos(theta3)*sin(theta2),                                         sin(theta2)*sin(theta3),             cos(theta2),        L1 + L2*cos(theta2)]
            % [                                                            0,                                                               0,                       0,                          1]
            %
        
            theta2 = atan2(sqrt(1 - rotation_03(3,3)^2), rotation_03(3,3)); %todo: theta2 sol
            % Put solution 2 always negative
            if(theta2 <= 0)
                theta2_b = theta2;
                theta2_a = atan2(-sqrt(1 - rotation_03(3,3)^2), rotation_03(3,3));
            else
                theta2_a = theta2;
                theta2_b = atan2(-sqrt(1 - rotation_03(3,3)^2), rotation_03(3,3));
            end

            % Check for singularity
            if(abs(theta2_b) < 0.1)
                theta2_b = 0;
                theta1_b = 0;
                theta3_b = atan2(rotation_03(2,1), rotation_03(2,2));
                solutionsNum = solutionsNum - 2; 
                if((theta3_b > obj.kuka_joint_lim_max(3)) || (theta3_b < obj.kuka_joint_lim_min(3)))
                    disp(['---' newline 'Error in theta3!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return; 
                end
            else
                theta1_a = atan2((rotation_03(2,3) / sin(theta2_a)), ( rotation_03(1,3) / sin(theta2_a))); %todo: theta1 sol1
                theta3_a = atan2((rotation_03(3,2) / sin(theta2_a)), (-rotation_03(3,1) / sin(theta2_a))); %todo: theta3 sol1          
            
            
                theta1_b = atan2((rotation_03(2,3) / sin(theta2_b)), ( rotation_03(1,3) / sin(theta2_b))); %todo: theta1 sol2
                theta3_b = atan2((rotation_03(3,2) / sin(theta2_b)), (-rotation_03(3,1) / sin(theta2_b))); %todo: theta3 sol2
                
                if((theta2_a > obj.kuka_joint_lim_max(2)) || (theta2_b < obj.kuka_joint_lim_min(2)))
                    disp(['---' newline 'Error in theta2!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return;  
                    
                elseif((theta1_a > obj.kuka_joint_lim_max(1)) || (theta1_a < obj.kuka_joint_lim_min(1)) || (theta1_b > obj.kuka_joint_lim_max(1)) || (theta1_b < obj.kuka_joint_lim_min(1)))
                    disp(['---' newline 'Error in theta1!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return; 
                    
                elseif((theta3_a > obj.kuka_joint_lim_max(3)) || (theta3_a < obj.kuka_joint_lim_min(3)) || (theta3_b > obj.kuka_joint_lim_max(3)) || (theta3_b < obj.kuka_joint_lim_min(3)))
                    disp(['---' newline 'Error in theta3!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return; 
                end
            end
            %*****

            %* Compute theta5, theta6, theta7
            rotation_47 = rotation_04' * rotation_07;

            % transf47 =
            % [  cos(theta5)*cos(theta6)*cos(theta7) - sin(theta5)*sin(theta7), - cos(theta7)*sin(theta5) - cos(theta5)*cos(theta6)*sin(theta7),  cos(theta5)*sin(theta6),  L4*cos(theta5)*sin(theta6)]
            % [                                       -cos(theta7)*sin(theta6),                                         sin(theta6)*sin(theta7),              cos(theta6),         L3 + L4*cos(theta6)]
            % [- cos(theta5)*sin(theta7) - cos(theta6)*cos(theta7)*sin(theta5),   cos(theta6)*sin(theta5)*sin(theta7) - cos(theta5)*cos(theta7), -sin(theta5)*sin(theta6), -L4*sin(theta5)*sin(theta6)]
            % [                                                              0,                                                               0,                        0,                           1]
 
            theta6 = atan2(sqrt(1 - rotation_47(2,3)^2), rotation_47(2,3)); %todo: theta6 sol1
            % Put solution 2 always negative
            if(theta6 <= 0)
                theta6_b = theta6;
                theta6_a = atan2(-sqrt(1 - rotation_47(2,3)^2), rotation_47(2,3));
            else
                theta6_a = theta6;
                theta6_b = atan2(-sqrt(1 - rotation_47(2,3)^2), rotation_47(2,3));
            end

            % Check for singularity
            if(abs(theta6_b) < 0.1)
                theta6_b = 0;
                theta5_b = 0;
                theta7_b = atan2(-rotation_47(3,1), -rotation_47(3,2));
                solutionsNum = solutionsNum - 2;
                if((theta7_b > obj.kuka_joint_lim_max(7)) || (theta7_b < obj.kuka_joint_lim_min(7)))
                    disp(['---' newline 'Error in theta7!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return; 
                end
            else
                theta5_a = atan2((-rotation_47(3,3) / sin(theta6_a)), ( rotation_47(1,3) / sin(theta6_a))); %todo: theta5 sol1
                theta7_a = atan2(( rotation_47(2,2) / sin(theta6_a)), (-rotation_47(2,1) / sin(theta6_a))); %todo: theta7 sol1

                theta5_b = atan2((-rotation_47(3,3) / sin(theta6_b)), ( rotation_47(1,3) / sin(theta6_b))); %todo: theta5 sol2
                theta7_b = atan2(( rotation_47(2,2) / sin(theta6_b)), (-rotation_47(2,1) / sin(theta6_b))); %todo: theta7 sol2
                
                if((theta6_a > obj.kuka_joint_lim_max(6)) || (theta6_b < obj.kuka_joint_lim_min(6)))
                    disp(['---' newline 'Error in theta6!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return;  
                
                elseif((theta5_a > obj.kuka_joint_lim_max(5)) || (theta5_a < obj.kuka_joint_lim_min(5)) || (theta5_b > obj.kuka_joint_lim_max(5)) || (theta5_b < obj.kuka_joint_lim_min(5)))
                    disp(['---' newline 'Error in theta5!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return; 
                    
                elseif((theta7_a > obj.kuka_joint_lim_max(7)) || (theta7_a < obj.kuka_joint_lim_min(7)) || (theta7_b > obj.kuka_joint_lim_max(7)) || (theta7_b < obj.kuka_joint_lim_min(7)))
                    disp(['---' newline 'Error in theta7!' newline 'Value is outside joint limits!' newline 'Exiting simulation...' newline '---']);
                    error = 1;
                    return; 
                end
            end

            if(solutionsNum == 4)
                joingAngles_sol1 = [theta1_a, theta2_a, theta3_a, theta4, theta5_b, theta6_b, theta7_b]';
                joingAngles_sol2 = [theta1_a, theta2_a, theta3_a, theta4, theta5_a, theta6_a, theta7_a]';
                joingAngles_sol3 = [theta1_b, theta2_b, theta3_b, theta4, theta5_b, theta6_b, theta7_b]';
                joingAngles_sol4 = [theta1_b, theta2_b, theta3_b, theta4, theta5_a, theta6_a, theta7_a]';
                solPossible = 1234;
            elseif(solutionsNum == 2)
                if(theta2_b == 0)
                    joingAngles_sol3 = [theta1_b, theta2_b, theta3_b, theta4, theta5_b, theta6_b, theta7_b]';
                    joingAngles_sol4 = [theta1_b, theta2_b, theta3_b, theta4, theta5_a, theta6_a, theta7_a]';
                    solPossible = 34;
                else
                    joingAngles_sol1 = [theta1_a, theta2_a, theta3_a, theta4, theta5_b, theta6_b, theta7_b]';
                    joingAngles_sol3 = [theta1_b, theta2_b, theta3_b, theta4, theta5_b, theta6_b, theta7_b]';
                    solPossible = 13;
                end
            else
                solPossible = 3;
                joingAngles_sol3 = [theta1_b, theta2_b, theta3_b, theta4, theta5_b, theta6_b, theta7_b]';
            end
        end
        %*****************************************************************

        %*******************Choose InvKin Solution ***********************
        function optimalSolution = chooseInvKinSolution(obj, jointAngles)
            optimalSolution = jointAngles(1, :);
            min = obj.maxDiffAngles(optimalSolution);
            for i = 2:size(jointAngles, 1)
                solution = jointAngles(i, :);
                if((all(solution == 0)) == 1)
                    continue;
                else
                    tmpMin = obj.maxDiffAngles(solution);
                    if(abs(tmpMin) < abs(min))
                        min = tmpMin;
                        optimalSolution = solution;
                    end
                end
            end
        end
    end


    methods (Access = private)
        function transfMatrix = dhTransfMatrix(~, alpha, a, d, theta)
            transfMatrix = [
                    cos(theta)               -sin(theta)               0.0               a;
                    sin(theta)*cos(alpha)     cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
                    sin(theta)*sin(alpha)     cos(theta)*sin(alpha)    cos(alpha)      cos(alpha)*d;
                    0.0                       0.0                      0.0             1.0
                    ];
        end
    
        function [yaw_x, pitch_y, roll_z] = computeMatrixToRPY(~, rot)
            psi = atan2(-rot(3,1),sqrt(rot(1,1)^2+rot(2,1)^2));
            
            if (abs(psi - pi/2) < 0.01)     % psi == pi/2 % 
                psi = pi/2;
                phi = 0;
                theta = atan2(rot(1,2),rot(2,2));
    
            elseif (abs(psi + pi/2) < 0.01) % psi == -pi/2
                psi = -pi/2;
                phi = 0;
                theta = -atan2(rot(1,2),rot(2,2));
            else
                phi   = atan2(rot(2,1),rot(1,1));
                theta = atan2(rot(3,2),rot(3,3));
            end
            
            if(abs(theta) < 0.01)
                theta = 0.0;
            end
    
            if(abs(psi) < 0.01)
                psi = 0.0;
            end
            
            if(abs(phi) < 0.01)
                phi = 0.0;
            end
            yaw_x = theta;
            pitch_y = psi;
            roll_z = phi; 
        end
        % Rot(z)Rot(y)Rot(x)
        function RPYmatrix = RPYTransfMatrix(~, yaw_x, pitch_y, roll_z)
            RPYmatrix = [
                    cos(roll_z)*cos(pitch_y)    -sin(roll_z)*cos(yaw_x)+cos(yaw_x)*sin(pitch_y)*sin(yaw_x)      sin(roll_z)*sin(yaw_x)+cos(roll_z)*sin(pitch_y)*cos(yaw_x);
                    sin(roll_z)*cos(pitch_y)     cos(roll_z)*cos(yaw_x)+sin(yaw_x)*sin(pitch_y)*sin(yaw_x)     -cos(roll_z)*sin(yaw_x)+sin(roll_z)*sin(pitch_y)*cos(yaw_x);
                   -sin(pitch_y)                 cos(pitch_y)*sin(yaw_x)                                       cos(pitch_y)*cos(yaw_x)
                    ];
        end 

        function maxDiff = maxDiffAngles(~, angles)
            maxDiff = 0;
            for i = 2:size(angles, 2)
                diff = angles(1, i) - angles(1, i-1); 
                if(maxDiff == 0)
                    maxDiff = diff;
                elseif(diff < maxDiff)
                    maxDiff = diff;
                end
            end
        end 
    end
end
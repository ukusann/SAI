% Functions and algorithms for direct and inverse kinematics of robotic manipulator

classdef kinematics < handle
    properties
        kuka_joint_lim_min  % Array with size equal to number of joints
        kuka_joint_lim_max  % Array with size equal to number of joints 
        Link                % Array with size equal to number of joints
    end
    methods
        function obj = kinematics(kuka_joint_lim_min, kuka_joint_lim_max, Link)
            obj.kuka_joint_lim_min = kuka_joint_lim_min;
            obj.kuka_joint_lim_max = kuka_joint_lim_max;
            obj.Link = Link;
        end

        function transfMatrix = dhTransfMatrix(~, alpha, a, d, theta)
            transfMatrix = [
                    cos(theta)               -sin(theta)               0.0               a;
                    sin(theta)*cos(alpha)     cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
                    sin(theta)*sin(alpha)     cos(theta)*sin(alpha)    cos(alpha)      cos(alpha)*d;
                    0.0                       0.0                      0.0             1.0
                    ];
        end

        function RPYmatrix = RPYTransfMatrix(~, yaw_x, pitch_y, roll_z)
            RPYmatrix = [
                    cos(roll_z)*cos(pitch_y)    -sin(roll_z)*cos(yaw_x)+cos(roll_z)*sin(pitch_y)*sen(yaw_x)      sin(roll_z)*sen(yaw_x)+cos(roll_z)*sin(pitch_y)*cos(yaw_x);
                    sin(roll_z)*cos(pitch_y)     cos(roll_z)*cos(yaw_x)+sen(roll_z)*sin(pitch_y)*sen(yaw_x)     -cos(roll_z)*sin(yaw_x)+sin(roll_z)*sin(pitch_y)*cos(yaw_x);
                   -sin(pitch_y)                 cos(pitch_y)*sin(yaw_x)                                         cos(pitch_y)*cos(yaw_x)
                    ];
        end

        function [yaw_x, pitch_y, roll_z] = computeMatrixToRPY(~, rot)
            psi = atan2(-rot(3,1),sqrt(rot(1,1)^2+rot(2,1)^2));
            
            if (abs(psi - pi/2) < 0.01)     % psi == pi/2 % 
                phi = 0;
                theta = atan2(rot(1,2),rot(2,2));

            elseif (abs(psi + pi/2) < 0.01) % psi == -pi/2
                phi = 0;
                theta = - atan2(rot(1,2),rot(2,2));
            else
                phi = atan2(rot(2,1)/cos(psi),rot(1,1)/cos(psi));
                theta = atan2(rot(3,2)/cos(psi),rot(3,3)/cos(psi));
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
    end
end
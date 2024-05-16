function [angle_RPY_out, RPY] = AngleConvertSystem(angle_RPY, mode)
    % function [angle_out] = AngleConvertSystem(angle, mode)
    % Calculate the converted angles from the current convention to the desired convention.
    %
    % Inputs:
    %   angle       -> 3D vector with the angle values for conversion ==
    %                  [α, β, γ]
    %   mode        -> Conversion operation mode ==
    %                  {'cop_to_world', 'world_to_cop'}
    %
    % Outputs:
    %   *none*      -> Print the converted angle values.
    %   angle_out   -> Converted angle values.

    if strcmp(mode, 'cop_to_world')
        a = angle_RPY(1);   % α (α - alpha)
        b = angle_RPY(2);   % β (β - beta)
        y = angle_RPY(3);   % γ (γ - gamma)

        % CoppeliaSim Angle Convention
        % RPY = R(X,α).R(Y,β).R(Z,γ)
        %       <-------------------
                
        RPY = [       1           0           0 ;...    % Rotation in X axis of angle α.
                      0      cos(a)     -sin(a) ;...
                      0      sin(a)      cos(a) ] * ...
              [  cos(b)           0      sin(b) ;...    % Rotation in Y axis of angle β.
                      0           1           0 ;...
                -sin(b)           0      cos(b) ] * ...
              [  cos(y)     -sin(y)           0 ;...    % Rotation in Z axis of angle γ.
                 sin(y)      cos(y)           0 ;...
                      0           0           1 ];
        
        % For the orientation methodology present in real world scenarios,
        % RPY = R(Z,α).R(Y,β).R(X,γ) =
        %       <-------------------
        % [cα*cβ, cα*sβ*sγ - cγ*sα, sα*sγ + cα*cγ*sβ]
        % [cβ*sα, cα*cγ + sα*sβ*sγ, cγ*sα*sβ - cα*sγ]
        % [  -sβ,            cβ*sγ,            cβ*cγ]
        
        b = atan2(- RPY(3,1), sqrt((RPY(1,1))^2 + (RPY(2,1)^2)));
                        
        if(abs(abs(b) - (pi/2)) < 0.01)
            a = 0;
        else
            a = atan2(RPY(2,1) / cos(b), RPY(1,1) / cos(b));
        end
                        
        if(abs(abs(b) - (pi/2)) < 0.01)
            y = atan2(- RPY(2,3), RPY(2,2));
        else
            y = atan2(RPY(3,2) / cos(b), RPY(3,3) / cos(b));
        end

        angle_RPY_out = [a; b; y];
    
    elseif strcmp(mode, 'world_to_cop')
        a = angle_RPY(1);   % α (α - alpha)
        b = angle_RPY(2);   % β (β - beta)
        y = angle_RPY(3);   % γ (γ - gamma)

        % World Angle Convention
        % RPY = R(Z,α).R(Y,β).R(X,γ)
        %       <-------------------
                
        RPY = [  cos(a)     -sin(a)           0 ;...    % Rotation in Z axis of angle α.
                 sin(a)      cos(a)           0 ;...    % Roll
                      0           0           1 ] * ...
              [  cos(b)           0      sin(b) ;...    % Rotation in Y axis of angle β.
                      0           1           0 ;...    % Pitch
                -sin(b)           0      cos(b) ] * ...
              [       1           0           0 ;...    % Rotation in X axis of angle γ.
                      0      cos(y)     -sin(y) ;...    % Yaw
                      0      sin(y)      cos(y) ];
        
        % For the orientation methodology present in CoppeliaSim scenarios,
        % RPY = R(X,α).R(Y,β).R(Z,γ) =
        %       <-------------------
        % [          cβ*cγ,          -cβ*sγ,     sβ]
        % [ sα*sβ*cγ+cα*sγ, -sα*sβ*sγ+cα*cγ, -sα*cβ]
        % [-cα*sβ*cγ+sα*sγ,  cα*sβ*sγ+sα*cγ,  cα*cβ]
        
        b = atan2(RPY(1,3), sqrt((RPY(2,3))^2 + (RPY(3,3)^2)));
                            
        if(abs(abs(b) - (pi/2)) < 0.01)
            a = 0;
        else
            a = atan2(- RPY(2,3) / cos(b), RPY(3,3) / cos(b));
        end
        
        if(abs(abs(b) - (pi/2)) < 0.01)
            y = atan2(RPY(2,1), RPY(2,2));
        else
            y = atan2(- RPY(1,2) / cos(b), RPY(1,1) / cos(b));
        end

        angle_RPY_out = [a; b; y];

    elseif strcmp(mode, 'angle_to_RPY')
        a = angle_RPY(1);   % α (α - alpha)
        b = angle_RPY(2);   % β (β - beta)
        y = angle_RPY(3);   % γ (γ - gamma)

        % World Angle Convention
        % RPY = R(Z,α).R(Y,β).R(X,γ)
        %       <-------------------
                
        RPY = [  cos(a)     -sin(a)           0 ;...    % Rotation in Z axis of angle α.
                 sin(a)      cos(a)           0 ;...    % Roll
                      0           0           1 ] * ...
              [  cos(b)           0      sin(b) ;...    % Rotation in Y axis of angle β.
                      0           1           0 ;...    % Pitch
                -sin(b)           0      cos(b) ] * ...
              [       1           0           0 ;...    % Rotation in X axis of angle γ.
                      0      cos(y)     -sin(y) ;...    % Yaw
                      0      sin(y)      cos(y) ];

        angle_RPY_out = RPY;

    elseif strcmp(mode, 'RPY_to_angle')
        RPY = angle_RPY;

        % For the orientation methodology present in real world scenarios,
        % RPY = R(Z,α).R(Y,β).R(X,γ) =
        %       <-------------------
        % [cα*cβ, cα*sβ*sγ - cγ*sα, sα*sγ + cα*cγ*sβ]
        % [cβ*sα, cα*cγ + sα*sβ*sγ, cγ*sα*sβ - cα*sγ]
        % [  -sβ,            cβ*sγ,            cβ*cγ]
        
        b = atan2(- RPY(3,1), sqrt((RPY(1,1))^2 + (RPY(2,1)^2)));
                        
        if(abs(abs(b) - (pi/2)) < 0.01)
            a = 0;
        else
            a = atan2(RPY(2,1) / cos(b), RPY(1,1) / cos(b));
        end
                        
        if(abs(abs(b) - (pi/2)) < 0.01)
            y = atan2(- RPY(2,3), RPY(2,2));
        else
            y = atan2(RPY(3,2) / cos(b), RPY(3,3) / cos(b));
        end

        angle_RPY_out = [a; b; y];

    end
    
    if (nargout == 0)
        array = ['α = ', num2str(a*180/pi), 'º  ', ...
                 'β = ', num2str(b*180/pi), 'º  ', ...
                 'γ = ', num2str(y*180/pi), 'º '];
        disp(array);
    end
end
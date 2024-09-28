%% Calculate Euler Angle from Rotation Matrix and Consider Gimbal Lock 
%% Code by Yingyu Wang 07/05/2024
function [roll, pitch, yaw] = FuncRotMatrix2Euler(R) 
% Calculate pitch
if R(3, 1) < 1
    if R(3, 1) > -1
        pitch = asin(-R(3, 1));
    else
        pitch = -pi/2;
        roll = atan2(R(1, 2), R(1, 3));
        % euler = [roll, pitch, 0];
        return;
    end
else
    pitch = pi/2;
    roll = atan2(R(1, 2), R(1, 3));
    % euler = [roll, pitch, 0];
    return;
end

% Calculate roll and yaw
roll = atan2(R(3, 2)/cos(pitch), R(3, 3)/cos(pitch));
yaw = atan2(R(2, 1)/cos(pitch), R(1, 1)/cos(pitch));
    
end

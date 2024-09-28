function quaternions = FuncEuler2Quat(euler_angles)
num_rows = size(euler_angles, 1);
quaternions = zeros(num_rows, 4);

for i = 1:num_rows
    euler_row = euler_angles(i, :);
    pitch = euler_row(1); 
    yaw = euler_row(2); 
    roll = euler_row(3); 

    pitch_half = pitch / 2;
    yaw_half = yaw / 2;
    roll_half = roll / 2;

    cos_pitch_half = cos(pitch_half);
    sin_pitch_half = sin(pitch_half);
    cos_yaw_half = cos(yaw_half);
    sin_yaw_half = sin(yaw_half);
    cos_roll_half = cos(roll_half);
    sin_roll_half = sin(roll_half);

    q0 = cos_pitch_half * cos_yaw_half * cos_roll_half + sin_pitch_half * sin_yaw_half * sin_roll_half;
    q1 = sin_pitch_half * cos_yaw_half * cos_roll_half - cos_pitch_half * sin_yaw_half * sin_roll_half;
    q2 = cos_pitch_half * sin_yaw_half * cos_roll_half + sin_pitch_half * cos_yaw_half * sin_roll_half;
    q3 = cos_pitch_half * cos_yaw_half * sin_roll_half - sin_pitch_half * sin_yaw_half * cos_roll_half;

    quaternions(i, :) = [q0, q1, q2, q3];
end
end

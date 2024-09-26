function quaternions = FuncEuler2Quat(euler_angles)
% 计算输入矩阵的行数
num_rows = size(euler_angles, 1);

% 初始化输出矩阵
quaternions = zeros(num_rows, 4);

% 循环处理每一行欧拉角
for i = 1:num_rows
    euler_row = euler_angles(i, :);
    % 提取pitch、yaw和roll
    pitch = euler_row(1); % 绕X轴的旋转角度
    yaw = euler_row(2); % 绕Y轴的旋转角度
    roll = euler_row(3); % 绕Z轴的旋转角度

    % 计算欧拉角的一半
    pitch_half = pitch / 2;
    yaw_half = yaw / 2;
    roll_half = roll / 2;

    % 计算sin和cos值
    cos_pitch_half = cos(pitch_half);
    sin_pitch_half = sin(pitch_half);
    cos_yaw_half = cos(yaw_half);
    sin_yaw_half = sin(yaw_half);
    cos_roll_half = cos(roll_half);
    sin_roll_half = sin(roll_half);

    % 计算四元数的四个分量
    q0 = cos_pitch_half * cos_yaw_half * cos_roll_half + sin_pitch_half * sin_yaw_half * sin_roll_half;
    q1 = sin_pitch_half * cos_yaw_half * cos_roll_half - cos_pitch_half * sin_yaw_half * sin_roll_half;
    q2 = cos_pitch_half * sin_yaw_half * cos_roll_half + sin_pitch_half * cos_yaw_half * sin_roll_half;
    q3 = cos_pitch_half * cos_yaw_half * sin_roll_half - sin_pitch_half * sin_yaw_half * cos_roll_half;

    % 将四元数放入输出矩阵中
    quaternions(i, :) = [q0, q1, q2, q3];
end
end

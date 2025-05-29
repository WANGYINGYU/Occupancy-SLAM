%% Convert 3D Pose (x,y,z,r,p,y) to Transformation Matrix
function T = FuncTFMatrix(Pose)

x = Pose(1);
y = Pose(2);
z = Pose(3);
roll = Pose(4);
pitch = Pose(5);
yaw = Pose(6);

% Rotation Matrix
Rx = [1, 0, 0;
      0, cos(roll), -sin(roll);
      0, sin(roll), cos(roll)];
  
Ry = [cos(pitch), 0, sin(pitch);
      0, 1, 0;
      -sin(pitch), 0, cos(pitch)];

Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];

R = Rz * Ry * Rx;


% Calculate Transformation Matrix
T = [R, [x; y; z];
     0, 0, 0, 1];

end
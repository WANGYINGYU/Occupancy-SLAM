function [Pose3D,Odom3D] = FuncCreate3DPoseFrom2D(Pose2D,Odom)

NumPose = size(Pose2D,1);

Pose3D = zeros(NumPose,6);
Odom3D = zeros(NumPose-1,6);

Pose3D(:,1:2) = Pose2D(:,1:2);
Pose3D(:,end) = Pose2D(:,end);

Odom3D(:,1:2) = Odom(2:end,1:2);
Odom3D(:,end) = Odom(2:end,3);


end
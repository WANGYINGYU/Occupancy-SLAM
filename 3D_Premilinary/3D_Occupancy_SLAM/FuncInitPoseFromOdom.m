function Pose = FuncInitPoseFromOdom(Odom)

NumPose = size(Odom,1);

Pose = zeros(NumPose,)

for i=1:NumPose
    if i==1
        Pose(i,:) = Odom(i,:);
    else




end   


end
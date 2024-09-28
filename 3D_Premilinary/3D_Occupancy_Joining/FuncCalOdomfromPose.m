function Odom = FuncCalOdomfromPose(Pose)

NumPose = size(Pose,1);
Odom = zeros(NumPose,6);

for i=1:NumPose
    if i==1
        Odom(i,:) = Pose(i,:);
    else
        prevPose = Pose(i-1,:);
        currentPose = Pose(i,:);
        
        R_Prev = eul2rotm(prevPose(4:6),"XYZ");
        delta_Trans = R_Prev'*(currentPose(1:3) - prevPose(1:3))';
        delta_Rot = currentPose(4:6) - prevPose(4:6);

        for j=1:3
            delta_Rot(j) = wrap(delta_Rot(j));
        end

        Odom(i,1:3) = delta_Trans;
        Odom(i,4:6) = delta_Rot;
    end    

end    
Odom(1,:) = [];

end
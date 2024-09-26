function Odom = FuncCalOdomfromPose(Pose)

NumPose = size(Pose,1);
Odom = zeros(NumPose,6);

for i=1:NumPose
    if i==1
        Odom(i,:) = Pose(i,:);
    else
        prevPose = Pose(i-1,:);
        currentPose = Pose(i,:);

        T_prev = FuncTFMatrix(prevPose);
        T_current = FuncTFMatrix(currentPose);

        T_relative = inv(T_prev) * T_current;
        delta_Trans = T_relative(1:3,4)';

        R_relative = T_relative(1:3,1:3);
        [delta_r,delta_p,delta_y]= FuncRotMatrix2Euler(R_relative);

        Odom(i,1:3) = delta_Trans;
        Odom(i,4:6) = [delta_r,delta_p,delta_y];
    end    

end    


end
function [GlobalMap,CoordinatePose] = FuncMapJoiningGlobal2LocalNLLS(GlobalMap,SubMap,CoordinatePose,AllPose,PoseOdom,LocalObs,OriginScan,TrajectoryGT,Timestamps,Param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve Non-linear Least Square Problem for Occupancy Submap Joining
% Problem (Global to Local Projection Form)
% 1) Generate Observations of Submap Joining Problem by Projecting Global
% Map to Local Maps (in FuncDiffSubmapJoiningJacobian)
% 2) Calculate Jacobians (in FuncDiffSubmapJoiningJacobian)
% 3) Calculate Delta for Poses Using Equivalent Pose-Only G-N Algorithm (in FuncPoseOnlyDelta)
% 4) Update Poses 
% 5) Re-calculate GlobalMap.N Using Updated Poses
% 6) Check If Pose-Only G-N Algorithm is Equivalent to Standard G-N
% Algorithm (Done)
% Code Witten by Yingyu Wang
% 02/06/2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Iter = 1;
MaxIter = Param.MaxIter;
MinPoseDelta = Param.MinPoseDelta;
MeanPoseDelta = 100;


DividePlan = Param.DividePlan;

% Calculate Odometry between Submaps from Poses 
Odom = zeros(size(DividePlan,2),6);
for i=1:size(DividePlan,2)
    if i==1
        CalPose = [PoseOdom(1,:);PoseOdom(DividePlan(1),:)];
    else
        CalPose = [PoseOdom(DividePlan(i-1),:);PoseOdom(DividePlan(i),:)];
    end
    Odom(i,:) = FuncCalOdomfromPose(CalPose);
end
Odom = [zeros(1,6);Odom];


[~,Trajectory] = FuncUpdateAllPoses(CoordinatePose,AllPose,Param);

FuncEvaluatePose(Trajectory,TrajectoryGT,Param);

while Iter <= MaxIter && MeanPoseDelta>= MinPoseDelta 
    fprintf("Iter Time of Submap Joining is %i\n\n", Iter);
    [JP,JM,JO,ErrorS,ErrorO,OccVal,MeanErrorObs] = FuncDiffSubmapJoiningJacobianUneven2(GlobalMap,SubMap,CoordinatePose,Odom,Param);
    [DeltaP,DeltaM,Mean_Delta,MeanPoseDelta] = FuncDelta(JP,JM,JO,ErrorS,ErrorO,GlobalMap,Param); 
    [GlobalMap,CoordinatePose] = FuncUpdate3D(GlobalMap,CoordinatePose,DeltaP,DeltaM);
    [GlobalMap,Param] = FuncUpdateUnevenGlobalN(GlobalMap,CoordinatePose,LocalObs,Param);

    GlobalMap = FuncMapGrid(GlobalMap);
    
    [~,Trajectory] = FuncUpdateAllPoses(CoordinatePose,AllPose,Param);

    FuncEvaluatePose(Trajectory,TrajectoryGT,Param);

    Iter = Iter + 1;

    FuncDrawTrajectory(Trajectory,TrajectoryGT,PoseOdom,4);

end


end

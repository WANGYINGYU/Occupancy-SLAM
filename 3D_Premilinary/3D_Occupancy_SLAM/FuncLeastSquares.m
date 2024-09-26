function [Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Scan,OriginalScan,TrajectoryGT,Timestamps,Iter,Param)

HH = FuncMapConst3D(Map);
SmoothWeight = Param.SmoothWeight;
HH2 = SmoothWeight*HH;
MaxIter = Param.MaxIter;

MeanDeltaPose = 100;
MeanError = 100;

PoseOdom = Pose;

while Iter<=MaxIter && MeanDeltaPose >= Param.PoseThreshold && MeanError >= Param.ObsThreshold
    fprintf("Iter Time is %i\n\n",Iter);
    [ErrorS,ErrorO,MeanError,JP,JM,JO] = FuncDiffJacobian(Map,Pose,Scan,Odom);
    [DeltaP,DeltaM,MeanDelta,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,Map,HH2,Param);
    clear JP;
    clear JM;
    clear JO;
    [Map,Pose] = FuncUpdate3D(Map,Pose,DeltaP,DeltaM);
    FuncEvaluatePose(Pose,PoseGT,Param);    
    FuncShowOccupancyMap(OriginalScan,Pose,2,Param.MaxRange,2);
    FuncShowPointCloud(Pose,OriginalScan,5);
    if mod(Iter,5)==0
        Param.SmoothWeight = SmoothWeight/10;
        HH2 = SmoothWeight*HH;
    end
    FuncDrawTrajectory(Pose,TrajectoryGT,PoseOdom,4);    
    Iter= Iter + 1;
    if Iter<=MaxIter || MeanDeltaPose >= Param.PoseThreshold || MeanError >= Param.ObsThreshold
        Map = FuncUpdateMapNNorm(Map,Pose,Scan);
        Map = FuncSmoothN2(Map,SmoothWeight,HH);
        Map = FuncMapGrid(Map);
    end
end 

end
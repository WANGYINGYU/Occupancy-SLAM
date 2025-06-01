function [Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Scan,OriginalScan,Iter,Param)

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
    [DeltaP,DeltaM,~,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,Map,HH2,Param);
    fprintf("Mean Pose Delta is %3d\n\n",MeanDeltaPose);
    clear JP;
    clear JM;
    clear JO;
    [Map,Pose] = FuncUpdate3D(Map,Pose,DeltaP,DeltaM);
    if Param.Evaluation
        FuncEvaluatePose(Pose,PoseGT,Param);
        FuncDrawTrajectory(Pose,TrajectoryGT,PoseOdom,4);    
    end
    FuncShowPointCloud(Pose,OriginalScan,Iter,Param);
    if Param.Visualization
        FuncShowOccupancyMap(OriginalScan,Pose,2,Param.MaxRange,2);
    end
    % Smoothing weight tuning
    if mod(Iter,5)==0
        Param.SmoothWeight = SmoothWeight/10;
        HH2 = SmoothWeight*HH;
    end
    Iter= Iter + 1;
    if Iter<=MaxIter || MeanDeltaPose >= Param.PoseThreshold || MeanError >= Param.ObsThreshold
        Map = FuncUpdateMapN(Map,Pose,Scan);
        Map = FuncSmoothN2(Map,SmoothWeight,HH);
        Map = FuncMapGrid(Map);
    end
end 

end
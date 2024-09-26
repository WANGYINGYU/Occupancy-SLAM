function Pose = FuncSelectLeastSquares(SelectMap,Pose,Odom,PoseGT,SelectObs,OriginalScan,TrajectoryGT,Timestamps,Iter,Param)

[SelectHH,Param] = FuncSelectMapConst3D(SelectMap,Param);
SmoothWeight = Param.SmoothWeight;
SelectHH2 = SmoothWeight*SelectHH;
MaxIter = Param.MaxIter;

MeanDeltaPose = 100;
MeanError = 100;

% Iter = 1;

PoseOdom = Pose;

% load T_Odom_GT.mat;
% Pose_Odom_Transfer = (R_est * Pose(:,1:3)' + t_est)';

while Iter<=MaxIter && MeanDeltaPose >= Param.PoseThreshold && MeanError >= Param.ObsThreshold
    fprintf("Iter Time is %i\n\n",Iter);
    SelectMap = FuncSmoothSelectN2(SelectMap,SelectHH,Param);
    SelectMap = FuncSelectMapGrid(SelectMap,Param);
    [ErrorS,ErrorO,Mean_Error,JP,JM,JO] = FuncDiffSelectJacobian(SelectMap,Pose,SelectObs,Odom,Param);
    [DeltaP,DeltaM,MeanDelta,MeanDeltaPose] = Func6DoFSelectDelta(JP,JM,JO,ErrorS,ErrorO,SelectMap,SelectHH2,Param);
    clear JP;
    clear JM;
    clear JO;
    [SelectMap,Pose] = FuncUpdateSelect3D(SelectMap,Pose,DeltaP,DeltaM,Param);

    figure(3)
    ab = SelectMap.Grid(:,:,10);
    exp_ab = exp(ab);
    img = exp_ab./(1+exp_ab);
    imshow(1-img)

    FuncEvaluatePose(Pose,PoseGT,Param);

    mean(abs(PoseGT - Pose))

    fprintf("Pose Delta is %4f\n\n", MeanDeltaPose);

    fileName = sprintf('Pose_%d.txt', Iter);
    SavePoseDict = fullfile(Param.FileDict, fileName);
    FuncSavePose2TUM(Pose,Timestamps,SavePoseDict);

    Iter= Iter + 1;
    
    if Iter<=MaxIter || MeanDeltaPose >= Param.PoseThreshold || MeanError >= Param.ObsThreshold
        SelectMap = FuncUpdateSelectMapN(SelectMap,Pose,SelectObs);
    end
    
    % figure(3)
    % plot(Pose(:,1),Pose(:,2),'b');
    % hold on
    % plot(PoseGT(:,1),PoseGT(:,2),'r')
    % hold off
    
    % FuncShowOccupancyMap(OriginalScan,Pose,2,Param.MaxRange,2);

    % Pose_Transfer = (R_est * Pose(:,1:3)' + t_est)';
    % FuncDrawTrajectory(Pose_Transfer,TrajectoryGT,Pose_Odom_Transfer,4);
    FuncDrawTrajectory(Pose,TrajectoryGT,PoseOdom,4);

   
    
end 

end
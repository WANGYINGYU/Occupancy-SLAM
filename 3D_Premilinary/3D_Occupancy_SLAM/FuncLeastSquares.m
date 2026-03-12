function [Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Scan,OriginalScan,Iter,Param)

UseTruncatedHH = isfield(Param,'UseTruncatedRegionOptimization') && Param.UseTruncatedRegionOptimization;
if UseTruncatedHH
    [Scan, ClipStats] = FuncPreclipObsByTruncatedRegion(Scan, Pose, Map, Param);
    if isfield(Param,'VoxelVerbose') && Param.VoxelVerbose && ClipStats.NumInput > 0
        fprintf('[TruncObs][OneShot] in=%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
                ClipStats.NumInput, ClipStats.NumOutput, ClipStats.CompressionRatio, 100*ClipStats.ReductionRatio);
    end
end

if UseTruncatedHH
    HH = [];
else
    HH = FuncMapConst3D(Map);
end

SmoothWeight = Param.SmoothWeight;
if ~isempty(HH)
    HH2 = SmoothWeight * HH;
else
    HH2 = [];
end

MaxIter = Param.MaxIter;
MeanDeltaPose = 100;
MeanError = 100;

while Iter<=MaxIter && MeanDeltaPose >= Param.PoseThreshold && MeanError >= Param.ObsThreshold
    fprintf('Iter Time is %i\n\n',Iter);

    [ErrorS,ErrorO,MeanError,JP,JM,JO,MapVarId] = FuncDiffJacobian(Map,Pose,Scan,Odom,Param);

    HHLocal = [];
    RegEHOffset = [];
    if ~isempty(MapVarId)
        if UseTruncatedHH
            [HHLocal, BoundaryTerm] = FuncMapConst3DByVarId(MapVarId, Map.Size_i, Map.Size_j, Map.Size_h, Map.Grid);
            HHi = SmoothWeight * HHLocal;
            RegEHOffset = SmoothWeight * BoundaryTerm;
        else
            HHi = HH2(MapVarId,MapVarId);
        end
    else
        if UseTruncatedHH
            HHi = sparse(size(JM,2), size(JM,2));
        else
            HHi = HH2;
        end
    end

    [DeltaP,DeltaM,~,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,Map,HHi,Param,MapVarId,RegEHOffset);
    fprintf('Mean Pose Delta is %3d\n\n',MeanDeltaPose);

    clear JP JM JO;

    [Map,Pose] = FuncUpdate3D(Map,Pose,DeltaP,DeltaM,MapVarId);

    if isfield(Param,'Evaluation') && Param.Evaluation
        FuncEvaluatePose(Pose,PoseGT,Param);
        FuncDrawTrajectory(Pose,PoseGT,4);
    end

    if isfield(Param,'VisualizationOGM') && Param.VisualizationOGM
        FuncShowOccupancyMap(OriginalScan,Pose,2,Param.MaxRange,2);
    end

    if mod(Iter,5)==0
        SmoothWeight = SmoothWeight/10;
        if ~isempty(HH)
            HH2 = SmoothWeight * HH;
        end
    end

    Iter = Iter + 1;

    if Iter<=MaxIter || MeanDeltaPose >= Param.PoseThreshold || MeanError >= Param.ObsThreshold
        Map = FuncUpdateMapNNorm(Map,Pose,Scan);
        if UseTruncatedHH && ~isempty(MapVarId)
            if isempty(HHLocal)
                [HHLocal, ~] = FuncMapConst3DByVarId(MapVarId, Map.Size_i, Map.Size_j, Map.Size_h);
            end
            [~, NBoundaryTerm] = FuncMapConst3DByVarId(MapVarId, Map.Size_i, Map.Size_j, Map.Size_h, Map.N);
            Map = FuncSmoothN2Subset(Map,SmoothWeight,HHLocal,MapVarId,NBoundaryTerm);
        elseif ~isempty(HH)
            Map = FuncSmoothN2(Map,SmoothWeight,HH);
        end
        Map = FuncMapGrid(Map);
    end
end

end

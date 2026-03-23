function [Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Scan,OriginalScan,Iter,Param)

UseTruncatedHH = isfield(Param,'UseTruncatedRegionOptimization') && Param.UseTruncatedRegionOptimization;
UseCoarseOccStabilityFilter = isfield(Param,'UseCoarseOccupancyStabilityFilter') && Param.UseCoarseOccupancyStabilityFilter;
ReprocessObsEachIter = isfield(Param,'ReprocessObsEachIter') && Param.ReprocessObsEachIter;

if UseCoarseOccStabilityFilter && ReprocessObsEachIter && (~iscell(OriginalScan) || isempty(OriginalScan))
    error('UseCoarseOccupancyStabilityFilter requires raw OriginalScan cell input for per-iteration reprocessing.');
end

UseBlockHashInTrunc = false;
if UseTruncatedHH && isfield(Param,'UseBlockHashMapInTruncatedMode') && Param.UseBlockHashMapInTruncatedMode
    UseBlockHashInTrunc = true;
end

FixedRegion = [];
HasFixedRegion = true;
FixedTruncVarId = [];
UseFixedTruncVarId = false;

if UseTruncatedHH && UseBlockHashInTrunc
    Map = FuncDenseMapToBlockHash(Map, Param);
    [FixedRegion, HasFixedRegion] = FuncBlockHashGetOccupiedBBox(Map, Param);
    if HasFixedRegion
        MapClip = FuncBlockHashExtractDenseRegion(Map, FixedRegion);
        MapClip = FuncMapGrid(MapClip);
        FixedTruncVarId = FuncFindCellOptimized(MapClip, Param);
        UseFixedTruncVarId = true;
        ParamClipOneShot = Param;
        ParamClipOneShot.FixedTruncatedVarId = FixedTruncVarId;
        [Scan, ClipStats] = FuncPreclipObsByTruncatedRegion(Scan, Pose, MapClip, ParamClipOneShot);
        if isfield(Param,'VoxelVerbose') && Param.VoxelVerbose && ClipStats.NumInput > 0
            fprintf('[TruncObs][OneShot] in=%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
                    ClipStats.NumInput, ClipStats.NumOutput, ClipStats.CompressionRatio, 100*ClipStats.ReductionRatio);
        end
    else
        for ii = 1:length(Scan)
            Scan{ii}.xyz = zeros(0,3);
            Scan{ii}.Odd = zeros(0,1);
        end
    end
elseif UseTruncatedHH
    FixedTruncVarId = FuncFindCellOptimized(Map, Param);
    UseFixedTruncVarId = true;
    ParamClipOneShot = Param;
    ParamClipOneShot.FixedTruncatedVarId = FixedTruncVarId;
    [Scan, ClipStats] = FuncPreclipObsByTruncatedRegion(Scan, Pose, Map, ParamClipOneShot);
    if isfield(Param,'VoxelVerbose') && Param.VoxelVerbose && ClipStats.NumInput > 0
        fprintf('[TruncObs][OneShot] in=%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
                ClipStats.NumInput, ClipStats.NumOutput, ClipStats.CompressionRatio, 100*ClipStats.ReductionRatio);
    end
end

if UseTruncatedHH || UseBlockHashInTrunc
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
MeanError = inf; % stop metric: scan-only mean squared residual
PrevScanOnlyMeanError = inf;
DecreaseCount = 0;
DecreaseSum = 0;
ScanOnlyDecreaseWindow = 3;
if isfield(Param,'ScanOnlyDecreaseWindow') && Param.ScanOnlyDecreaseWindow >= 1
    ScanOnlyDecreaseWindow = floor(Param.ScanOnlyDecreaseWindow);
end
ScanOnlyDecreaseTol = 1e-4;
if isfield(Param,'ScanOnlyDecreaseTol') && Param.ScanOnlyDecreaseTol >= 0
    ScanOnlyDecreaseTol = Param.ScanOnlyDecreaseTol;
end

ParamSolve = Param;
[ResolvedOdomSigma, OdomSigmaInfo] = FuncResolveOdomSigma(Param, Pose);
ParamSolve.OdomSigma = ResolvedOdomSigma;
if isfield(Param,'VoxelVerbose') && Param.VoxelVerbose && OdomSigmaInfo.Auto
    fprintf('[OdomSigma][Auto] trans=[%.4f %.4f %.4f] m, rot=[%.3f %.3f %.3f] deg, leverage=%.3f m\n', ...
        ResolvedOdomSigma(1), ResolvedOdomSigma(2), ResolvedOdomSigma(3), ...
        ResolvedOdomSigma(4)*180/pi, ResolvedOdomSigma(5)*180/pi, ResolvedOdomSigma(6)*180/pi, ...
        OdomSigmaInfo.LeverageLength);
end

while Iter<=MaxIter && MeanDeltaPose >= Param.PoseThreshold && MeanError >= Param.ObsThreshold
    fprintf('Iter Time is %i\n\n',Iter);

    HasRegion = true;
    Region = [];
    if UseTruncatedHH && UseBlockHashInTrunc
        Region = FixedRegion;
        HasRegion = HasFixedRegion;
        if HasRegion
            MapWork = FuncBlockHashExtractDenseRegion(Map, Region);
            MapWork = FuncMapGrid(MapWork);
        else
            MapWork = LocalBuildEmptyWorkingMap(Map);
        end
    else
        MapWork = Map;
    end
    ParamIter = ParamSolve;
    if UseTruncatedHH && UseFixedTruncVarId
        ParamIter.FixedTruncatedVarId = FixedTruncVarId;
    end
    [ErrorS,ErrorO,MeanError,JP,JM,JO,MapVarId] = FuncDiffJacobian(MapWork,Pose,Scan,Odom,ParamIter);
    if MeanError < PrevScanOnlyMeanError
        DecreaseCount = DecreaseCount + 1;
        DecreaseSum = DecreaseSum + (PrevScanOnlyMeanError - MeanError);
        if DecreaseCount >= ScanOnlyDecreaseWindow
            AvgDecrease = DecreaseSum / DecreaseCount;
            if AvgDecrease < ScanOnlyDecreaseTol
                fprintf('[Converged] stop: avg scan_only decrease over %d consecutive decreasing iterations is below threshold (avg=%.6e, tol=%.6e)\n', ...
                        DecreaseCount, AvgDecrease, ScanOnlyDecreaseTol);
                break;
            end
        end
    else
        DecreaseCount = 0;
        DecreaseSum = 0;
    end
    PrevScanOnlyMeanError = MeanError;

    HHLocal = [];
    RegEHOffset = [];
    if ~isempty(MapVarId)
        if UseTruncatedHH
            [HHLocal, BoundaryTerm] = FuncMapConst3DByVarId(MapVarId, MapWork.Size_i, MapWork.Size_j, MapWork.Size_h, MapWork.Grid);
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

    [DeltaP,DeltaM,~,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,MapWork,HHi,ParamSolve,MapVarId,RegEHOffset);
    fprintf('Mean Pose Delta is %3d\n\n',MeanDeltaPose);

    clear JP JM JO;

    [MapWork,Pose] = FuncUpdate3D(MapWork,Pose,DeltaP,DeltaM,MapVarId);

    if isfield(Param,'Evaluation') && Param.Evaluation
        FuncEvaluatePose(Pose,PoseGT,Param);
        FuncDrawTrajectory(Pose,PoseGT,4);
    end

    if UseCoarseOccStabilityFilter && ReprocessObsEachIter
        ParamObsIter = Param;
        if isfield(ParamObsIter,'UseTemporalRangeConsistencyFilter')
            ParamObsIter.UseTemporalRangeConsistencyFilter = 0;
        end
        Scan = FuncEqualProcessObs(OriginalScan,Pose,ParamObsIter);

        if UseTruncatedHH
            ParamClipIter = Param;
            if UseFixedTruncVarId
                ParamClipIter.FixedTruncatedVarId = FixedTruncVarId;
            end
            [Scan, ClipStatsIter] = FuncPreclipObsByTruncatedRegion(Scan, Pose, MapWork, ParamClipIter);
            if isfield(Param,'VoxelVerbose') && Param.VoxelVerbose && ClipStatsIter.NumInput > 0
                fprintf('[TruncObs][Iter] in=%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
                        ClipStatsIter.NumInput, ClipStatsIter.NumOutput, ClipStatsIter.CompressionRatio, 100*ClipStatsIter.ReductionRatio);
            end
        end
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
        MapWork = FuncUpdateMapNNorm(MapWork,Pose,Scan);
        if UseTruncatedHH && ~isempty(MapVarId)
            if isempty(HHLocal)
                [HHLocal, ~] = FuncMapConst3DByVarId(MapVarId, MapWork.Size_i, MapWork.Size_j, MapWork.Size_h);
            end
            [~, NBoundaryTerm] = FuncMapConst3DByVarId(MapVarId, MapWork.Size_i, MapWork.Size_j, MapWork.Size_h, MapWork.N);
            MapWork = FuncSmoothN2Subset(MapWork,SmoothWeight,HHLocal,MapVarId,NBoundaryTerm,Param);
        elseif ~isempty(HH)
            MapWork = FuncSmoothN2(MapWork,SmoothWeight,HH,Param);
        end
        MapWork = FuncMapGrid(MapWork);
    end

    if UseTruncatedHH && UseBlockHashInTrunc
        if HasRegion
            Map = FuncBlockHashMergeDenseRegion(Map, MapWork, Region);
        end
    else
        Map = MapWork;
    end
end

end

function MapWork = LocalBuildEmptyWorkingMap(Map)
MapWork = struct();
MapWork.Grid = zeros(1,1,1);
MapWork.N = zeros(1,1,1);
MapWork.Scale = Map.Scale;
MapWork.Origin = Map.Origin;
MapWork.Size_i = 1;
MapWork.Size_j = 1;
MapWork.Size_h = 1;
if isfield(Map,'WorldToMapR')
    MapWork.WorldToMapR = Map.WorldToMapR;
end
if isfield(Map,'WorldToMapT')
    MapWork.WorldToMapT = Map.WorldToMapT;
end
if isfield(Map,'MapFrameStats')
    MapWork.MapFrameStats = Map.MapFrameStats;
end
MapWork = FuncMapGrid(MapWork);
end

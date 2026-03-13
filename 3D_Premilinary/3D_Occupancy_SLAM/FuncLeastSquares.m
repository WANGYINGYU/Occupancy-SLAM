function [Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Scan,OriginalScan,Iter,Param)

UseTruncatedHH = isfield(Param,'UseTruncatedRegionOptimization') && Param.UseTruncatedRegionOptimization;

UseBlockHashInTrunc = false;
if UseTruncatedHH && isfield(Param,'UseBlockHashMapInTruncatedMode') && Param.UseBlockHashMapInTruncatedMode
    UseBlockHashInTrunc = true;
end

FixedRegion = [];
HasFixedRegion = true;

if UseTruncatedHH && UseBlockHashInTrunc
    Map = FuncDenseMapToBlockHash(Map, Param);
    [FixedRegion, HasFixedRegion] = FuncBlockHashGetOccupiedBBox(Map, Param);
    if HasFixedRegion
        MapClip = FuncBlockHashExtractDenseRegion(Map, FixedRegion);
        MapClip = FuncMapGrid(MapClip);
        [Scan, ClipStats] = FuncPreclipObsByTruncatedRegion(Scan, Pose, MapClip, Param);
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
    [Scan, ClipStats] = FuncPreclipObsByTruncatedRegion(Scan, Pose, Map, Param);
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
MeanError = 100;

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

    [ErrorS,ErrorO,MeanError,JP,JM,JO,MapVarId] = FuncDiffJacobian(MapWork,Pose,Scan,Odom,Param);

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

    [DeltaP,DeltaM,~,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,MapWork,HHi,Param,MapVarId,RegEHOffset);
    fprintf('Mean Pose Delta is %3d\n\n',MeanDeltaPose);

    clear JP JM JO;

    [MapWork,Pose] = FuncUpdate3D(MapWork,Pose,DeltaP,DeltaM,MapVarId);

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
        MapWork = FuncUpdateMapNNorm(MapWork,Pose,Scan);
        if UseTruncatedHH && ~isempty(MapVarId)
            if isempty(HHLocal)
                [HHLocal, ~] = FuncMapConst3DByVarId(MapVarId, MapWork.Size_i, MapWork.Size_j, MapWork.Size_h);
            end
            [~, NBoundaryTerm] = FuncMapConst3DByVarId(MapVarId, MapWork.Size_i, MapWork.Size_j, MapWork.Size_h, MapWork.N);
            MapWork = FuncSmoothN2Subset(MapWork,SmoothWeight,HHLocal,MapVarId,NBoundaryTerm);
        elseif ~isempty(HH)
            MapWork = FuncSmoothN2(MapWork,SmoothWeight,HH);
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

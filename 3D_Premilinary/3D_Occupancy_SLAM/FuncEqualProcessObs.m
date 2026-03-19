function Obs = FuncEqualProcessObs(Scan,Pose,Param)

if nargin == 2
    Param = Pose;
    Pose = [];
end

NumPose = length(Scan);
MaxRange = Param.MaxRange;
MinRange = Param.MinRange;
UseVoxelDownsample = isfield(Param,'UseVoxelDownsample') && Param.UseVoxelDownsample;
UseAdaptiveVoxelDownsample = isfield(Param,'UseAdaptiveVoxelDownsample') && Param.UseAdaptiveVoxelDownsample;
UseGlobalVoxelDensityFilter = isfield(Param,'UseGlobalVoxelDensityFilter') && Param.UseGlobalVoxelDensityFilter;
UseCoarseOccStabilityFilter = isfield(Param,'UseCoarseOccupancyStabilityFilter') && Param.UseCoarseOccupancyStabilityFilter;
VoxelSize = 0.15;
if isfield(Param,'VoxelSize')
    VoxelSize = Param.VoxelSize;
end
AdaptiveRangeEdges = [0,15,30,inf];
if isfield(Param,'AdaptiveVoxelRangeEdges') && ~isempty(Param.AdaptiveVoxelRangeEdges)
    AdaptiveRangeEdges = Param.AdaptiveVoxelRangeEdges;
end
AdaptiveVoxelSizes = [0.05,0.10,0.15];
if isfield(Param,'AdaptiveVoxelSizes') && ~isempty(Param.AdaptiveVoxelSizes)
    AdaptiveVoxelSizes = Param.AdaptiveVoxelSizes;
end
VoxelMethod = 'centroid';
if isfield(Param,'VoxelMethod') && ~isempty(Param.VoxelMethod)
    VoxelMethod = Param.VoxelMethod;
end
VoxelVerbose = isfield(Param,'VoxelVerbose') && Param.VoxelVerbose;

Obs = cell(1,NumPose);
HitLocal = cell(1,NumPose);
TotalInputValid = 0;
TotalOutput = 0;
for i=1:NumPose
    HitPi = double(Scan{i}(1:end,:));
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];

    Distance = sqrt(HitPi(1,:).^2 + HitPi(2,:).^2 + HitPi(3,:).^2);
    OutId = find((Distance>=MaxRange)|(Distance<=MinRange));
    HitPi(:,OutId) = [];

    if UseAdaptiveVoxelDownsample && ~isempty(HitPi)
        [HitPi, Stats] = FuncRangeAdaptiveVoxelDownsample(HitPi, AdaptiveRangeEdges, AdaptiveVoxelSizes, VoxelMethod, false);
        TotalInputValid = TotalInputValid + Stats.NumInputValid;
        TotalOutput = TotalOutput + Stats.NumOutput;
    elseif UseVoxelDownsample && ~isempty(HitPi) && VoxelSize > 0
        [HitPi, Stats] = FuncVoxelGridDownsample(HitPi, VoxelSize, VoxelMethod, false);
        TotalInputValid = TotalInputValid + Stats.NumInputValid;
        TotalOutput = TotalOutput + Stats.NumOutput;
    end
    HitLocal{i} = HitPi';
end

if UseCoarseOccStabilityFilter
    HasPose = ~isempty(Pose) && size(Pose,1) >= NumPose;
    if HasPose
        [HitLocal, CoarseOccStats] = FuncCoarseOccupancyStabilityFilter(HitLocal, Pose, Param);
        if VoxelVerbose && CoarseOccStats.NumInput > 0
            fprintf('[CoarseOccFilter][Total] in=%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
                    CoarseOccStats.NumInput, CoarseOccStats.NumOutput, ...
                    CoarseOccStats.CompressionRatio, 100*CoarseOccStats.ReductionRatio);
        end
    elseif VoxelVerbose
        fprintf('[CoarseOccFilter] skipped (Pose missing or length mismatch)\n');
    end
end

if UseGlobalVoxelDensityFilter
    HasPose = ~isempty(Pose) && size(Pose,1) >= NumPose;
    if HasPose
        [HitLocal, GlobalFilterStats] = FuncGlobalVoxelDensityFilter(HitLocal, Pose, Param);
        if VoxelVerbose && GlobalFilterStats.NumInput > 0
            fprintf('[GlobalVoxelFilter][Total] in=%d, out=%d, compression=%.4f, reduction=%.2f%%, fallback_pose=%d\n', ...
                    GlobalFilterStats.NumInput, GlobalFilterStats.NumOutput, ...
                    GlobalFilterStats.CompressionRatio, 100*GlobalFilterStats.ReductionRatio, ...
                    GlobalFilterStats.NumFallbackPose);
        end
    elseif VoxelVerbose
        fprintf('[GlobalVoxelFilter] skipped (Pose missing or length mismatch)\n');
    end
end

for i = 1:NumPose
    HitPi = HitLocal{i};
    [AllXYZi,AllOddi] = FuncEqualDistanceSample3DLines(HitPi,Param);
    Obs{i}.HitXYZ = HitPi;
    Obs{i}.xyz = AllXYZi;
    Obs{i}.Odd = AllOddi;
end

if (UseVoxelDownsample || UseAdaptiveVoxelDownsample) && VoxelVerbose && TotalInputValid > 0
    CompressionRatio = TotalOutput / TotalInputValid;
    ReductionRatio = 1 - CompressionRatio;
    fprintf('[VoxelDS][EqualProcessObs][Total] in=%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
            TotalInputValid, TotalOutput, CompressionRatio, 100*ReductionRatio);
end

end

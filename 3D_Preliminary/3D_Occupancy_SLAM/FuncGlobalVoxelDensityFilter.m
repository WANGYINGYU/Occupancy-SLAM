function [HitOut, Stats] = FuncGlobalVoxelDensityFilter(HitIn, Pose, Param)
% Filter global outliers by world-frame voxel density.
% Keep points only in voxels with at least MinPointsPerVoxel points.

NumPose = length(HitIn);
HitOut = HitIn;

Stats = struct('NumInput',0,'NumOutput',0,'CompressionRatio',0,'ReductionRatio',0,'NumFallbackPose',0);
if NumPose == 0
    return;
end

if nargin < 3
    Param = struct();
end

VoxelSize = 0.2;
if isfield(Param,'VoxelSize') && Param.VoxelSize > 0
    VoxelSize = max(0.2, 4 * Param.VoxelSize);
end
MinPointsPerVoxel = 3;

WorldCell = cell(NumPose,1);
LocalCell = cell(NumPose,1);
NumEach = zeros(NumPose,1);

parfor i = 1:NumPose
    Pi = HitIn{i};
    if isempty(Pi)
        WorldCell{i} = zeros(0,3);
        LocalCell{i} = zeros(0,3);
        NumEach(i) = 0;
        continue;
    end

    if size(Pi,2) > 3
        Pi = Pi(:,1:3);
    end
    if size(Pi,2) ~= 3
        WorldCell{i} = zeros(0,3);
        LocalCell{i} = zeros(0,3);
        NumEach(i) = 0;
        continue;
    end

    ValidLocal = all(isfinite(Pi),2);
    Pi = Pi(ValidLocal,:);
    if isempty(Pi)
        WorldCell{i} = zeros(0,3);
        LocalCell{i} = zeros(0,3);
        NumEach(i) = 0;
        continue;
    end

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');

    Wi = (Ri * Pi' + Posei(1:3)')';
    ValidWorld = all(isfinite(Wi),2);
    Pi = Pi(ValidWorld,:);
    Wi = Wi(ValidWorld,:);
    LocalCell{i} = Pi;
    WorldCell{i} = Wi;
    NumEach(i) = size(Wi,1);
end

NumInput = sum(NumEach);
if NumInput == 0
    for i = 1:NumPose
        HitOut{i} = zeros(0,3);
    end
    return;
end

AllWorld = vertcat(WorldCell{:});
Vox = floor(AllWorld / VoxelSize);

ix = int64(Vox(:,1));
iy = int64(Vox(:,2));
iz = int64(Vox(:,3));

ix = ix - min(ix);
iy = iy - min(iy);
iz = iz - min(iz);

DimX = max(ix) + 1;
DimY = max(iy) + 1;
Hash = ix + DimX * (iy + DimY * iz);

[~,~,GroupId] = unique(Hash);
VoxelCount = accumarray(GroupId, 1);
KeepVoxel = VoxelCount >= MinPointsPerVoxel;
KeepAll = KeepVoxel(GroupId);

Offset = [0; cumsum(NumEach)];
NumOutput = 0;
NumFallbackPose = 0;
for i = 1:NumPose
    if NumEach(i) == 0
        HitOut{i} = zeros(0,3);
        continue;
    end
    Idx = (Offset(i)+1):Offset(i+1);
    Keepi = KeepAll(Idx);
    Pi = LocalCell{i};
    Outi = Pi(Keepi,:);
    if isempty(Outi) && ~isempty(Pi)
        % Keep at least one-frame support to avoid completely empty observations.
        Outi = Pi;
        NumFallbackPose = NumFallbackPose + 1;
    end
    HitOut{i} = Outi;
    NumOutput = NumOutput + size(Outi,1);
end

Stats.NumInput = NumInput;
Stats.NumOutput = NumOutput;
Stats.CompressionRatio = NumOutput / max(NumInput,1);
Stats.ReductionRatio = 1 - Stats.CompressionRatio;
Stats.NumFallbackPose = NumFallbackPose;
end

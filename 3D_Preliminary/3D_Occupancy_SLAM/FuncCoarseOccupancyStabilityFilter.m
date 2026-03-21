function [HitOut, Stats] = FuncCoarseOccupancyStabilityFilter(HitIn, Pose, Param)
% One-shot coarse occupancy filter in world frame:
% remove hit points falling into voxels with strong free-space evidence.
% Each ray contributes to a voxel at most once.

NumPose = length(HitIn);
HitOut = HitIn;
Stats = struct('NumInput',0,'NumOutput',0,'CompressionRatio',0,'ReductionRatio',0);
if NumPose == 0
    return;
end

if nargin < 2 || isempty(Pose) || size(Pose,1) < NumPose
    return;
end
if nargin < 3 || isempty(Param)
    Param = struct();
end

VoxelSize = 1.0;
if isfield(Param,'CoarseOccVoxelSize') && isfinite(Param.CoarseOccVoxelSize) && Param.CoarseOccVoxelSize > 0
    VoxelSize = double(Param.CoarseOccVoxelSize);
end

SampleDistance = 0.1;
if isfield(Param,'SampleDistance') && isfinite(Param.SampleDistance) && Param.SampleDistance > 0
    SampleDistance = double(Param.SampleDistance);
end

ValOddHit = 0.847297860387203;
if isfield(Param,'ValOddHit') && isfinite(Param.ValOddHit)
    ValOddHit = double(Param.ValOddHit);
end
ValOddFree = -0.405465108108164;
if isfield(Param,'ValOddFree') && isfinite(Param.ValOddFree)
    ValOddFree = double(Param.ValOddFree);
end

FreeProbTh = 0.30;
if isfield(Param,'CoarseOccFreeProbThreshold') && isfinite(Param.CoarseOccFreeProbThreshold)
    FreeProbTh = double(Param.CoarseOccFreeProbThreshold);
end
FreeProbTh = min(max(FreeProbTh, 1e-4), 0.4999);
FreeLogOddsTh = log(FreeProbTh / (1 - FreeProbTh));

LocalPts = cell(NumPose,1);
HitIdxCells = cell(NumPose,1);
RayIdxCells = cell(NumPose,1);
RayOddCells = cell(NumPose,1);
NumInput = 0;

for i = 1:NumPose
    Pi = LocalToNx3(HitIn{i});
    if isempty(Pi)
        LocalPts{i} = zeros(0,3);
        HitIdxCells{i} = zeros(0,3,'int32');
        RayIdxCells{i} = zeros(0,3,'int32');
        RayOddCells{i} = zeros(0,1);
        continue;
    end

    Valid = all(isfinite(Pi),2);
    Pi = Pi(Valid,:);
    if isempty(Pi)
        LocalPts{i} = zeros(0,3);
        HitIdxCells{i} = zeros(0,3,'int32');
        RayIdxCells{i} = zeros(0,3,'int32');
        RayOddCells{i} = zeros(0,1);
        continue;
    end

    [R, T] = LocalPoseRT(Pose(i,:));
    PHitW = (R * Pi' + T)';
    HitIdxCells{i} = int32(floor(PHitW ./ VoxelSize));
    LocalPts{i} = Pi;
    NumInput = NumInput + size(Pi,1);

    [RayIdxCells{i}, RayOddCells{i}] = LocalBuildRayVoxelContribFast(Pi, R, T, VoxelSize, SampleDistance, ValOddFree, ValOddHit);
end

if NumInput == 0
    return;
end

NumRayVoxel = 0;
for i = 1:NumPose
    NumRayVoxel = NumRayVoxel + size(RayIdxCells{i},1);
end
if NumRayVoxel == 0
    return;
end

AllRayIdx = zeros(NumRayVoxel,3,'int32');
AllRayOdd = zeros(NumRayVoxel,1);
s = 1;
for i = 1:NumPose
    Ni = size(RayIdxCells{i},1);
    if Ni <= 0
        continue;
    end
    e = s + Ni - 1;
    AllRayIdx(s:e,:) = RayIdxCells{i};
    AllRayOdd(s:e) = RayOddCells{i};
    s = e + 1;
end

[OccIdx,~,GAll] = unique(AllRayIdx,'rows');
OccLogOdds = accumarray(GAll, AllRayOdd, [], @sum);

NumOutput = 0;
for i = 1:NumPose
    Pi = LocalPts{i};
    if isempty(Pi)
        HitOut{i} = zeros(0,3);
        continue;
    end

    IHit = HitIdxCells{i};
    [tf, loc] = ismember(IHit, OccIdx, 'rows');

    Keep = true(size(Pi,1),1);
    if any(tf)
        Li = OccLogOdds(loc(tf));
        Keep(tf) = Li > FreeLogOddsTh;
    end

    Outi = Pi(Keep,:);
    HitOut{i} = Outi;
    NumOutput = NumOutput + size(Outi,1);
end

Stats.NumInput = NumInput;
Stats.NumOutput = NumOutput;
Stats.CompressionRatio = NumOutput / max(NumInput,1);
Stats.ReductionRatio = 1 - Stats.CompressionRatio;

end

function [RayIdx, RayOdd] = LocalBuildRayVoxelContribFast(Pi, R, T, VoxelSize, SampleDistance, ValOddFree, ValOddHit)
% Faster per-ray voxel contribution:
% 1) sample along each ray
% 2) convert to coarse voxel ids
% 3) remove consecutive duplicated voxels (keep last in each run)
% Keeping last ensures terminal voxel gets hit contribution.

NumRay = size(Pi,1);
IdxCell = cell(NumRay,1);
OddCell = cell(NumRay,1);
SensorW = double(T(:)');

for k = 1:NumRay
    p = Pi(k,:);
    Dist = sqrt(sum(p.^2));
    NumSample = fix(Dist / SampleDistance) + 1;
    if NumSample < 1
        NumSample = 1;
    end

    DirW = (R * p')';
    t = (1:NumSample)' / NumSample;
    SegWorld = SensorW + t .* DirW;
    SegIdx = int32(floor(SegWorld ./ VoxelSize));

    if NumSample == 1
        KeepLast = true;
    else
        Change = any(diff(SegIdx,1,1) ~= 0, 2);
        KeepLast = [Change; true];
    end

    Uid = SegIdx(KeepLast,:);
    OddOnce = repmat(ValOddFree, size(Uid,1), 1);
    OddOnce(end) = ValOddHit;

    IdxCell{k} = Uid;
    OddCell{k} = OddOnce;
end

RayIdx = vertcat(IdxCell{:});
RayOdd = vertcat(OddCell{:});
if isempty(RayIdx)
    RayIdx = zeros(0,3,'int32');
    RayOdd = zeros(0,1);
end
end

function [R, T] = LocalPoseRT(Posei)
RX = FuncRX(Posei(4));
RY = FuncRY(Posei(5));
RZ = FuncRZ(Posei(6));
R = FuncR(RZ',RY',RX');
T = Posei(1:3)';
end

function P = LocalToNx3(X)
P = zeros(0,3);
if isempty(X)
    return;
end

X = double(X);
nRow = size(X,1);
nCol = size(X,2);

if nRow >= 3 && nRow <= 8 && nCol > nRow
    P = X(1:3,:)';
elseif nCol >= 3
    P = X(:,1:3);
elseif nRow >= 3
    P = X(1:3,:)';
end
end

function [HitOut, Stats] = FuncCoarseOccupancyStabilityFilter(HitIn, Pose, Param)
% One-shot coarse occupancy filter in world frame:
% remove hit points falling into voxels with strong free-space evidence.

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

    [RayLocal, RayOdd] = FuncEqualDistanceSample3DLines(Pi, Param);
    if isempty(RayLocal)
        RayIdxCells{i} = zeros(0,3,'int32');
        RayOddCells{i} = zeros(0,1);
        continue;
    end

    PRayW = (R * RayLocal' + T)';
    IRay = int32(floor(PRayW ./ VoxelSize));
    [Uid,~,G] = unique(IRay,'rows');
    SumOdd = accumarray(G, double(RayOdd), [], @sum);
    RayIdxCells{i} = Uid;
    RayOddCells{i} = SumOdd;
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

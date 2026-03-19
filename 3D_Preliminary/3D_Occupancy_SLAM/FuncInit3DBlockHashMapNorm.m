function [Map,Param] = FuncInit3DBlockHashMapNorm(Pose,Scan,Param)
% Native block-hash map initialization for truncated mode.
% This path avoids constructing full dense Grid/N.

NumPose = size(Pose,1);
Scale = Param.Scale;
[WorldToMapR, WorldToMapT, MapFrameStats] = FuncEstimateMapFrameTransform(Pose, Scan, Param);
Param.WorldToMapR = WorldToMapR;
Param.WorldToMapT = WorldToMapT;

BlockSize = 16;
if isfield(Param,'BlockHashBlockSize') && Param.BlockHashBlockSize > 0
    BlockSize = floor(Param.BlockHashBlockSize);
end

% Fast bound estimation: rotate local AABB corners (8 points) per scan
% instead of transforming all scan points in pass-1.
UseAABBCornerBound = 1;
if isfield(Param,'BlockHashInitUseAABBCorners')
    UseAABBCornerBound = Param.BlockHashInitUseAABBCorners ~= 0;
end
InitVerbose = isfield(Param,'VoxelVerbose') && Param.VoxelVerbose;
if InitVerbose && MapFrameStats.Enabled
    fprintf('[MapFrame] enabled: mode=yaw_pca, yaw=%.3f deg, pts=%d\n', ...
        MapFrameStats.YawRad * 180 / pi, MapFrameStats.NumWorldPoints);
end

% Pass-1: bound estimation only.
tAll = tic;
tPass1 = tic;
MinXYZ = [inf; inf; inf];
MaxXYZ = [-inf; -inf; -inf];

RotCell = cell(NumPose,1);
for i = 1:NumPose
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    RotCell{i} = FuncR(RZ',RY',RX');
end

for i = 1:NumPose
    Posei = Pose(i,:);
    Ri = RotCell{i};

    xyzRaw = double(Scan{i}.xyz);
    if isempty(xyzRaw)
        continue;
    end

    if UseAABBCornerBound
        minx = min(xyzRaw(:,1)); maxx = max(xyzRaw(:,1));
        miny = min(xyzRaw(:,2)); maxy = max(xyzRaw(:,2));
        minz = min(xyzRaw(:,3)); maxz = max(xyzRaw(:,3));
        C = [minx, maxx, minx, maxx, minx, maxx, minx, maxx; ...
             miny, miny, maxy, maxy, miny, miny, maxy, maxy; ...
             minz, minz, minz, minz, maxz, maxz, maxz, maxz];
        Si = Ri*C + Posei(1:3)';
    else
        Si = Ri*xyzRaw' + Posei(1:3)';
    end
    Si = FuncWorldToMapFrame(Si, WorldToMapR, WorldToMapT);

    MinXYZ = min(MinXYZ, min(Si,[],2));
    MaxXYZ = max(MaxXYZ, max(Si,[],2));
end
tPass1Sec = toc(tPass1);

if ~all(isfinite(MinXYZ)) || ~all(isfinite(MaxXYZ))
    Origin = [-fix(1/Scale); -fix(1/Scale); -fix(4/Scale)];
    Size_i = max(1, fix(2/Scale));
    Size_j = max(1, fix(2/Scale));
    Size_h = max(1, fix(8/Scale));
else
    min_x = MinXYZ(1); min_y = MinXYZ(2); min_z = MinXYZ(3);
    if min_x <= 1e-6
        int_minx = floor(min_x);
    else
        int_minx = fix(min_x);
    end
    if min_y <= 1e-6
        int_miny = floor(min_y);
    else
        int_miny = fix(min_y);
    end
    if min_z <= 1e-6
        int_minz = floor(min_z);
    else
        int_minz = fix(min_z);
    end

    Origin = [int_minx - fix(1/Scale); int_miny - fix(1/Scale); int_minz - fix(4/Scale)];
    XYZ3Max = (MaxXYZ-Origin) / Scale + 1;
    max_x = round(XYZ3Max(1));
    max_y = round(XYZ3Max(2));
    max_z = round(XYZ3Max(3));

    Size_i = max_y + fix(2/Scale);
    Size_j = max_x + fix(2/Scale);
    Size_h = max_z + fix(8/Scale);
end

Param.Origin = Origin;
Param.Size_i = Size_i;
Param.Size_j = Size_j;
Param.Size_h = Size_h;

KeyToIdx = containers.Map('KeyType','uint64','ValueType','uint32');
BlockGrid = {};
BlockN = {};
BlockCount = uint32(0);

% Pass-2: block-hash accumulation directly from observations.
tPass2 = tic;
for i = 1:NumPose
    Posei = Pose(i,:);
    Ri = RotCell{i};

    xyz = double(Scan{i}.xyz');
    Oddi = double(Scan{i}.Odd(:));
    if isempty(xyz) || isempty(Oddi)
        continue;
    end
    nPts = min(size(xyz,2), length(Oddi));
    if nPts <= 0
        continue;
    end
    xyz = xyz(:,1:nPts);
    Oddi = Oddi(1:nPts);

    Si = Ri*xyz + Posei(1:3)';
    Si = FuncWorldToMapFrame(Si, WorldToMapR, WorldToMapT);
    XYZ3 = (Si-Origin) / Scale + 1;
    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));

    Valid = x >= 1 & x <= Size_j & ...
            y >= 1 & y <= Size_i & ...
            z >= 1 & z <= Size_h;
    if ~any(Valid)
        continue;
    end

    x = x(Valid);
    y = y(Valid);
    z = z(Valid);
    Oddi = Oddi(Valid(:));
    if isempty(Oddi)
        continue;
    end

    ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;
    [UniInd, ~, Ic] = unique(ind(:));
    HitCnt = accumarray(Ic, 1);
    SumOdd = accumarray(Ic, Oddi(:));
    MeanOdd = SumOdd ./ HitCnt;

    [uy, ux, uz] = ind2sub([Size_i, Size_j, Size_h], UniInd);
    bi = floor((uy-1)/BlockSize) + 1;
    bj = floor((ux-1)/BlockSize) + 1;
    bh = floor((uz-1)/BlockSize) + 1;

    BlockKey = LocalMakeKeyNum(bi, bj, bh);
    [BlockKeySorted, IdSort] = sort(BlockKey);
    uy = uy(IdSort); ux = ux(IdSort); uz = uz(IdSort);
    bi = bi(IdSort); bj = bj(IdSort); bh = bh(IdSort);
    MeanOdd = MeanOdd(IdSort);

    SplitPos = [1; find(diff(BlockKeySorted) ~= 0) + 1; numel(BlockKeySorted) + 1];
    for b = 1:(length(SplitPos)-1)
        s = SplitPos(b);
        e = SplitPos(b+1) - 1;
        Key = BlockKeySorted(s);
        bii = bi(s);
        bjj = bj(s);
        bhh = bh(s);

        if isKey(KeyToIdx, Key)
            Idx = double(KeyToIdx(Key));
        else
            BlockCount = BlockCount + 1;
            Idx = double(BlockCount);
            KeyToIdx(Key) = uint32(Idx);

            r0 = (bii-1)*BlockSize + 1; r1 = min(bii*BlockSize, Size_i);
            c0 = (bjj-1)*BlockSize + 1; c1 = min(bjj*BlockSize, Size_j);
            h0 = (bhh-1)*BlockSize + 1; h1 = min(bhh*BlockSize, Size_h);
            BlockGrid{Idx,1} = zeros(r1-r0+1, c1-c0+1, h1-h0+1); %#ok<AGROW>
            BlockN{Idx,1} = zeros(r1-r0+1, c1-c0+1, h1-h0+1); %#ok<AGROW>
        end

        GridBlk = BlockGrid{Idx};
        NBlk = BlockN{Idx};

        r0 = (bii-1)*BlockSize + 1;
        c0 = (bjj-1)*BlockSize + 1;
        h0 = (bhh-1)*BlockSize + 1;

        lr = uy(s:e) - r0 + 1;
        lc = ux(s:e) - c0 + 1;
        lh = uz(s:e) - h0 + 1;
        LinLocal = sub2ind(size(GridBlk), lr, lc, lh);

        NBlk(LinLocal) = NBlk(LinLocal) + 1;
        GridBlk(LinLocal) = GridBlk(LinLocal) + MeanOdd(s:e);

        BlockGrid{Idx} = GridBlk;
        BlockN{Idx} = NBlk;
    end
end
tPass2Sec = toc(tPass2);

Map = struct();
Map.Scale = Scale;
Map.Origin = Origin;
Map.Size_i = Size_i;
Map.Size_j = Size_j;
Map.Size_h = Size_h;
Map.StorageType = 'blocked';
Map.BlockSize = BlockSize;
Map.BlockKeyToIdx = KeyToIdx;
Map.BlockGrid = BlockGrid;
Map.BlockN = BlockN;
Map.WorldToMapR = WorldToMapR;
Map.WorldToMapT = WorldToMapT;
Map.MapFrameStats = MapFrameStats;

if InitVerbose
    fprintf('[BlockHashInit] pass1=%.3fs, pass2=%.3fs, total=%.3fs, blocks=%d\n', ...
        tPass1Sec, tPass2Sec, toc(tAll), numel(BlockGrid));
end

end

function Key = LocalMakeKeyNum(bi,bj,bh)
% Pack 3 positive integer indices into uint64 key.
Key = uint64(bi) + bitshift(uint64(bj), 21) + bitshift(uint64(bh), 42);
end

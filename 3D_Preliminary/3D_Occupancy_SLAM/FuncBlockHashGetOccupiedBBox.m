function [Region, HasRegion] = FuncBlockHashGetOccupiedBBox(Map, Param)
% Compute working dense region from occupied cells in blocked map storage.
% Region is expanded by truncated layers and a small interpolation padding.

HasRegion = false;
Region = struct('RowMin',1,'RowMax',1,'ColMin',1,'ColMax',1,'HMin',1,'HMax',1);

if ~isfield(Map,'StorageType') || ~strcmp(Map.StorageType,'blocked')
    error('FuncBlockHashGetOccupiedBBox requires blocked map storage.');
end

OccThreshold = 0;
if isfield(Param,'TruncatedOccupancyThreshold')
    OccThreshold = Param.TruncatedOccupancyThreshold;
end

Layers = 0;
if isfield(Param,'TruncatedRegionLayers')
    Layers = Layers + max(0, floor(Param.TruncatedRegionLayers));
end
if isfield(Param,'TruncatedObsClipExtraLayers')
    Layers = Layers + max(0, floor(Param.TruncatedObsClipExtraLayers));
end
if isfield(Param,'BlockHashExtraLayers')
    Layers = Layers + max(0, floor(Param.BlockHashExtraLayers));
end

InterpPad = 1;
if isfield(Param,'BlockHashInterpPadding') && Param.BlockHashInterpPadding >= 0
    InterpPad = floor(Param.BlockHashInterpPadding);
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
BlockSize = Map.BlockSize;
UseNumericKey = strcmp(Map.BlockKeyToIdx.KeyType, 'uint64');

MinRow = inf; MaxRow = -inf;
MinCol = inf; MaxCol = -inf;
MinH = inf; MaxH = -inf;

Keys = Map.BlockKeyToIdx.keys;
for k = 1:numel(Keys)
    Key = Keys{k};
    Idx = double(Map.BlockKeyToIdx(Key));
    if Idx <= 0 || Idx > numel(Map.BlockGrid) || isempty(Map.BlockGrid{Idx})
        continue;
    end
    GridBlk = Map.BlockGrid{Idx};
    Mask = (GridBlk > OccThreshold);
    if ~any(Mask(:))
        continue;
    end

    [bi,bj,bh] = LocalParseKeyCompat(Key, UseNumericKey);
    br0 = (bi-1)*BlockSize + 1;
    bc0 = (bj-1)*BlockSize + 1;
    bh0 = (bh-1)*BlockSize + 1;

    rAny = any(reshape(Mask, size(Mask,1), []), 2);
    cAny = any(reshape(permute(Mask, [2,1,3]), size(Mask,2), []), 2);
    hAny = any(reshape(permute(Mask, [3,1,2]), size(Mask,3), []), 2);

    rMinBlk = find(rAny, 1, 'first');
    rMaxBlk = find(rAny, 1, 'last');
    cMinBlk = find(cAny, 1, 'first');
    cMaxBlk = find(cAny, 1, 'last');
    hMinBlk = find(hAny, 1, 'first');
    hMaxBlk = find(hAny, 1, 'last');

    MinRow = min(MinRow, br0 + rMinBlk - 1);
    MaxRow = max(MaxRow, br0 + rMaxBlk - 1);
    MinCol = min(MinCol, bc0 + cMinBlk - 1);
    MaxCol = max(MaxCol, bc0 + cMaxBlk - 1);
    MinH = min(MinH, bh0 + hMinBlk - 1);
    MaxH = max(MaxH, bh0 + hMaxBlk - 1);
end

if ~isfinite(MinRow)
    return;
end

Pad = Layers + InterpPad;

Region.RowMin = max(1, MinRow - Pad);
Region.RowMax = min(Size_i, MaxRow + Pad);
Region.ColMin = max(1, MinCol - Pad);
Region.ColMax = min(Size_j, MaxCol + Pad);
Region.HMin = max(1, MinH - Pad);
Region.HMax = min(Size_h, MaxH + Pad);

HasRegion = true;

end

function [bi,bj,bh] = LocalParseKeyCompat(Key, UseNumericKey)
if UseNumericKey
    Mask21 = uint64(2^21 - 1);
    bi = double(bitand(Key, Mask21));
    bj = double(bitand(bitshift(Key, -21), Mask21));
    bh = double(bitand(bitshift(Key, -42), Mask21));
else
    vals = sscanf(Key, '%d_%d_%d');
    bi = vals(1);
    bj = vals(2);
    bh = vals(3);
end
end

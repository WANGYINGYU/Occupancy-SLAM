function MapDense = FuncBlockHashExtractDenseRegion(Map, Region)
% Extract a dense local map from block-hash storage for solver/interpolation.

if ~isfield(Map,'StorageType') || ~strcmp(Map.StorageType,'blocked')
    error('FuncBlockHashExtractDenseRegion requires blocked map storage.');
end

if isempty(Region)
    error('Region must not be empty.');
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
Scale = Map.Scale;
Origin = Map.Origin;

r0 = max(1, floor(Region.RowMin));
r1 = min(Size_i, floor(Region.RowMax));
c0 = max(1, floor(Region.ColMin));
c1 = min(Size_j, floor(Region.ColMax));
h0 = max(1, floor(Region.HMin));
h1 = min(Size_h, floor(Region.HMax));

if r1 < r0 || c1 < c0 || h1 < h0
    r0 = 1; r1 = 1; c0 = 1; c1 = 1; h0 = 1; h1 = 1;
end

LocalSize_i = r1 - r0 + 1;
LocalSize_j = c1 - c0 + 1;
LocalSize_h = h1 - h0 + 1;

GridLocal = zeros(LocalSize_i, LocalSize_j, LocalSize_h);
NLocal = zeros(LocalSize_i, LocalSize_j, LocalSize_h);

BlockSize = Map.BlockSize;
UseNumericKey = strcmp(Map.BlockKeyToIdx.KeyType, 'uint64');
bi0 = floor((r0-1)/BlockSize) + 1;
bi1 = floor((r1-1)/BlockSize) + 1;
bj0 = floor((c0-1)/BlockSize) + 1;
bj1 = floor((c1-1)/BlockSize) + 1;
bh0 = floor((h0-1)/BlockSize) + 1;
bh1 = floor((h1-1)/BlockSize) + 1;

for bh = bh0:bh1
    for bj = bj0:bj1
        for bi = bi0:bi1
            Key = LocalMakeKeyCompat(bi,bj,bh,UseNumericKey);
            if ~isKey(Map.BlockKeyToIdx, Key)
                continue;
            end

            Idx = double(Map.BlockKeyToIdx(Key));
            if Idx <= 0 || Idx > numel(Map.BlockGrid) || isempty(Map.BlockGrid{Idx})
                continue;
            end

            GridBlk = Map.BlockGrid{Idx};
            NBlk = Map.BlockN{Idx};

            br0 = (bi-1)*BlockSize + 1;
            bc0 = (bj-1)*BlockSize + 1;
            bh0g = (bh-1)*BlockSize + 1;

            br1 = br0 + size(GridBlk,1) - 1;
            bc1 = bc0 + size(GridBlk,2) - 1;
            bh1g = bh0g + size(GridBlk,3) - 1;

            rr0 = max(r0, br0); rr1 = min(r1, br1);
            cc0 = max(c0, bc0); cc1 = min(c1, bc1);
            hh0 = max(h0, bh0g); hh1 = min(h1, bh1g);

            if rr1 < rr0 || cc1 < cc0 || hh1 < hh0
                continue;
            end

            lr0 = rr0 - r0 + 1; lr1 = rr1 - r0 + 1;
            lc0 = cc0 - c0 + 1; lc1 = cc1 - c0 + 1;
            lh0 = hh0 - h0 + 1; lh1 = hh1 - h0 + 1;

            brs0 = rr0 - br0 + 1; brs1 = rr1 - br0 + 1;
            bcs0 = cc0 - bc0 + 1; bcs1 = cc1 - bc0 + 1;
            bhs0 = hh0 - bh0g + 1; bhs1 = hh1 - bh0g + 1;

            GridLocal(lr0:lr1, lc0:lc1, lh0:lh1) = GridBlk(brs0:brs1, bcs0:bcs1, bhs0:bhs1);
            NLocal(lr0:lr1, lc0:lc1, lh0:lh1) = NBlk(brs0:brs1, bcs0:bcs1, bhs0:bhs1);
        end
    end
end

OriginLocal = Origin + [(c0-1)*Scale; (r0-1)*Scale; (h0-1)*Scale];

MapDense = struct();
MapDense.Grid = GridLocal;
MapDense.N = NLocal;
MapDense.Scale = Scale;
MapDense.Origin = OriginLocal;
MapDense.Size_i = LocalSize_i;
MapDense.Size_j = LocalSize_j;
MapDense.Size_h = LocalSize_h;
MapDense.RegionGlobal = struct('RowMin',r0,'RowMax',r1,'ColMin',c0,'ColMax',c1,'HMin',h0,'HMax',h1);
if isfield(Map,'WorldToMapR')
    MapDense.WorldToMapR = Map.WorldToMapR;
end
if isfield(Map,'WorldToMapT')
    MapDense.WorldToMapT = Map.WorldToMapT;
end
if isfield(Map,'MapFrameStats')
    MapDense.MapFrameStats = Map.MapFrameStats;
end

end

function Key = LocalMakeKeyCompat(bi,bj,bh,UseNumericKey)
if UseNumericKey
    Key = uint64(bi) + bitshift(uint64(bj), 21) + bitshift(uint64(bh), 42);
else
    Key = sprintf('%d_%d_%d', bi, bj, bh);
end
end

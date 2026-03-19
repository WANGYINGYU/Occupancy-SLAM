function Map = FuncBlockHashMergeDenseRegion(Map, MapDense, Region)
% Write back local dense region into block-hash map storage.

if ~isfield(Map,'StorageType') || ~strcmp(Map.StorageType,'blocked')
    error('FuncBlockHashMergeDenseRegion requires blocked map storage.');
end

if isempty(Region)
    return;
end

if ~isfield(MapDense,'Grid') || ~isfield(MapDense,'N')
    error('MapDense must contain Grid and N.');
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
BlockSize = Map.BlockSize;
UseNumericKey = strcmp(Map.BlockKeyToIdx.KeyType, 'uint64');

r0 = max(1, floor(Region.RowMin));
r1 = min(Size_i, floor(Region.RowMax));
c0 = max(1, floor(Region.ColMin));
c1 = min(Size_j, floor(Region.ColMax));
h0 = max(1, floor(Region.HMin));
h1 = min(Size_h, floor(Region.HMax));

if r1 < r0 || c1 < c0 || h1 < h0
    return;
end

GridLocal = MapDense.Grid;
NLocal = MapDense.N;

bi0 = floor((r0-1)/BlockSize) + 1;
bi1 = floor((r1-1)/BlockSize) + 1;
bj0 = floor((c0-1)/BlockSize) + 1;
bj1 = floor((c1-1)/BlockSize) + 1;
bh0 = floor((h0-1)/BlockSize) + 1;
bh1 = floor((h1-1)/BlockSize) + 1;

for bh = bh0:bh1
    for bj = bj0:bj1
        for bi = bi0:bi1
            br0 = (bi-1)*BlockSize + 1;
            bc0 = (bj-1)*BlockSize + 1;
            bh0g = (bh-1)*BlockSize + 1;

            br1 = min(bi*BlockSize, Size_i);
            bc1 = min(bj*BlockSize, Size_j);
            bh1g = min(bh*BlockSize, Size_h);

            rr0 = max(r0, br0); rr1 = min(r1, br1);
            cc0 = max(c0, bc0); cc1 = min(c1, bc1);
            hh0 = max(h0, bh0g); hh1 = min(h1, bh1g);

            if rr1 < rr0 || cc1 < cc0 || hh1 < hh0
                continue;
            end

            lr0 = rr0 - r0 + 1; lr1 = rr1 - r0 + 1;
            lc0 = cc0 - c0 + 1; lc1 = cc1 - c0 + 1;
            lh0 = hh0 - h0 + 1; lh1 = hh1 - h0 + 1;

            SubGrid = GridLocal(lr0:lr1, lc0:lc1, lh0:lh1);
            SubN = NLocal(lr0:lr1, lc0:lc1, lh0:lh1);

            Key = LocalMakeKeyCompat(bi,bj,bh,UseNumericKey);
            if isKey(Map.BlockKeyToIdx, Key)
                Idx = double(Map.BlockKeyToIdx(Key));
                if Idx <= 0 || Idx > numel(Map.BlockGrid) || isempty(Map.BlockGrid{Idx})
                    [Map, Idx] = LocalAllocBlock(Map, Key, bi, bj, bh);
                end
            else
                [Map, Idx] = LocalAllocBlock(Map, Key, bi, bj, bh);
            end

            Idx = double(Map.BlockKeyToIdx(Key));
            GridBlk = Map.BlockGrid{Idx};
            NBlk = Map.BlockN{Idx};

            brs0 = rr0 - br0 + 1; brs1 = rr1 - br0 + 1;
            bcs0 = cc0 - bc0 + 1; bcs1 = cc1 - bc0 + 1;
            bhs0 = hh0 - bh0g + 1; bhs1 = hh1 - bh0g + 1;

            GridBlk(brs0:brs1, bcs0:bcs1, bhs0:bhs1) = SubGrid;
            NBlk(brs0:brs1, bcs0:bcs1, bhs0:bhs1) = SubN;

            if any(GridBlk(:) ~= 0) || any(NBlk(:) ~= 0)
                Map.BlockGrid{Idx} = GridBlk;
                Map.BlockN{Idx} = NBlk;
            else
                remove(Map.BlockKeyToIdx, Key);
                Map.BlockGrid{Idx} = [];
                Map.BlockN{Idx} = [];
            end
        end
    end
end

end

function [Map, Idx] = LocalAllocBlock(Map, Key, bi, bj, bh)
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
BlockSize = Map.BlockSize;

br0 = (bi-1)*BlockSize + 1; br1 = min(bi*BlockSize, Size_i);
bc0 = (bj-1)*BlockSize + 1; bc1 = min(bj*BlockSize, Size_j);
bh0 = (bh-1)*BlockSize + 1; bh1 = min(bh*BlockSize, Size_h);

BlkSize = [br1-br0+1, bc1-bc0+1, bh1-bh0+1];
Idx = double(numel(Map.BlockGrid) + 1);
Map.BlockGrid{Idx,1} = zeros(BlkSize);
Map.BlockN{Idx,1} = zeros(BlkSize);
Map.BlockKeyToIdx(Key) = uint32(Idx);
end

function Key = LocalMakeKeyCompat(bi,bj,bh,UseNumericKey)
if UseNumericKey
    Key = uint64(bi) + bitshift(uint64(bj), 21) + bitshift(uint64(bh), 42);
else
    Key = sprintf('%d_%d_%d', bi, bj, bh);
end
end

function Map = FuncDenseMapToBlockHash(Map, Param)
% Convert dense map storage (Grid/N) to block-hash storage.
% This is intended for truncated optimization mode to reduce memory use.

if isfield(Map,'StorageType') && strcmp(Map.StorageType,'blocked')
    return;
end

if ~isfield(Map,'Grid') || ~isfield(Map,'N')
    error('FuncDenseMapToBlockHash requires dense Map.Grid and Map.N.');
end

BlockSize = 16;
if nargin >= 2 && isfield(Param,'BlockHashBlockSize') && Param.BlockHashBlockSize > 0
    BlockSize = floor(Param.BlockHashBlockSize);
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

Grid = Map.Grid;
N = Map.N;

KeyToIdx = containers.Map('KeyType','uint64','ValueType','uint32');
BlockGrid = {};
BlockN = {};
BlockCount = uint32(0);

IdNZ = find((Grid ~= 0) | (N ~= 0));
if ~isempty(IdNZ)
    [Row, Col, H] = ind2sub([Size_i, Size_j, Size_h], IdNZ);
    Bi = floor((Row - 1) / BlockSize) + 1;
    Bj = floor((Col - 1) / BlockSize) + 1;
    Bh = floor((H - 1) / BlockSize) + 1;
    BlockKeys = LocalMakeKeyNum(Bi, Bj, Bh);
    UniqueKeys = unique(BlockKeys);

    for k = 1:numel(UniqueKeys)
        [bi, bj, bh] = LocalParseKeyNum(UniqueKeys(k));

        r0 = (bi-1)*BlockSize + 1;
        c0 = (bj-1)*BlockSize + 1;
        h0 = (bh-1)*BlockSize + 1;
        r1 = min(bi*BlockSize, Size_i);
        c1 = min(bj*BlockSize, Size_j);
        h1 = min(bh*BlockSize, Size_h);

        GridBlk = Grid(r0:r1, c0:c1, h0:h1);
        NBlk = N(r0:r1, c0:c1, h0:h1);

        if ~any(GridBlk(:) ~= 0) && ~any(NBlk(:) ~= 0)
            continue;
        end

        BlockCount = BlockCount + 1;
        BlockGrid{BlockCount,1} = GridBlk; %#ok<AGROW>
        BlockN{BlockCount,1} = NBlk; %#ok<AGROW>
        KeyToIdx(UniqueKeys(k)) = BlockCount;
    end
end

Map.StorageType = 'blocked';
Map.BlockSize = BlockSize;
Map.BlockKeyToIdx = KeyToIdx;
Map.BlockGrid = BlockGrid;
Map.BlockN = BlockN;

% Keep dense map size/origin/scale metadata, drop dense payload to save memory.
if isfield(Map,'Grid'); Map = rmfield(Map,'Grid'); end
if isfield(Map,'N'); Map = rmfield(Map,'N'); end
if isfield(Map,'DgridG'); Map = rmfield(Map,'DgridG'); end
if isfield(Map,'NgridG'); Map = rmfield(Map,'NgridG'); end
if isfield(Map,'DgridGu'); Map = rmfield(Map,'DgridGu'); end
if isfield(Map,'DgridGv'); Map = rmfield(Map,'DgridGv'); end
if isfield(Map,'DgridGz'); Map = rmfield(Map,'DgridGz'); end

end

function Key = LocalMakeKeyNum(bi,bj,bh)
Key = uint64(bi) + bitshift(uint64(bj), 21) + bitshift(uint64(bh), 42);
end

function [bi,bj,bh] = LocalParseKeyNum(Key)
Mask21 = uint64(2^21 - 1);
bi = double(bitand(Key, Mask21));
bj = double(bitand(bitshift(Key, -21), Mask21));
bh = double(bitand(bitshift(Key, -42), Mask21));
end

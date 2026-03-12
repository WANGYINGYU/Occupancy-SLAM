function ActiveVarId = FuncExpandActiveVoxelIds(BaseVarId, Size_i, Size_j, Size_h, NeighborLayers)
% Expand active voxel ids with k-layer neighbors in a 3D grid.

if nargin < 5 || isempty(NeighborLayers)
    NeighborLayers = 0;
end

BaseVarId = unique(double(BaseVarId(:)));
if isempty(BaseVarId)
    ActiveVarId = BaseVarId;
    return;
end

NeighborLayers = max(0, floor(double(NeighborLayers)));
if NeighborLayers == 0
    ActiveVarId = BaseVarId;
    return;
end

[row0, col0, h0] = ind2sub([Size_i, Size_j, Size_h], BaseVarId);
AllId = BaseVarId;

for dh = -NeighborLayers:NeighborLayers
    for dc = -NeighborLayers:NeighborLayers
        for dr = -NeighborLayers:NeighborLayers
            if dr == 0 && dc == 0 && dh == 0
                continue;
            end

            row = row0 + dr;
            col = col0 + dc;
            h = h0 + dh;

            Valid = row >= 1 & row <= Size_i & ...
                    col >= 1 & col <= Size_j & ...
                    h >= 1 & h <= Size_h;

            if any(Valid)
                NeighborId = sub2ind([Size_i, Size_j, Size_h], row(Valid), col(Valid), h(Valid));
                AllId = [AllId; NeighborId]; %#ok<AGROW>
            end
        end
    end
end

ActiveVarId = unique(AllId);

end

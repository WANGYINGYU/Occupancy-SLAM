function IdReserve = FuncFindCellOptimized(Map, Param)

if nargin < 2 || isempty(Param)
    Param = struct();
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

Range = 2;
if isfield(Param,'TruncatedRegionLayers') && Param.TruncatedRegionLayers >= 0
    Range = floor(Param.TruncatedRegionLayers);
end

OccThreshold = 0;
if isfield(Param,'TruncatedOccupancyThreshold')
    OccThreshold = Param.TruncatedOccupancyThreshold;
end

IdOccupied = find(Map.Grid > OccThreshold);
if isempty(IdOccupied)
    IdReserve = [];
    return;
end

[Row0, Col0, H0] = ind2sub([Size_i, Size_j, Size_h], IdOccupied);
IdAll = IdOccupied;

for dh = -Range:Range
    for dc = -Range:Range
        for dr = -Range:Range
            if dr == 0 && dc == 0 && dh == 0
                continue;
            end
            Row = Row0 + dr;
            Col = Col0 + dc;
            H = H0 + dh;

            Valid = Row >= 1 & Row <= Size_i & ...
                    Col >= 1 & Col <= Size_j & ...
                    H >= 1 & H <= Size_h;

            if any(Valid)
                IdNeighbor = sub2ind([Size_i, Size_j, Size_h], Row(Valid), Col(Valid), H(Valid));
                IdAll = [IdAll; IdNeighbor]; %#ok<AGROW>
            end
        end
    end
end

IdReserve = unique(double(IdAll(:)));

end

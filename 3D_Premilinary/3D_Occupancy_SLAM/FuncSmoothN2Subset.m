function Map = FuncSmoothN2Subset(Map, Lambda, HHSub, MapVarId, BoundaryTerm)
% Smooth map hit count only on selected variable ids.

if isempty(MapVarId)
    return;
end
if nargin < 5
    BoundaryTerm = [];
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

MapVarId = unique(double(MapVarId(:)));
NumVar = length(MapVarId);

if NumVar == 0
    return;
end

NVec = reshape(permute(Map.N, [2, 1, 3]), Size_i * Size_j * Size_h, 1);
NSub = double(NVec(MapVarId));

ObsLocalId = find(NSub ~= 0);
NumObs = length(ObsLocalId);

if NumObs > 0
    A1 = sparse((1:NumObs)', ObsLocalId, 1, NumObs, NumVar);
    Val = sparse(NSub(ObsLocalId));
    I = A1' * A1 + Lambda * HHSub;
    E = A1' * Val;
else
    I = Lambda * HHSub;
    E = sparse(NumVar,1);
end

if ~isempty(BoundaryTerm)
    E = E + Lambda * sparse(double(BoundaryTerm(:)));
end

I = I + 1e-9 * speye(NumVar);
DeltaNSub = I \ E;
if any(~isfinite(DeltaNSub))
    I = I + 1e-6 * speye(NumVar);
    DeltaNSub = I \ E;
end
DeltaNSub(~isfinite(DeltaNSub)) = 0;
NVec(MapVarId) = full(DeltaNSub);

NMat = reshape(NVec, [Size_j, Size_i, Size_h]);
Map.N = permute(NMat, [2, 1, 3]);

end

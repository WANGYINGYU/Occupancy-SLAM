function Map = FuncSmoothN2Subset(Map, Lambda, HHSub, MapVarId, BoundaryTerm, Param)
% Smooth map hit count only on selected variable ids.

if isempty(MapVarId)
    return;
end
if nargin < 5
    BoundaryTerm = [];
end
if nargin < 6 || isempty(Param)
    Param = struct();
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
    ObsLocalId = double(ObsLocalId(:));
    Val = double(NSub(ObsLocalId));
    DObs = sparse(ObsLocalId, ObsLocalId, 1, NumVar, NumVar);
    I = DObs + Lambda * HHSub;
    E = sparse(ObsLocalId, 1, Val, NumVar, 1);
else
    I = Lambda * HHSub;
    E = sparse(NumVar,1);
end

if ~isempty(BoundaryTerm)
    E = E + Lambda * sparse(double(BoundaryTerm(:)));
end

SolverRidge = 1e-9;
if isfield(Param,'LinearSolverRidge') && Param.LinearSolverRidge > 0
    SolverRidge = Param.LinearSolverRidge;
end
I = I + SolverRidge * speye(NumVar);

UsePCGSolver = isfield(Param,'UsePCGSolver') && Param.UsePCGSolver;
if UsePCGSolver
    persistent PCGWarmStart PCGWarmSize PCGPrecondL PCGPatternKey

    A = 0.5 * (I + I');
    x0 = zeros(NumVar,1);
    if ~isempty(PCGWarmStart) && ~isempty(PCGWarmSize) && PCGWarmSize == NumVar && all(isfinite(PCGWarmStart))
        x0 = PCGWarmStart;
    end

    % Reuse preconditioner when subset structure is unchanged.
    KeyHead = 0;
    KeyTail = 0;
    if NumVar > 0
        KeyHead = MapVarId(1);
        KeyTail = MapVarId(end);
    end
    PatternKey = [NumVar, nnz(HHSub), KeyHead, KeyTail];

    UseCachedPrecond = ~isempty(PCGPrecondL) && ~isempty(PCGPatternKey) && isequal(PCGPatternKey, PatternKey);
    Lp = [];
    if UseCachedPrecond
        Lp = PCGPrecondL;
    else
        try
            IcholSetup = struct('type','ict', 'droptol',1e-3, 'diagcomp',1e-3);
            Lp = ichol(A, IcholSetup);
            PCGPrecondL = Lp;
            PCGPatternKey = PatternKey;
        catch
            PCGPrecondL = [];
            PCGPatternKey = [];
            Lp = [];
        end
    end

    PCGTol = 1e-3;
    PCGMaxIter = 80;
    if ~isempty(Lp)
        [DeltaNSub, flag] = pcg(A, E, PCGTol, PCGMaxIter, Lp, Lp', x0);
    else
        [DeltaNSub, flag] = pcg(A, E, PCGTol, PCGMaxIter, [], [], x0);
    end

    if flag == 0 && all(isfinite(DeltaNSub))
        PCGWarmStart = DeltaNSub;
        PCGWarmSize = NumVar;
    else
        DeltaNSub = I \ E;
        PCGWarmStart = [];
        PCGWarmSize = [];
    end
else
    DeltaNSub = I \ E;
end

if any(~isfinite(DeltaNSub))
    I = I + 1e-6 * speye(NumVar);
    DeltaNSub = I \ E;
end
DeltaNSub(~isfinite(DeltaNSub)) = 0;
NVec(MapVarId) = full(DeltaNSub);

NMat = reshape(NVec, [Size_j, Size_i, Size_h]);
Map.N = permute(NMat, [2, 1, 3]);

end

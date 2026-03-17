function [Map] = FuncSmoothN2(Map,Lambda,HH2,Param)

if nargin < 4 || isempty(Param)
    Param = struct();
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

[i, j, h] = ind2sub(size(Map.N), find(Map.N ~= 0));
Val = Map.N(Map.N ~= 0);


ID2 = Size_i*Size_j*(h-1)+Size_j*(i-1)+j;

NumVar = Size_i*Size_j*Size_h;
Val = double(Val(:));

% A1'*A1 is diagonal with ones on observed voxel ids.
DObs = sparse(double(ID2), double(ID2), 1, NumVar, NumVar);
E = sparse(double(ID2), 1, Val, NumVar, 1);
I = DObs + Lambda * HH2;

UsePCGSolver = isfield(Param,'UsePCGSolver') && Param.UsePCGSolver;
if UsePCGSolver
    persistent PCGWarmStart PCGWarmSize PCGPrecondL PCGPrecondNnz PCGPrecondSize

    SolverRidge = 1e-9;
    if isfield(Param,'LinearSolverRidge') && Param.LinearSolverRidge > 0
        SolverRidge = Param.LinearSolverRidge;
    end
    A = 0.5 * (I + I') + SolverRidge * speye(NumVar);

    x0 = zeros(NumVar,1);
    if ~isempty(PCGWarmStart) && ~isempty(PCGWarmSize) && PCGWarmSize == NumVar && all(isfinite(PCGWarmStart))
        x0 = PCGWarmStart;
    end

    RebuildPrecond = true;
    if ~isempty(PCGPrecondL) && ~isempty(PCGPrecondNnz) && ~isempty(PCGPrecondSize) ...
            && PCGPrecondNnz == nnz(A) && PCGPrecondSize == NumVar
        RebuildPrecond = false;
    end

    Lp = [];
    if ~RebuildPrecond
        Lp = PCGPrecondL;
    else
        try
            IcholSetup = struct('type','ict', 'droptol',1e-3, 'diagcomp',1e-3);
            Lp = ichol(A, IcholSetup);
            PCGPrecondL = Lp;
            PCGPrecondNnz = nnz(A);
            PCGPrecondSize = NumVar;
        catch
            PCGPrecondL = [];
            PCGPrecondNnz = [];
            PCGPrecondSize = [];
            Lp = [];
        end
    end

    PCGTol = 1e-3;
    PCGMaxIter = 80;
    if ~isempty(Lp)
        [DeltaN, flag] = pcg(A, E, PCGTol, PCGMaxIter, Lp, Lp', x0);
    else
        [DeltaN, flag] = pcg(A, E, PCGTol, PCGMaxIter, [], [], x0);
    end

    if flag == 0 && all(isfinite(DeltaN))
        PCGWarmStart = DeltaN;
        PCGWarmSize = NumVar;
    else
        I = I + SolverRidge * speye(NumVar);
        DeltaN = I \ E;
        PCGWarmStart = [];
        PCGWarmSize = [];
    end
else
    DeltaN = I \ E;
end

DeltaN = full(DeltaN);

MatrixReshape = reshape(DeltaN, [Size_j,Size_i,Size_h]);

Map.N = permute(MatrixReshape, [2, 1, 3]);


end

function [HH, BoundaryTerm] = FuncMapConst3DByVarId(VarId, Size_i, Size_j, Size_h, FullVec)
% Build principal smoothness block (H_ss) on selected variable ids.
% BoundaryTerm = -H_su * x_u for fixed outside variables (optional).

if nargin < 5
    FullVec = [];
end

VarId = unique(double(VarId(:)));
NumVar = length(VarId);

if NumVar == 0
    HH = sparse(0,0);
    BoundaryTerm = sparse(0,1);
    return;
end

TotalSize = Size_i * Size_j * Size_h;
VarToLocal = sparse(VarId, ones(NumVar,1), 1:NumVar, TotalSize, 1);

UseBoundaryTerm = ~isempty(FullVec);
UseVectorLookup = false;
if UseBoundaryTerm
    if isvector(FullVec)
        FullVec = double(FullVec(:));
        if length(FullVec) ~= TotalSize
            error('FullVec length mismatch in FuncMapConst3DByVarId.');
        end
        UseVectorLookup = true;
    else
        if ~isequal(size(FullVec), [Size_i, Size_j, Size_h])
            error('FullVec size mismatch in FuncMapConst3DByVarId.');
        end
    end
end

PlaneSize = Size_i * Size_j;

% Upper bound: each variable contributes up to 3 forward edges.
MaxEdge = 3 * NumVar;
OffI = zeros(2*MaxEdge,1);
OffJ = zeros(2*MaxEdge,1);
OffV = zeros(2*MaxEdge,1);
DiagV = zeros(NumVar,1);
BoundaryTerm = zeros(NumVar,1);

EdgeCnt = 0;

for k = 1:NumVar
    vid = VarId(k);

    h = floor((vid-1) / PlaneSize) + 1;
    rem2 = vid - (h-1) * PlaneSize;
    r = floor((rem2-1) / Size_j) + 1;
    c = rem2 - (r-1) * Size_j;

    % Full-grid degree (same as global HH principal block diagonal).
    DiagV(k) = (c > 1) + (c < Size_j) + ...
               (r > 1) + (r < Size_i) + ...
               (h > 1) + (h < Size_h);

    % Positive directions to create each in-subset edge once.
    if c < Size_j
        nb = vid + 1;
        localNb = full(VarToLocal(nb));
        if localNb > 0
            EdgeCnt = EdgeCnt + 1;
            id = 2*EdgeCnt-1;
            OffI(id:id+1) = [k; localNb];
            OffJ(id:id+1) = [localNb; k];
            OffV(id:id+1) = [-1; -1];
        end
    end

    if r < Size_i
        nb = vid + Size_j;
        localNb = full(VarToLocal(nb));
        if localNb > 0
            EdgeCnt = EdgeCnt + 1;
            id = 2*EdgeCnt-1;
            OffI(id:id+1) = [k; localNb];
            OffJ(id:id+1) = [localNb; k];
            OffV(id:id+1) = [-1; -1];
        end
    end

    if h < Size_h
        nb = vid + PlaneSize;
        localNb = full(VarToLocal(nb));
        if localNb > 0
            EdgeCnt = EdgeCnt + 1;
            id = 2*EdgeCnt-1;
            OffI(id:id+1) = [k; localNb];
            OffJ(id:id+1) = [localNb; k];
            OffV(id:id+1) = [-1; -1];
        end
    end

    % Outside-neighbor anchor term: BoundaryTerm = -H_su * x_u.
    if UseBoundaryTerm
        if c > 1
            nb = vid - 1;
            if full(VarToLocal(nb)) == 0
                if UseVectorLookup
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(nb);
                else
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(r, c-1, h);
                end
            end
        end
        if c < Size_j
            nb = vid + 1;
            if full(VarToLocal(nb)) == 0
                if UseVectorLookup
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(nb);
                else
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(r, c+1, h);
                end
            end
        end
        if r > 1
            nb = vid - Size_j;
            if full(VarToLocal(nb)) == 0
                if UseVectorLookup
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(nb);
                else
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(r-1, c, h);
                end
            end
        end
        if r < Size_i
            nb = vid + Size_j;
            if full(VarToLocal(nb)) == 0
                if UseVectorLookup
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(nb);
                else
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(r+1, c, h);
                end
            end
        end
        if h > 1
            nb = vid - PlaneSize;
            if full(VarToLocal(nb)) == 0
                if UseVectorLookup
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(nb);
                else
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(r, c, h-1);
                end
            end
        end
        if h < Size_h
            nb = vid + PlaneSize;
            if full(VarToLocal(nb)) == 0
                if UseVectorLookup
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(nb);
                else
                    BoundaryTerm(k) = BoundaryTerm(k) + FullVec(r, c, h+1);
                end
            end
        end
    end
end

DiagI = (1:NumVar)';
DiagJ = DiagI;

if EdgeCnt > 0
    OffI = OffI(1:2*EdgeCnt);
    OffJ = OffJ(1:2*EdgeCnt);
    OffV = OffV(1:2*EdgeCnt);
    HH = sparse([DiagI; OffI], [DiagJ; OffJ], [DiagV; OffV], NumVar, NumVar);
else
    HH = sparse(DiagI, DiagJ, DiagV, NumVar, NumVar);
end

BoundaryTerm = sparse(BoundaryTerm);

end

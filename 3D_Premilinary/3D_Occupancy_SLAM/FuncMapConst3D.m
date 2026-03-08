function HH = FuncMapConst3D(Map)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

nEdgeJ = Size_i * max(Size_j - 1, 0) * Size_h;
nEdgeI = max(Size_i - 1, 0) * Size_j * Size_h;
nEdgeH = Size_i * Size_j * max(Size_h - 1, 0);
nEdges = nEdgeJ + nEdgeI + nEdgeH;
nVoxels = Size_i * Size_j * Size_h;

if nEdges == 0
    HH = sparse(nVoxels, nVoxels);
    return;
end

ID1 = repelem((1:nEdges)',2,1);
ID2 = zeros(2*nEdges,1);
Val = repmat([1; -1], nEdges, 1);

edgeId = 0;
for i = 1:Size_i
    for j = 1:Size_j
        for h = 1:Size_h
            ij0 = Size_i*Size_j*(h-1) + Size_j*(i-1) + j;

            if j+1 <= Size_j
                edgeId = edgeId + 1;
                p = 2*edgeId - 1;
                ID2(p) = ij0;
                ID2(p+1) = ij0 + 1;
            end

            if i+1 <= Size_i
                edgeId = edgeId + 1;
                p = 2*edgeId - 1;
                ID2(p) = ij0;
                ID2(p+1) = ij0 + Size_j;
            end

            if h+1 <= Size_h
                edgeId = edgeId + 1;
                p = 2*edgeId - 1;
                ID2(p) = ij0;
                ID2(p+1) = ij0 + Size_i*Size_j;
            end
        end
    end
end

J = sparse(ID1,ID2,Val,nEdges,nVoxels);
HH = J'*J;

end

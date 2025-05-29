function [Map,Pose] = FuncUpdate3D(Map,Pose,DeltaP,DeltaM)
DeltaP2 = reshape(DeltaP,6,[])';
Pose(2:end,:) = Pose(2:end,:)+DeltaP2;
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
GridReshape = reshape(permute(Map.Grid, [2, 1, 3]), Size_i * Size_j * Size_h, 1);
GridUpdate = GridReshape + DeltaM;
MatrixReshape = reshape(GridUpdate, [Size_j,Size_i,Size_h]);
Map.Grid = permute(MatrixReshape, [2, 1, 3]);
end



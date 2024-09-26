function [SelectMap,Pose] = FuncUpdateSelect3D(SelectMap,Pose,DeltaP,DeltaM,Param)

DeltaP2 = reshape(DeltaP,6,[])';
Pose(2:end,:) = Pose(2:end,:)+DeltaP2;

Size_i = SelectMap.Size_i;
Size_j = SelectMap.Size_j;
Size_h = SelectMap.Size_h;

% Recover DeltaM to full
VarId = Param.SortId;
FullDeltaM = sparse(VarId,1,DeltaM,Size_i*Size_j*Size_h,1);

GridReshape = reshape(permute(SelectMap.Grid, [2, 1, 3]), Size_i * Size_j * Size_h, 1);
% GridReshape = reshape(permute(Map.NormalizeMap, [2, 1, 3]), Size_i * Size_j * Size_h, 1);



GridUpdate = GridReshape + FullDeltaM;

MatrixReshape = reshape(GridUpdate, [Size_j,Size_i,Size_h]);

SelectMap.Grid = permute(MatrixReshape, [2, 1, 3]);
% Map.NormalizeMap = permute(MatrixReshape, [2, 1, 3]);


end



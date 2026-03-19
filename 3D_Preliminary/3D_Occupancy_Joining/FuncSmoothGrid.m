function [Map] = FuncSmoothGrid(Map,Lambda,HH2)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;


% [i, j, h] = ind2sub(size(Map.Grid), find(Map.Grid ~= 0));
% Val = Map.Grid(Map.Grid ~= 0);

[i, j, h] = ind2sub(size(Map.Grid), find(isnan(Map.Grid) ~= 1));
Val = Map.Grid(isnan(Map.Grid) ~= 1);

ni = length(i); % the number of non-zero elements
ID1 = (1:ni)';

ID2 = Size_i*Size_j*(h-1)+Size_j*(i-1)+j;

A1 = sparse(ID1,ID2,1,ni,Size_i*Size_j*Size_h);
Val = sparse(Val);

HH = HH2*Lambda;

% XH0 = reshape(permute(Map.Grid, [2, 1, 3]), Size_i * Size_j * Size_h, 1);

% EH = - HH * XH0;

I = A1'*A1+HH;
E = A1'*Val;

% E = A1'*Val + EH;

DeltaGrid = I\E;

DeltaGrid = full(DeltaGrid);

MatrixReshape = reshape(DeltaGrid, [Size_j,Size_i,Size_h]);

Map.Grid = permute(MatrixReshape, [2, 1, 3]);

end
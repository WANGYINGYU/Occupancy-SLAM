function [Map] = FuncSmoothN2(Map,Lambda,HH2)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

[i, j, h] = ind2sub(size(Map.N), find(Map.N ~= 0));
Val = Map.N(Map.N ~= 0);


ni = length(i); % the number of non-zero elements
ID1 = (1:ni)';

ID2 = Size_i*Size_j*(h-1)+Size_j*(i-1)+j;

% ID1 = single(ID1);

A1 = sparse(ID1,ID2,1,ni,Size_i*Size_j*Size_h);
Val = sparse(Val);

HH = HH2*Lambda;
I = A1'*A1+HH;
E = A1'*Val;

DeltaN = I\E;

DeltaN = full(DeltaN);

MatrixReshape = reshape(DeltaN, [Size_j,Size_i,Size_h]);

Map.N = permute(MatrixReshape, [2, 1, 3]);


end
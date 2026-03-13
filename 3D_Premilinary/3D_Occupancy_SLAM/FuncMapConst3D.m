function HH = FuncMapConst3D(Map, Param)
% Build full-map smoothness Hessian HH = J'J on 6-neighbor edges.
% Optional anisotropic weights on x/y/z edge residuals.

if nargin < 2 || isempty(Param)
    Param = struct();
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

UseAnisotropic = isfield(Param,'UseAnisotropicSmoothness') && Param.UseAnisotropicSmoothness;
LambdaX = 1;
LambdaY = 1;
LambdaZ = 1;
if UseAnisotropic && isfield(Param,'SmoothnessLambdaXYZ') && numel(Param.SmoothnessLambdaXYZ) >= 3
    L = double(Param.SmoothnessLambdaXYZ(:)');
    LambdaX = max(0, L(1));
    LambdaY = max(0, L(2));
    LambdaZ = max(0, L(3));
end
SqrtLambdaX = sqrt(LambdaX);
SqrtLambdaY = sqrt(LambdaY);
SqrtLambdaZ = sqrt(LambdaZ);

NumVoxel = Size_i * Size_j * Size_h;
NumEdgeX = Size_i * max(Size_j-1,0) * Size_h;
NumEdgeY = max(Size_i-1,0) * Size_j * Size_h;
NumEdgeZ = Size_i * Size_j * max(Size_h-1,0);
MaxEdge = NumEdgeX + NumEdgeY + NumEdgeZ;

if MaxEdge == 0
    HH = sparse(NumVoxel, NumVoxel);
    return;
end

ID1 = zeros(2*MaxEdge,1,'uint32');
ID2 = zeros(2*MaxEdge,1,'uint32');
Val = zeros(2*MaxEdge,1);

nCnt = uint32(0);
nNZ = uint32(0);

for i = 1:Size_i
    for j = 1:Size_j
        for h = 1:Size_h

            ij0 = Size_i*Size_j*(h-1) + Size_j*(i-1) + j;
            ij1 = ij0 + 1;         % x direction (j+1)
            ij2 = ij0 + Size_j;    % y direction (i+1)
            ij3 = ij0 + Size_i*Size_j; % z direction (h+1)

            if j + 1 <= Size_j
                nCnt = nCnt + 1;
                nNZ = nNZ + 2;
                ID1(nNZ-1:nNZ) = nCnt;
                ID2(nNZ-1:nNZ) = uint32([ij0; ij1]);
                Val(nNZ-1:nNZ) = [SqrtLambdaX; -SqrtLambdaX];
            end

            if i + 1 <= Size_i
                nCnt = nCnt + 1;
                nNZ = nNZ + 2;
                ID1(nNZ-1:nNZ) = nCnt;
                ID2(nNZ-1:nNZ) = uint32([ij0; ij2]);
                Val(nNZ-1:nNZ) = [SqrtLambdaY; -SqrtLambdaY];
            end

            if h + 1 <= Size_h
                nCnt = nCnt + 1;
                nNZ = nNZ + 2;
                ID1(nNZ-1:nNZ) = nCnt;
                ID2(nNZ-1:nNZ) = uint32([ij0; ij3]);
                Val(nNZ-1:nNZ) = [SqrtLambdaZ; -SqrtLambdaZ];
            end
        end
    end
end

ID1 = ID1(1:nNZ);
ID2 = ID2(1:nNZ);
Val = Val(1:nNZ);

J = sparse(double(ID1), double(ID2), Val, double(nCnt), NumVoxel);
HH = J' * J;

end

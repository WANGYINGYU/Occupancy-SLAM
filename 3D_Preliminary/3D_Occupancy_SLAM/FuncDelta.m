function [DeltaP,DeltaM,Mean_Delta,Mean_Delta_Pose] = FuncDelta(JP,JM,JO,ErrorS,ErrorO,Map,HH,Param)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;


THRESHOLD = 8;
Lambda_O = Param.LambdaO;

JP = JP(:,7:end);
JO = JO(:,7:end);

% Mask_JM = (JM~=0);
% Sum_Num_JM = sum(Mask_JM,1);
% Id_JM = find(Sum_Num_JM<THRESHOLD);
% 
% Id_Recover = FuncFindCellOptimized(Map);
% 
% 
% Id_Recover = find(Sum_Num_JM >=THRESHOLD);
% JM(:,Id_JM) = [];

U = JP'*JP+Lambda_O*(JO'*JO);
V = JM'*JM;
W = JP'*JM;

ErrorS = sparse(ErrorS);
ErrorO = sparse(ErrorO);
EP = -JP'*ErrorS - Lambda_O*JO'*ErrorO;
EM = -JM'*ErrorS;

% NormalMap = Map.Grid./Map.N;
% NormalMap(isnan(NormalMap))=0;
% 
% XH0 = reshape(permute(NormalMap, [2, 1, 3]), Size_i * Size_j * Size_h, 1);

XH0 = reshape(permute(Map.Grid, [2, 1, 3]), Size_i * Size_j * Size_h, 1);


% XH0 = reshape(permute(Map.NormalizeMap, [2, 1, 3]), Size_i * Size_j * Size_h, 1);

EH = -HH*XH0;

EM = EM+EH;

II = [U,W;
      W',V+HH];
  
EE = [EP;EM];
Delta = II\EE;

nP = size(JP,2);
DeltaP = Delta(1:nP);
DeltaM = Delta(nP+1:end);


% Id2_Recover = ones(1,length(Id_Recover));
% DeltaM = sparse(Id_Recover,Id2_Recover,DeltaM',Size_i*Size_j*Size_h,1); 


Delta = [DeltaP;DeltaM];

Sum_Delta = Delta'*Delta;
Mean_Delta = full(Sum_Delta/length(Delta));
Sum_Delta_Pose = DeltaP'*DeltaP;
Mean_Delta_Pose = full(Sum_Delta_Pose/length(DeltaP));

end

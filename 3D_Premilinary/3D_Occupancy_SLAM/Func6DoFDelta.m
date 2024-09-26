function [DeltaP,DeltaM,MeanDelta,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,Map,HH,Param)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

LambdaO = Param.LambdaO;
InfMatO = Param.InfMatO;

IO = FuncGet3DIO(ErrorO,InfMatO(1),InfMatO(2),InfMatO(3),InfMatO(4),InfMatO(5),InfMatO(6));

JP = JP(:,7:end);
JO = JO(:,7:end);

U = JP'*JP+LambdaO*(JO'*IO*JO);
V = JM'*JM;
W = JP'*JM;

ErrorS = sparse(ErrorS);
ErrorO = sparse(ErrorO);
EP = -JP'*ErrorS - LambdaO*JO'*IO*ErrorO;
EM = -JM'*ErrorS;


XH0 = reshape(permute(Map.Grid, [2, 1, 3]), Size_i * Size_j * Size_h, 1);

EH = -HH*XH0;

EM = EM+EH;

II = [U,W;
      W',V+HH];
  
EE = [EP;EM];
Delta = II\EE;

nP = size(JP,2);
DeltaP = Delta(1:nP);
DeltaM = Delta(nP+1:end);

Delta = [DeltaP;DeltaM];

Sum_Delta = Delta'*Delta;
MeanDelta = full(Sum_Delta/length(Delta));
Sum_Delta_Pose = DeltaP'*DeltaP;
MeanDeltaPose = full(Sum_Delta_Pose/length(DeltaP));

end

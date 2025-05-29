function [DeltaP,DeltaM,Mean_Delta,Mean_Delta_Pose] = FuncDelta(JP,JM,JO,ErrorS,ErrorO,Map,Param)
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
THRESHOLD = 1; % if observed number of a cell is lower than this threshold is removed
LambdaO = Param.LambdaO;
JP = JP(:,7:end);
JO = JO(:,7:end);
InfMatO = Param.InfMatO;
IO = FuncGet3DIO(ErrorO,InfMatO(1),InfMatO(2),InfMatO(3),InfMatO(4),InfMatO(5),InfMatO(6));
Mask_JM = (JM~=0);
Sum_Num_JM = sum(Mask_JM,1);
Id_JM = find(Sum_Num_JM<THRESHOLD);
Id_Recover = find(Sum_Num_JM >=THRESHOLD);
JM(:,Id_JM) = [];
U = JP'*JP + LambdaO^2 * (JO'*IO*JO);
V = JM'*JM;
W = JP'*JM;
ErrorS = sparse(ErrorS);
ErrorO = sparse(ErrorO);
EP = -JP'*ErrorS - LambdaO^2*JO'*IO*ErrorO;
EM = -JM'*ErrorS;
InvDiagV = 1./diag(V);
InvDiagV(InvDiagV>10000) = 0;
InvV = spdiags(InvDiagV,0,length(InvDiagV),length(InvDiagV));
IIP = U - W*InvV*W';
EEP = EP - W*InvV*EM;
DeltaP = IIP\EEP;
EEM = EM - W'*DeltaP;
DeltaM = V\EEM;
Id2_Recover = ones(1,length(Id_Recover));
DeltaM = sparse(Id_Recover,Id2_Recover,DeltaM',Size_i*Size_j*Size_h,1); 
Delta = [DeltaP;DeltaM];
Sum_Delta = Delta'*Delta;
Mean_Delta = full(Sum_Delta/length(Delta));
Sum_Delta_Pose = DeltaP'*DeltaP;
Mean_Delta_Pose = full(Sum_Delta_Pose/length(DeltaP));

end

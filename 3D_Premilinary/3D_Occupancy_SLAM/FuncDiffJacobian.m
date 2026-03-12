function [ErrorS,ErrorO,Mean_Error,JP,JM,JO,MapVarId] = FuncDiffJacobian(Map,Pose,Scan,Odom,Param)

if nargin < 5 || isempty(Param)
    Param = struct();
end

NumPose = size(Pose,1);

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

Scale = Map.Scale;
Origin = Map.Origin;
DgridG = Map.DgridG;
NgridG = Map.NgridG;
DgridGu = Map.DgridGu;
DgridGv = Map.DgridGv;
DgridGz = Map.DgridGz;

MinHitCount = 1;
if isfield(Param,'MinHitCount') && Param.MinHitCount > 0
    MinHitCount = Param.MinHitCount;
end

nPts = 0;
Cnti = 1;

cell_ErrorS = cell(1,NumPose);
cell_ErrorO = cell(1,max(NumPose-1,0));

cell_JPID1 = cell(1,NumPose);
cell_JPID2 = cell(1,NumPose);
cell_JPVal = cell(1,NumPose);
cell_JMID1 = cell(1,NumPose);
cell_JMID2 = cell(1,NumPose);
cell_JMVal = cell(1,NumPose);

cell_JOID1 = cell(1,max(NumPose-1,0));
cell_JOID2 = cell(1,max(NumPose-1,0));
cell_JOVal = cell(1,max(NumPose-1,0));

for i=1:NumPose

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));

    Ri = FuncR(RZ',RY',RX');

    xyz = Scan{i}.xyz';
    Oddi = Scan{i}.Odd;

    Si = Ri*xyz+Posei(1:3)';

    Pi = (Si-Origin) / Scale + 1;
    IdBadPi = ~all(isfinite(Pi),1);
    if any(IdBadPi)
        Pi(:,IdBadPi) = [];
        Oddi(IdBadPi) = [];
        xyz(:,IdBadPi) = [];
    end

    CheckVal = DgridG(Pi(2,:),Pi(1,:),Pi(3,:));
    CheckN = NgridG(Pi(2,:),Pi(1,:),Pi(3,:));
    CheckGradu = DgridGu(Pi(2,:),Pi(1,:),Pi(3,:));
    CheckGradv = DgridGv(Pi(2,:),Pi(1,:),Pi(3,:));
    CheckGradz = DgridGz(Pi(2,:),Pi(1,:),Pi(3,:));

    IdNan = isnan(CheckVal) | isnan(CheckN) | isnan(CheckGradu) | isnan(CheckGradv) | isnan(CheckGradz);
    if any(IdNan)
        Pi(:,IdNan) = [];
        Oddi(IdNan) = [];
        xyz(:,IdNan) = [];
    end

    Md = DgridG(Pi(2,:),Pi(1,:),Pi(3,:));
    MN = NgridG(Pi(2,:),Pi(1,:),Pi(3,:));
    MNSafe = MN;
    MNSafe(MNSafe < MinHitCount) = MinHitCount;

    Ei = Md./MNSafe-Oddi';

    IdDeleta = find(MN==0);
    Ei(IdDeleta) = 0;

    cell_ErrorS{i} = Ei';

    dMdPm = [DgridGu(Pi(2,:),Pi(1,:),Pi(3,:));DgridGv(Pi(2,:),Pi(1,:),Pi(3,:));DgridGz(Pi(2,:),Pi(1,:),Pi(3,:))]./MNSafe;

    dRdr = FuncRZ(Posei(6))'*FuncRY(Posei(5))'*FuncdRXdG(Posei(4))';
    dRdp = FuncRZ(Posei(6))'*FuncdRYdB(Posei(5))'*FuncRX(Posei(4))';
    dRdy = FuncdRZdA(Posei(6))'*FuncRY(Posei(5))'*FuncRX(Posei(4))';

    dPdr = (dRdr * xyz)/Scale;
    dPdp = (dRdp * xyz)/Scale;
    dPdy = (dRdy * xyz)/Scale;

    dMdr = sum(dMdPm.*dPdr);
    dMdp = sum(dMdPm.*dPdp);
    dMdy = sum(dMdPm.*dPdy);

    dMdT = dMdPm / Scale;

    dEdP = [dMdT;dMdr;dMdp;dMdy];

    nPtsi = length(Oddi);
    IDi = nPts+1:nPts+nPtsi;
    nPts = nPts+nPtsi;

    dEdPID1 = repmat(IDi,6,1);
    dEdPID2 = repmat(6*(i-1)+1:6*i,nPtsi,1)';

    cell_JPID1{i} = reshape(dEdPID1',[],1);
    cell_JPID2{i} = reshape(dEdPID2',[],1);
    cell_JPVal{i} = reshape(dEdP',[],1);

    u = Pi(1,:);
    v = Pi(2,:);
    h = Pi(3,:);

    u1 = fix(u);
    v1 = fix(v);
    h1 = fix(h);

    dEdm1 = (v1+1-v).*(u1+1-u).*(h1+1-h);
    dEdm2 = (v-v1).*(u1+1-u).*(h1+1-h);
    dEdm3 = (v1+1-v).*(u-u1).*(h1+1-h);
    dEdm4 = (v-v1).*(u-u1).*(h1+1-h);

    dEdm5 = (v1+1-v).*(u1+1-u).*(h-h1);
    dEdm6 = (v-v1).*(u1+1-u).*(h-h1);
    dEdm7 = (v1+1-v).*(u-u1).*(h-h1);
    dEdm8 = (v-v1).*(u-u1).*(h-h1);

    dEdM = [dEdm1;dEdm2;dEdm3;dEdm4;dEdm5;dEdm6;dEdm7;dEdm8] ./ MNSafe;
    dEdM(:,MN==0) = 0;

    IdDeleta = find(dEdM<1e-3);
    dEdM(IdDeleta) = 0;

    dEdMID1 = repmat(IDi,8,1);
    m1ID2 = Size_j*(v1-1)+u1 + (h1-1)*Size_i*Size_j;
    m2ID2 = Size_j*v1+u1 + (h1-1)*Size_i*Size_j;
    m3ID2 = Size_j*(v1-1)+u1+1 + (h1-1)*Size_i*Size_j;
    m4ID2 = Size_j*v1+u1+1 + (h1-1)*Size_i*Size_j;

    m5ID2 = Size_j*(v1-1)+u1 + h1*Size_i*Size_j;
    m6ID2 = Size_j*v1+u1 + h1*Size_i*Size_j;
    m7ID2 = Size_j*(v1-1)+u1+1 + h1*Size_i*Size_j;
    m8ID2 = Size_j*v1+u1+1 + h1*Size_i*Size_j;

    dEdMID2 = [m1ID2;m2ID2;m3ID2;m4ID2;m5ID2;m6ID2;m7ID2;m8ID2];

    cell_JMID1{i} = reshape(dEdMID1',[],1);
    cell_JMID2{i} = reshape(dEdMID2',[],1);
    cell_JMVal{i} = reshape(dEdM',[],1);

    if i < NumPose
        P1 = Pose(i,:)';
        P2 = Pose(i+1,:)';

        dT = Ri'*(P2(1:3)-P1(1:3));
        dPhi = P2(4:6)-P1(4:6);
        for j=1:3
            while dPhi(j)>pi || dPhi(j)<-pi
                dPhi(j) = wrap(dPhi(j));
            end
        end
        EOi = [dT;dPhi]-Odom(i+1,:)';

        cell_ErrorO{i} = EOi;
        dTdT1 = -Ri';
        dTdT2 = Ri';
        dTdPhi1 = dRdr'*(P2(1:3)-P1(1:3));
        dTdPhi2 = dRdp'*(P2(1:3)-P1(1:3));
        dTdPhi3 = dRdy'*(P2(1:3)-P1(1:3));
        dPhid1 = -ones(1,3);
        dPhid2 = ones(1,3);

        a = (6*i-5:6*i)';
        b = (6*i+1:6*i+6)';
        cell_JOID1{i} = [Cnti;Cnti;Cnti;Cnti+1;Cnti+1;Cnti+1;Cnti+2;Cnti+2;Cnti+2;Cnti;Cnti;Cnti;Cnti+1;Cnti+1;Cnti+1;Cnti+2;Cnti+2;Cnti+2;...
            Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;...
            Cnti+3;Cnti+4;Cnti+5;Cnti+3;Cnti+4;Cnti+5];
        cell_JOID2{i} = [a(1:3);  a(1:3); a(1:3); b(1:3);  b(1:3); b(1:3);...
            a(4); a(4); a(4); a(5); a(5); a(5); a(6); a(6); a(6); ...
            a(4); a(5); a(6); b(4); b(5); b(6)];
        cell_JOVal{i} = [dTdT1(1,:),dTdT1(2,:), dTdT1(3,:),dTdT2(1,:),dTdT2(2,:),dTdT2(3,:),...
            dTdPhi1',dTdPhi2',dTdPhi3',...
            dPhid1,dPhid2];
        Cnti = Cnti+6;
    end

end

ErrorS = vertcat(cell_ErrorS{:});
clear cell_ErrorS;
JPID1 = vertcat(cell_JPID1{:});
clear cell_JPID1;
JPID2 = vertcat(cell_JPID2{:});
clear cell_JPID2;
JPVal = vertcat(cell_JPVal{:});
clear cell_JPVal;

JMID1 = vertcat(cell_JMID1{:});
clear cell_JMID1;
JMID2 = vertcat(cell_JMID2{:});
clear cell_JMID2;
JMVal = vertcat(cell_JMVal{:});
clear cell_JMVal;

if isempty(cell_ErrorO)
    ErrorO = zeros(0,1);
    JOID1 = zeros(0,1);
    JOID2 = zeros(0,1);
    JOVal = [];
else
    ErrorO = vertcat(cell_ErrorO{:});
    JOID1 = vertcat(cell_JOID1{:});
    JOID2 = vertcat(cell_JOID2{:});
    JOVal = horzcat(cell_JOVal{:});
end
clear cell_ErrorO cell_JOID1 cell_JOID2 cell_JOVal;

JO = sparse(JOID1,JOID2,JOVal);
clear JOID1 JOID2 JOVal;

if isempty(ErrorS)
    Mean_Error = 0;
else
    Sum_Error = ErrorS'*ErrorS;
    Mean_Error = Sum_Error/length(ErrorS);
end

UseRobustKernel = isfield(Param,'UseRobustKernel') && Param.UseRobustKernel;
if UseRobustKernel && ~isempty(ErrorS)
    RobustKernel = 'huber';
    if isfield(Param,'RobustKernel') && ~isempty(Param.RobustKernel)
        RobustKernel = Param.RobustKernel;
    end
    RobustDelta = 1.5;
    if isfield(Param,'RobustDelta')
        RobustDelta = Param.RobustDelta;
    end
    RobustMinWeight = 0.0;
    if isfield(Param,'RobustMinWeight')
        RobustMinWeight = Param.RobustMinWeight;
    end
    RobustUseMAD = 1;
    if isfield(Param,'RobustUseMAD')
        RobustUseMAD = Param.RobustUseMAD;
    end

    WeightS = FuncRobustWeights(ErrorS, RobustKernel, RobustDelta, RobustMinWeight, RobustUseMAD);
    SqrtWeightS = sqrt(WeightS);
    ErrorS = ErrorS .* SqrtWeightS;
    JPVal = JPVal .* SqrtWeightS(double(JPID1));
    JMVal = JMVal .* SqrtWeightS(double(JMID1));
end

JPVal = double(JPVal);
JP = sparse(JPID1,JPID2,JPVal);
clear JPID1 JPID2 JPVal;

UseActiveVoxelOptimization = isfield(Param,'UseActiveVoxelOptimization') && Param.UseActiveVoxelOptimization;
ActiveVoxelNeighborLayers = 0;
if isfield(Param,'ActiveVoxelNeighborLayers')
    ActiveVoxelNeighborLayers = Param.ActiveVoxelNeighborLayers;
end
UseTruncatedRegionOptimization = isfield(Param,'UseTruncatedRegionOptimization') && Param.UseTruncatedRegionOptimization;

JMVal = double(JMVal);
JMID1 = double(JMID1);
JMID2 = double(JMID2);

HitVarId = unique(JMID2);
CandidateVarId = [];

if UseActiveVoxelOptimization && ~isempty(HitVarId)
    CandidateVarId = FuncExpandActiveVoxelIds(HitVarId, Size_i, Size_j, Size_h, ActiveVoxelNeighborLayers);
end

if UseTruncatedRegionOptimization
    TruncatedVarId = FuncFindCellOptimized(Map, Param);
    if isempty(CandidateVarId)
        CandidateVarId = TruncatedVarId;
    else
        CandidateVarId = intersect(CandidateVarId, TruncatedVarId);
    end
end

if ~isempty(CandidateVarId)
    MapVarId = unique(double(CandidateVarId(:)));
    [tf, LocalColId] = ismember(JMID2, MapVarId);

    if UseTruncatedRegionOptimization
        KeepRowMask = false(nPts,1);
        if any(tf)
            KeepRowMask(unique(JMID1(tf))) = true;
        end

        nPtsKeep = nnz(KeepRowMask);
        if nPtsKeep > 0
            ErrorS = ErrorS(KeepRowMask);
            JP = JP(KeepRowMask,:);

            RowMap = zeros(nPts,1);
            RowMap(KeepRowMask) = 1:nPtsKeep;

            NewRow = RowMap(JMID1(tf));
            NewCol = LocalColId(tf);
            NewVal = JMVal(tf);
            JM = sparse(NewRow, NewCol, NewVal, nPtsKeep, length(MapVarId));
        else
            ErrorS = zeros(0,1);
            JP = sparse(0,size(JP,2));
            MapVarId = zeros(0,1);
            JM = sparse(0,0);
        end
    else
        JMID1 = JMID1(tf);
        LocalColId = LocalColId(tf);
        JMVal = JMVal(tf);
        JM = sparse(JMID1, LocalColId, JMVal, nPts, length(MapVarId));
    end
else
    if UseTruncatedRegionOptimization
        MapVarId = zeros(0,1);
        ErrorS = zeros(0,1);
        JP = sparse(0,size(JP,2));
        JM = sparse(0,0);
    else
        MapVarId = [];
        JM = sparse(JMID1,JMID2,JMVal,nPts,Size_i*Size_j*Size_h);
    end
end

clear JMID1 JMID2 JMVal;

if isempty(ErrorS)
    Mean_Error = 0;
else
    Mean_Error = (ErrorS' * ErrorS) / length(ErrorS);
end

end

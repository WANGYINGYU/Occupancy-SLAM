function [ErrorS,ErrorO,Mean_Error,JP,JM,JO,MapVarId] = FuncDiffJacobian(Map,Pose,Scan,Odom,Param)

if nargin < 5 || isempty(Param)
    Param = struct();
end

NumPose = size(Pose,1);
NumScan = numel(Scan);
NumOdom = size(Odom,1);

if NumPose == 0
    ErrorS = zeros(0,1);
    ErrorO = zeros(0,1);
    Mean_Error = inf;
    JP = sparse(0,0);
    JM = sparse(0,0);
    JO = sparse(0,0);
    MapVarId = [];
    return;
end

% Pad inputs to avoid parfor sliced-index supply overflow on i+1 access.
PosePad = [Pose; Pose(end,:)];
ScanPad = cell(1,NumPose);
EmptyObs = struct('xyz', zeros(0,3), 'Odd', zeros(0,1));
for k = 1:NumPose
    if k <= NumScan
        ScanPad{k} = Scan{k};
    else
        ScanPad{k} = EmptyObs;
    end
end

OdomCols = size(Odom,2);
OdomPad = zeros(max(NumOdom, NumPose+1), OdomCols);
if NumOdom > 0
    OdomPad(1:NumOdom,:) = Odom;
end
MaxOdomEdge = min(NumPose-1, NumOdom-1);

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

Scale = Map.Scale;
Origin = Map.Origin;
WorldToMapR = eye(3);
WorldToMapT = zeros(3,1);
if isfield(Map,'WorldToMapR') && isequal(size(Map.WorldToMapR),[3,3])
    WorldToMapR = double(Map.WorldToMapR);
end
if isfield(Map,'WorldToMapT') && numel(Map.WorldToMapT) == 3
    WorldToMapT = double(Map.WorldToMapT(:));
end
DgridG = Map.DgridG;
NgridG = Map.NgridG;
DgridGu = Map.DgridGu;
DgridGv = Map.DgridGv;
DgridGz = Map.DgridGz;

MinHitCount = 1;
if isfield(Param,'MinHitCount') && Param.MinHitCount > 0
    MinHitCount = Param.MinHitCount;
end

nPtsPose = zeros(NumPose,1,'uint32');
ObsInputCount = zeros(NumPose,1,'uint32');
AfterFiniteWorldCount = zeros(NumPose,1,'uint32');
AfterInterpCount = zeros(NumPose,1,'uint32');

cell_ErrorS = cell(1,NumPose);
cell_ErrorO = cell(1,NumPose);

cell_JPID1 = cell(1,NumPose);
cell_JPID2 = cell(1,NumPose);
cell_JPVal = cell(1,NumPose);
cell_JMID1 = cell(1,NumPose);
cell_JMID2 = cell(1,NumPose);
cell_JMVal = cell(1,NumPose);

cell_JOID1 = cell(1,NumPose);
cell_JOID2 = cell(1,NumPose);
cell_JOVal = cell(1,NumPose);

parfor i=1:NumPose
    cell_ErrorS{i} = zeros(0,1);
    cell_JPID1{i} = zeros(0,1,'uint32');
    cell_JPID2{i} = zeros(0,1,'uint32');
    cell_JPVal{i} = zeros(0,1);
    cell_JMID1{i} = zeros(0,1,'uint32');
    cell_JMID2{i} = zeros(0,1,'uint32');
    cell_JMVal{i} = zeros(0,1);
    cell_ErrorO{i} = zeros(0,1);
    cell_JOID1{i} = zeros(0,1,'uint32');
    cell_JOID2{i} = zeros(0,1,'uint32');
    cell_JOVal{i} = zeros(1,0);

    Posei = PosePad(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));

    Ri = FuncR(RZ',RY',RX');

    dRdr = FuncRZ(Posei(6))'*FuncRY(Posei(5))'*FuncdRXdG(Posei(4))';
    dRdp = FuncRZ(Posei(6))'*FuncdRYdB(Posei(5))'*FuncRX(Posei(4))';
    dRdy = FuncdRZdA(Posei(6))'*FuncRY(Posei(5))'*FuncRX(Posei(4))';

    % Scan residual/Jacobian term (robust to Pose/Scan length mismatch).
    Scani = ScanPad{i};
    if isstruct(Scani) && isfield(Scani,'xyz') && isfield(Scani,'Odd') && ~isempty(Scani.xyz)
        xyz = Scani.xyz';
        Oddi = Scani.Odd;

        if isempty(Oddi) || isempty(xyz)
            nPtsi = uint32(0);
        else
            nUse = min(size(xyz,2), length(Oddi));
            xyz = xyz(:,1:nUse);
            Oddi = Oddi(1:nUse);
            ObsInputCount(i) = uint32(nUse);

            Si = Ri*xyz+Posei(1:3)';
            SiMap = FuncWorldToMapFrame(Si, WorldToMapR, WorldToMapT);

            Pi = (SiMap-Origin) / Scale + 1;
            IdBadPi = ~all(isfinite(Pi),1);
            if any(IdBadPi)
                Pi(:,IdBadPi) = [];
                Oddi(IdBadPi) = [];
                xyz(:,IdBadPi) = [];
            end
            AfterFiniteWorldCount(i) = uint32(size(xyz,2));

            CheckVal = DgridG(Pi(2,:),Pi(1,:),Pi(3,:));
            CheckN = NgridG(Pi(2,:),Pi(1,:),Pi(3,:));
            CheckGradu = DgridGu(Pi(2,:),Pi(1,:),Pi(3,:));
            CheckGradv = DgridGv(Pi(2,:),Pi(1,:),Pi(3,:));
            CheckGradz = DgridGz(Pi(2,:),Pi(1,:),Pi(3,:));

            IdNan = isnan(CheckVal) | isnan(CheckN) | isnan(CheckGradu) | isnan(CheckGradv) | isnan(CheckGradz);
            IdValidInterp = ~IdNan;
            if any(IdNan)
                Pi(:,IdNan) = [];
                Oddi(IdNan) = [];
                xyz(:,IdNan) = [];
            end
            AfterInterpCount(i) = uint32(length(Oddi));

            Md = CheckVal(IdValidInterp);
            MN = CheckN(IdValidInterp);
            MNSafe = MN;
            MNSafe(MNSafe < MinHitCount) = MinHitCount;

            Ei = Md./MNSafe-Oddi';
            IdDeleta = find(MN==0);
            Ei(IdDeleta) = 0;
            cell_ErrorS{i} = Ei';

            dMdPm = [CheckGradu(IdValidInterp);CheckGradv(IdValidInterp);CheckGradz(IdValidInterp)]./MNSafe;

            dPdr = (WorldToMapR * dRdr * xyz)/Scale;
            dPdp = (WorldToMapR * dRdp * xyz)/Scale;
            dPdy = (WorldToMapR * dRdy * xyz)/Scale;

            dMdr = sum(dMdPm.*dPdr);
            dMdp = sum(dMdPm.*dPdp);
            dMdy = sum(dMdPm.*dPdy);

            dMdT = (WorldToMapR' * dMdPm) / Scale;
            dEdP = [dMdT;dMdr;dMdp;dMdy];

            nPtsi = uint32(length(Oddi));
            if nPtsi > 0
                IDi = uint32(1:double(nPtsi));
                dEdPID1 = repmat(IDi,6,1);
                dEdPID2 = uint32(repmat(6*(i-1)+1:6*i,double(nPtsi),1)');
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

                dEdMID2 = uint32([m1ID2;m2ID2;m3ID2;m4ID2;m5ID2;m6ID2;m7ID2;m8ID2]);
                cell_JMID1{i} = reshape(dEdMID1',[],1);
                cell_JMID2{i} = reshape(dEdMID2',[],1);
                cell_JMVal{i} = reshape(dEdM',[],1);
            end
        end
        nPtsPose(i) = nPtsi;
    end

    if i <= MaxOdomEdge
        P1 = PosePad(i,:)';
        P2 = PosePad(i+1,:)';

        dT = Ri'*(P2(1:3)-P1(1:3));
        dPhi = P2(4:6)-P1(4:6);
        for j=1:3
            while dPhi(j)>pi || dPhi(j)<-pi
                dPhi(j) = wrap(dPhi(j));
            end
        end
        EOi = [dT;dPhi]-OdomPad(i+1,:)';

        cell_ErrorO{i} = EOi;
        dTdT1 = -Ri';
        dTdT2 = Ri';
        dTdPhi1 = dRdr'*(P2(1:3)-P1(1:3));
        dTdPhi2 = dRdp'*(P2(1:3)-P1(1:3));
        dTdPhi3 = dRdy'*(P2(1:3)-P1(1:3));
        dPhid1 = -ones(1,3);
        dPhid2 = ones(1,3);

        a = uint32((6*i-5:6*i)');
        b = uint32((6*i+1:6*i+6)');
        Cnti = uint32(6*(i-1)+1);
        cell_JOID1{i} = [Cnti;Cnti;Cnti;Cnti+1;Cnti+1;Cnti+1;Cnti+2;Cnti+2;Cnti+2;Cnti;Cnti;Cnti;Cnti+1;Cnti+1;Cnti+1;Cnti+2;Cnti+2;Cnti+2;...
            Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;...
            Cnti+3;Cnti+4;Cnti+5;Cnti+3;Cnti+4;Cnti+5];
        cell_JOID2{i} = [a(1:3);  a(1:3); a(1:3); b(1:3);  b(1:3); b(1:3);...
            a(4); a(4); a(4); a(5); a(5); a(5); a(6); a(6); a(6); ...
            a(4); a(5); a(6); b(4); b(5); b(6)];
        cell_JOVal{i} = [dTdT1(1,:),dTdT1(2,:), dTdT1(3,:),dTdT2(1,:),dTdT2(2,:),dTdT2(3,:),...
            dTdPhi1',dTdPhi2',dTdPhi3',...
            dPhid1,dPhid2];
    end

end

nPts = double(sum(nPtsPose));
nPoseVar = 6 * NumPose;

ZeroScanPoseId = find(nPtsPose == 0);
if ~isempty(ZeroScanPoseId)
    NumZero = length(ZeroScanPoseId);
    NumShow = min(20, NumZero);
    ShowId = double(ZeroScanPoseId(1:NumShow));
    IdStr = sprintf('%d,', ShowId);
    IdStr(end) = [];
    if NumZero > NumShow
        IdStr = [IdStr, ', ...']; 
    end

    IdObsEmpty = ZeroScanPoseId(ObsInputCount(ZeroScanPoseId) == 0);
    IdWorldOut = ZeroScanPoseId(ObsInputCount(ZeroScanPoseId) > 0 & AfterFiniteWorldCount(ZeroScanPoseId) == 0);
    IdInterpNan = ZeroScanPoseId(AfterFiniteWorldCount(ZeroScanPoseId) > 0 & AfterInterpCount(ZeroScanPoseId) == 0);

    ReasonStr = sprintf('reason_count={obs_empty:%d, world_out:%d, interp_nan:%d}', ...
                        length(IdObsEmpty), length(IdWorldOut), length(IdInterpNan));
    error(['FuncDiffJacobian:ZeroScanResidual'], ...
          ['Detected %d/%d poses with zero scan residuals. Pose indices: [%s]. %s. ' ...
           'Observations were likely emptied by preprocessing/clipping or projected outside map bounds.'], ...
          NumZero, NumPose, IdStr, ReasonStr);
end

RowOffset = zeros(NumPose,1,'uint32');
for i = 2:NumPose
    RowOffset(i) = RowOffset(i-1) + nPtsPose(i-1);
end
for i = 1:NumPose
    if ~isempty(cell_JPID1{i})
        cell_JPID1{i} = cell_JPID1{i} + RowOffset(i);
    end
    if ~isempty(cell_JMID1{i})
        cell_JMID1{i} = cell_JMID1{i} + RowOffset(i);
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
    JOID1 = zeros(0,1,'uint32');
    JOID2 = zeros(0,1,'uint32');
    JOVal = zeros(0,1);
else
    ErrorO = vertcat(cell_ErrorO{:});
    JOID1 = vertcat(cell_JOID1{:});
    JOID2 = vertcat(cell_JOID2{:});
    JOVal = horzcat(cell_JOVal{:})';
end
clear cell_ErrorO cell_JOID1 cell_JOID2 cell_JOVal;

JO = sparse(double(JOID1),double(JOID2),double(JOVal),length(ErrorO),nPoseVar);
clear JOID1 JOID2 JOVal;

if isempty(ErrorS)
    Mean_Error = inf;
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
JP = sparse(double(JPID1),double(JPID2),JPVal,nPts,nPoseVar);
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
    if isfield(Param,'FixedTruncatedVarId')
        TruncatedVarId = unique(double(Param.FixedTruncatedVarId(:)));
    else
        TruncatedVarId = FuncFindCellOptimized(Map, Param);
    end
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
    Mean_Error = inf;
else
    Mean_Error = (ErrorS' * ErrorS) / length(ErrorS);
end

end

function [JP,JM,JO,ErrorS,ErrorO,OccVal,MeanErrorObs] = FuncDiffSubmapJoiningJacobianUneven2(GlobalMap,SubMap,Pose,Odom,Param)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Jacobians for Occupancy Submap Joining Problem
% 1) Generate Observations of Submap Joining Problem by Projecting Global Map to Local Maps
% 2) Calculate Jacobian of Observation Term w.r.t. Poses
% 3) Calculate Jacobian of Observation Term w.r.t. Global Map
% 4) Calculate Jacobian of Odometry Term w.r.t. Poses (TODO)
% Code Witten by Yingyu Wang
% 30/05/2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GlobalOrigin  = GlobalMap.Origin;
% GlobalScale = GlobalMap.Scale;
GlobalScaleX = Param.GlobalScaleX;
GlobalScaleY = Param.GlobalScaleY;
GlobalScaleZ = Param.GlobalScaleZ;
GlobalGrid = GlobalMap.Grid;
GlobalN = GlobalMap.N;
GlobalSize_i = GlobalMap.Size_i;
GlobalSize_j = GlobalMap.Size_j;
GlobalSize_h = GlobalMap.Size_h;

NumPose = size(Pose,1);

cell_JPID1 = cell(1,NumPose);
cell_JPID2 = cell(1,NumPose);
cell_JPVal = cell(1,NumPose);

cell_JMID1 = cell(1,NumPose);
cell_JMID2 = cell(1,NumPose);
cell_JMVal = cell(1,NumPose);


if Param.LambdaO~=0
    cell_JOID1 = cell(1,NumPose);
    cell_JOID2 = cell(1,NumPose);
    cell_JOVal = cell(1,NumPose);
    cell_ErrorO = cell(1,NumPose);
end

cell_ErrorS = cell(1,NumPose);
cell_OccVal = cell(1,NumPose);

nPts = 0;
Cnti = 1;

% GlobalMapPoints = FuncTruncatedCell(GlobalMap,Param);
% Id = sub2ind(size(GlobalN),GlobalMapPoints(:,1),GlobalMapPoints(:,2),GlobalMapPoints(:,3)); 


Id = find(isnan(GlobalGrid)==0);

[I,J,H] = ind2sub(size(GlobalN),Id);
GlobalMapPoints = [J,I,H];
GlobalP = (GlobalMapPoints - 1);
GlobalSiPre = [GlobalP(:,1) * GlobalScaleX,GlobalP(:,2) * GlobalScaleY,GlobalP(:,3) * GlobalScaleZ] + GlobalOrigin';




for i=1:NumPose
    % find observed cells in global map using global hit map  
    GlobalIdi = Id;
    GlobalSi = GlobalSiPre;
    % GlobalSi = (GlobalMapPoints - 1) * GlobalScale + GlobalOrigin';

    %% Process Observations
    LocalOrigin = SubMap{i}.Origin;
    LocalScaleX = SubMap{i}.ScaleX;
    LocalScaleY = SubMap{i}.ScaleY;
    LocalScaleZ = SubMap{i}.ScaleZ;
    
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));    
    Ri = FuncR(RZ',RY',RX');
    
    % project cells of global map to local map i
    Coef = (GlobalSi' - Posei(1:3)');
    LocalXYZi = Ri'*Coef-LocalOrigin;

    LocalMapPoints = [LocalXYZi(1,:)./LocalScaleX;LocalXYZi(2,:)./LocalScaleY;LocalXYZi(3,:)./LocalScaleZ] + 1;

    % delete points out of map
    IdOut = find((LocalMapPoints(1,:)>=1).*(LocalMapPoints(2,:)>=1).*(LocalMapPoints(3,:)>=1) ...
        .*(LocalMapPoints(1,:)<=SubMap{i}.Size_j).*(LocalMapPoints(2,:)<=SubMap{i}.Size_i).*(LocalMapPoints(3,:)<=SubMap{i}.Size_h)==0);
    LocalMapPoints(:,IdOut) = [];
    Coef(:,IdOut) = [];
   
    GlobalIdi(IdOut) = []; % calculate the corresponding cells in the global map
    Val = SubMap{i}.DgridG(LocalMapPoints(2,:),LocalMapPoints(1,:),LocalMapPoints(3,:));

  
    %% caculate Observation Errors
    Weight = SubMap{i}.NgridG(LocalMapPoints(2,:),LocalMapPoints(1,:),LocalMapPoints(3,:)) ./ GlobalN(GlobalIdi)';

    IDNaN = isnan(Weight)==1 ;
    Weight(IDNaN) = 1;
   

    IDInf = isinf(Weight)==1 ;
    Weight(IDInf) = 1;

    GlobalVal = GlobalGrid(GlobalIdi)'.*Weight;

    Ei = GlobalVal - Val;

    cell_OccVal{i} = Val';
    cell_ErrorS{i} = Ei';
    NumObs = size(LocalMapPoints,2);

    %% Calculate Jacobian of Observation Term w.r.t. Poses
    % derevative of local map w.r.t. projected points
    
    dLdP = [SubMap{i}.DgridGu(LocalMapPoints(2,:),LocalMapPoints(1,:),LocalMapPoints(3,:))./LocalScaleX; SubMap{i}.DgridGv(LocalMapPoints(2,:),LocalMapPoints(1,:),LocalMapPoints(3,:))./LocalScaleY;...
        SubMap{i}.DgridGz(LocalMapPoints(2,:),LocalMapPoints(1,:),LocalMapPoints(3,:))./LocalScaleZ];

    % derevative projected points w.r.t. Translation and Rotation
    dPdT = -Ri';
    

    dRdr = FuncRZ(Posei(6))'*FuncRY(Posei(5))'*FuncdRXdG(Posei(4))';
    dRdp = FuncRZ(Posei(6))'*FuncdRYdB(Posei(5))'*FuncRX(Posei(4))';
    dRdy = FuncdRZdA(Posei(6))'*FuncRY(Posei(5))'*FuncRX(Posei(4))';


    dPdr = (dRdr' * Coef);
    dPdp = (dRdp' * Coef);
    dPdy = (dRdy' * Coef);

    dPdX = repmat(dPdT(:,1),1,NumObs);
    dPdY = repmat(dPdT(:,2),1,NumObs);
    dPdZ = repmat(dPdT(:,3),1,NumObs);

    dEdX = sum(dLdP.*dPdX);
    dEdY = sum(dLdP.*dPdY);
    dEdZ = sum(dLdP.*dPdZ);

    dEdT = - [dEdX;dEdY;dEdZ]; % Checked 

    dEdr = sum(dLdP .* dPdr);
    dEdp = sum(dLdP .* dPdp);
    dEdy = sum(dLdP .* dPdy);

    dEdEul = -[dEdr;dEdp;dEdy];

    dEdPose = [dEdT;dEdEul];

  
    IDi = nPts+1:nPts+NumObs;
    nPts = nPts+NumObs;

    dEdPID1 = repmat(IDi',6,1);
    dEdPID2 = repmat(6*(i-1)+1:6*i,NumObs,1)';
    
    cell_JPID1{i} = reshape(dEdPID1',[],1);
    cell_JPID2{i} = reshape(dEdPID2',[],1);
    cell_JPVal{i} = reshape(dEdPose',[],1);

    %% Calculate Jacobian of Observation Term w.r.t. Global Map
    [GlobalIdI,GlobalIdJ,GlobalIdH] = ind2sub(size(GlobalGrid),GlobalIdi);
    % convert sub index to the variable order in state vector
    IdVar = (GlobalIdH - 1).* GlobalSize_i*GlobalSize_j + (GlobalIdI - 1) * GlobalSize_j + GlobalIdJ;
    cell_JMID1{i} = IDi';
    cell_JMID2{i} = IdVar;
    cell_JMVal{i} = Weight';
    % cell_JMVal{i} = 1./GlobalN(GlobalIdi);

    %% Calculate Jacobian of Odometry Term w.r.t. Poses
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

JPID1 = vertcat(cell_JPID1{:});
JPID2 = vertcat(cell_JPID2{:});
JPVal = vertcat(cell_JPVal{:});
JP = sparse(JPID1,JPID2,JPVal);

JMID1 = vertcat(cell_JMID1{:});
JMID2 = vertcat(cell_JMID2{:});
JMVal = vertcat(cell_JMVal{:});

JMVal = double(JMVal);
JMID1 = double(JMID1);
JMID2 = double(JMID2);

JM = sparse(JMID1,JMID2,JMVal,nPts,GlobalSize_i*GlobalSize_j*GlobalSize_h);

ErrorO = vertcat(cell_ErrorO{:});
JOID1 = vertcat(cell_JOID1{:});
JOID2 = vertcat(cell_JOID2{:});
JOVal = horzcat(cell_JOVal{:});
JO = sparse(JOID1,JOID2,JOVal);

ErrorS = vertcat(cell_ErrorS{:});
OccVal = vertcat(cell_OccVal{:});


SumErrorObs = ErrorS'*ErrorS;

MeanErrorObs = sqrt(SumErrorObs)/(length(ErrorS));
MeanErrorObs = full(MeanErrorObs);


end

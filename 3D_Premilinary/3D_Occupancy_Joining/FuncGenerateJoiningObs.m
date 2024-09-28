function FuncGenerateJoiningObs(Pose,GlobalMap,Submap,Param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Observations of Submap Joining Problem by Projecting Global Map to Local Maps
% Code Witten by Yingyu Wang
% 30/05/2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GlobalOrigin  = GlobalMap.Origin;
GlobalScale = GlobalMap.Scale;

NumPose = size(Pose,1);

for i=1:NumPose
    LocalOrigin = Submap{i}.Origin;
    LocalScale = Submap{i}.Scale;
    
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));    
    Ri = FuncR(RZ',RY',RX');

    % find observed cells in global map
    Id = find(GlobalMap.Grid~=0);
    Oddi = GlobalMap.Grid(Id);
    [I,J,H] = ind2sub(size(GlobalMap.Grid),Id);
    MapPoints = [I,J,H];
    % project cells of global map to local map i
    GlobalSi = (MapPoints - 1) * GlobalScale + GlobalOrigin';
    LocalXYZi = Ri'*(GlobalSi - Posei(1:3)');
    LocalMapPoints = (LocalXYZi - LocalOrigin')/LocalScale + 1;

end    




end
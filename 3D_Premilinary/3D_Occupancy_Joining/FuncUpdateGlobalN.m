function [GlobalMap,Param] = FuncUpdateGlobalN(GlobalMap,Pose,LocalObs,Param)

NumPose = size(Pose,1);
GlobalScale = Param.GlobalScale;

PointsGlobal = cell(1,NumPose);
for i=1:NumPose
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    XYZi = LocalObs{i}.xyz;
    Si = Ri*XYZi'+Posei(1:3)';
    PointsGlobal{i} = Si;
end 

GlobalOrigin = Param.GlobalOrigin;

Size_i = Param.GlobalSize_i;
Size_j = Param.GlobalSize_j;
Size_h = Param.GlobalSize_h;


N = zeros(Size_i, Size_j, Size_h);
for i=1:NumPose
    Si = PointsGlobal{i};
    Ni = LocalObs{i}.N;
    XYZ = Si-GlobalOrigin;
    XYZ3 = XYZ/GlobalScale + 1;      
    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));
    Ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;
    TemN = accumarray(Ind',Ni,[size(N,1)*size(N,2)*size(N,3),1]);
    TemN_reshaped = reshape(TemN, size(N));
    N = N + TemN_reshaped; 
  
end  

Check = GlobalMap.Grid./N;
GlobalMap.Grid(abs(Check)>=1000) = 0;
N(abs(Check)>=1000) = 0;
GlobalMap.N = N;
end

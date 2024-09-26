function SelectObs = FuncSelectObs(Map,HighObs,Pose,Param)

LowSelectInd = Param.LowSelectInd;
Origin = Param.Origin;
Scale = Param.Scale;

SelectObs = cell(1,length(HighObs));

for i=1:length(HighObs)
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    xyz = double(HighObs{i}.xyz');
    Oddi = HighObs{i}.Odd;
    Si = Ri*xyz+Posei(1:3)';
    XYZ3 = (Si-Origin) / Scale + 1;
    XYZ = fix(XYZ3);
    Ind = sub2ind(size(Map.Grid),XYZ(2,:),XYZ(1,:),XYZ(3,:));
    [isMember, ~] = ismember(Ind, LowSelectInd);
    Id = find(isMember==1);
    xyz_select = xyz(:,Id);
    odd_select = Oddi(Id);
    SelectObs{i}.xyz = xyz_select';
    SelectObs{i}.Odd = odd_select;
end


end
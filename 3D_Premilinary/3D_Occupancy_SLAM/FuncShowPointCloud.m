function FuncShowPointCloud(Pose,Scan,FigId)

PointsGlobal = [];
parfor i = 1:length(Pose)
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    xyz = single(Scan{i});
    Si = Ri*xyz+Posei(1:3)';
    PointsGlobal = [PointsGlobal,Si];
end
figure(FigId)
pcshow(PointsGlobal')

end
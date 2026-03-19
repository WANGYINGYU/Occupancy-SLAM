function FuncDrawOccupancyMap(Pose,Scan,Resolution)


Quat = FuncEuler2Quat(Pose(:,4:6));

PoseQuat = [Pose(:,1:3),Quat];

Param.Scale = 2;

OccupancyMap = occupancyMap3D(Resolution,"ProbabilitySaturation",[0.001,0.999]);

for i = 1:NumPose
    HitPi = Scan{i}(1:end,:);
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];
    insertPointCloud(OccupancyMap,PoseQuat(i,:),HitPi',Param.MaxRange);
end

figure(2)
show(OccupancyMap)
function FuncShowOccupancyMap(Scan,Pose,Resolution,MaxRange,Fig_No)
NumPose = length(Scan);

Quat = FuncEuler2Quat(Pose(:,4:6));

PoseQuat = [Pose(:,1:3),Quat];

OccupancyMap = occupancyMap3D(Resolution,"ProbabilitySaturation",[0.001,0.999]);

for i = 1:NumPose
    HitPi = Scan{i}(1:end,:);
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];
    insertPointCloud(OccupancyMap,PoseQuat(i,:),HitPi',MaxRange);
end

figure(Fig_No)
show(OccupancyMap)
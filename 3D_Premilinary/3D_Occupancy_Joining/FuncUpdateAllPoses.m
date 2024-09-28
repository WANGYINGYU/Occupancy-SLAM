function [AllPose,Trajectory] = FuncUpdateAllPoses(CoordinatePose,AllPose,Param)

NumSubmap = size(CoordinatePose,1);
Trajectory = [];

OverlapScan = Param.OverlapScan;

Trajectory = [Trajectory;AllPose{1}];

for i=2:NumSubmap

    NumOverlapScan = OverlapScan(i-1);
    
    MapPose_Pre = AllPose{i}(NumOverlapScan+1,:);
    MapPose_Current = CoordinatePose(i,:);
    
    MapTFMat_Pre = FuncTFMatrix(MapPose_Pre);
    MapTFMat_Current = FuncTFMatrix(MapPose_Current);
    % RelativeTF =  MapTFMat_Pre * inv(MapTFMat_Current);
    RelativeTF =  MapTFMat_Current * inv(MapTFMat_Pre);
    
    

    % Transfer all poses at cuurent frame
    for j=NumOverlapScan+1:size(AllPose{i},1)
        Pose_Pre_j = AllPose{i}(j,:);
        Mat_Pre_j = FuncTFMatrix(Pose_Pre_j);
        Mat_Transfer_j = RelativeTF * Mat_Pre_j;
        [raw,pitch,yaw] = FuncRotMatrix2Euler(Mat_Transfer_j(1:3,1:3));
        Pose_Transfer_j = [Mat_Transfer_j(1:3,4)',raw,pitch,yaw];
        AllPose{i}(j,:) = Pose_Transfer_j;
        Trajectory = [Trajectory;Pose_Transfer_j];
    end
    % end
end    



end

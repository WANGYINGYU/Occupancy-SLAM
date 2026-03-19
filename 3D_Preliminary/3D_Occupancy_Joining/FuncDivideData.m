function [SubmapScan,SubmapPose,CoordinatePose] = FuncDivideData(Scan,Pose,Param)

NumSubmap = Param.NumSubmap;
DividePlan = Param.DividePlan;
OverlapScan = Param.OverlapScan;

SubmapScan = cell(1,NumSubmap);
SubmapPose = cell(1,NumSubmap);
CoordinatePose = zeros(NumSubmap,6);


for i = 1:NumSubmap
    
    if i <NumSubmap
        DivideNumber = DividePlan(i);
    else
        DivideNumber = size(Scan,2);
    end
    
    if i==1
        SubmapScan{i} = Scan(1:DivideNumber-1);
        SubmapPose{i} = Pose(1:DivideNumber-1,:);
        CoordinatePose(1,:) = Pose(1,:);
    elseif i==NumSubmap
        NumOverlapScan = OverlapScan(i-1);
        Pre_DivideNumber = DividePlan(i-1);
        SubmapScan{i} = Scan(Pre_DivideNumber-NumOverlapScan:DivideNumber);
        SubmapPose{i} = Pose(Pre_DivideNumber-NumOverlapScan:DivideNumber,:);
        CoordinatePose(NumSubmap,:) = Pose(Pre_DivideNumber,:);
    else
        NumOverlapScan = OverlapScan(i-1);
        Pre_DivideNumber = DividePlan(i-1);
        SubmapScan{i} = Scan(Pre_DivideNumber-NumOverlapScan:DivideNumber-1);
        SubmapPose{i} = Pose(Pre_DivideNumber-NumOverlapScan:DivideNumber-1,:);
        CoordinatePose(i,:) = Pose(Pre_DivideNumber,:);
    end

end    


end
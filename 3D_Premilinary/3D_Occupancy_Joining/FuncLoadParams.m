function Param = FuncLoadParams()
    Param.FileDict = './Results/Voxgraph';
    Param.MaxRange = 40;
    Param.MaxIter = 1000;
    Param.MinPoseDelta = 0.00001;
    Param.LocalScale = 1; % Resolution of submaps
    Param.GlobalScale = 1; % Resolution of global map

    Param.LocalSmoothWeight = 0.00001;
    Param.TruncatedDistance = 3;
    Param.MinRange = Param.LocalScale;
        
    Param.ValOddHit = 0.847297860387203;
    Param.ValOddFree = -0.405465108108164;
    
    % Submaps division plan
    Param.DividePlan = [275,405,520,588,679,731,781,852,943,1020,1118,1204,1330,1383,1511,1720,1820,1910,2030,2290,2410,2605,2870];
    Param.NumSubmap = length(Param.DividePlan)+1;
    % How many overlapped scans between adjecent submaps,
    Param.OverlapScan = 0*ones(1,Param.NumSubmap-1);

    Param.LambdaO = 0; % Weight of odometry term (relative pose between 2 submaps)
    Param.InfMatO = [1,1,2500,1,1,2500];
   
    Param.Visualization = false;
    Param.Evaluation = false;
end
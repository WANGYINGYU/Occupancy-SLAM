function Param = FuncLoadParams()
    Param.FileDict = './Results/Voxgraph';
    Param.MaxRange = 40;
    Param.MaxIter = 1000;
    Param.MinPoseDelta = 0.00001;
    Param.LocalScale = 0.5;
    Param.LocalSmoothWeight = 0.001;
    Param.TruncatedDistance = 3;
    Param.MinRange = Param.LocalScale;
        
    Param.ValOddHit = 0.847297860387203;
    Param.ValOddFree = -0.405465108108164;
    
    Param.DividePlan = [275,405,520,588,679,731,781,852,943,1020,1118,1204,1330,1383,1511,1720,1820,1910,2030,2290,2410,2605,2870];

    Param.NumSubmap = length(Param.DividePlan)+1;

    Param.Threshold_OutObserved = 0.00;
    Param.Threshold_Select = 0.0;

    Param.PreProcess = 1;

    Param.LambdaO = 0;
    Param.InfMatO = [1,1,2500,1,1,2500];
    Param.OverlapScan = [0*ones(1,2)];

    Param.GlobalScaleX = 0.5;
    Param.GlobalScaleY = 0.5;
    Param.GlobalScaleZ = 0.5;

    Param.LocalScaleX = 0.5;
    Param.LocalScaleY = 0.5;
    Param.LocalScaleZ = 0.5;
end
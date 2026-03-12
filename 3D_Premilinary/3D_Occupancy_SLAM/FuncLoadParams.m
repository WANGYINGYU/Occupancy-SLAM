function Param = FuncLoadParams()
    Param.FileDict = './Results';
    Param.MaxRange =30;
    Param.MinRange = 0.2;
    Param.Scale = 1;
    Param.SampleDistance = Param.Scale;
    Param.TruncatedSamplingDist = Param.MaxRange;

    Param.ExtraOrigin = [1,1,4];
    Param.ExtraBoundary = [2,2,8];

    Param.MaxIter = 50;
    Param.SmoothWeight = 0.00001;
    Param.PoseThreshold = 0.005;
    Param.ObsThreshold = 0.001;

    Param.ValOddHit = 0.847297860387203;
    Param.ValOddFree = -0.405465108108164;

    Param.PreProcess = true;

    Param.UseVoxelDownsample = 0; % Set it to 1 to downsample observations
    Param.VoxelSize = 0.05;
    Param.VoxelMethod = 'centroid';
    Param.VoxelVerbose = 1;
    Param.UseAdaptiveVoxelDownsample = 0;
    Param.AdaptiveVoxelRangeEdges = [0,15,30,inf];
    Param.AdaptiveVoxelSizes = [0.05,0.10,0.15];

    Param.LambdaO = 1000;
    Param.InfMatO = [1,1,1,1,1,1];

    Param.UseRobustKernel = 0;
    Param.RobustKernel = 'huber';
    Param.RobustDelta = 1.0;
    Param.RobustMinWeight = 0.0;
    Param.RobustUseMAD = 1;

    Param.OptimizerType = 'GN';
    Param.LMDamping = 1e-3;
    Param.LinearSolverRidge = 1e-9;

    Param.MinHitCount = 1;
    Param.UseActiveVoxelOptimization = 0;
    Param.ActiveVoxelNeighborLayers = 1;

    Param.UseTruncatedRegionOptimization = 0; % Set it to 1 to open this mode to only optimizing cells within truncated area only when you want to save memory, but you must ensure the intial poses are good
    Param.TruncatedRegionLayers = 2;
    Param.TruncatedOccupancyThreshold = 0;
    Param.TruncatedObsClipExtraLayers = 2;

    Param.VisualizationPC = true;
    Param.VisualizationOGM = false;
    Param.Evaluation = false;
    Param.SavePCD = true;
end

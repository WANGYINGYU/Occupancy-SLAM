function Param = FuncLoadParams()
    Param.FileDict = './Results'; % output folder
    Param.MaxRange =30; % lidar max range used in preprocessing
    Param.MinRange = 0.2;
    Param.Scale = 1; % voxel size (m)
    Param.SampleDistance = Param.Scale;
    Param.UseMapFrameRotation = 0; % 1: estimate a fixed rotated map frame for tighter map bounds
    Param.MapFrameRotationMode = 'yaw_pca'; % current supported mode: yaw_pca
    Param.MapFrameSamplePerScan = 300; % sampled points per scan for map-frame rotation estimation

    Param.ExtraOrigin = [1,1,4];
    Param.ExtraBoundary = [2,2,8];

    Param.MaxIter = 50; % max GN/LM iterations
    Param.SmoothWeight = 0.00001; % map smoothness term weight
    Param.UseAnisotropicSmoothness = 0; % 1: use xyz anisotropic smoothness
    Param.SmoothnessLambdaXYZ = [1,1,0.5]; % [lambda_x, lambda_y, lambda_z]
    Param.PoseThreshold = 0.00001;
    Param.ObsThreshold = 0.000001;
    Param.ScanOnlyDecreaseWindow = 3; % stop if consecutive scan-only decreases are too small
    Param.ScanOnlyDecreaseTol = 1e-4; % threshold on average decrease of scan-only mean error

    Param.ValOddHit = 0.847297860387203;
    Param.ValOddFree = -0.405465108108164;

    Param.PreProcess = true; % build observations from raw scans

    Param.UseVoxelDownsample = 0; % local voxel downsample on each scan
    Param.VoxelSize = 0.05; % local voxel size (m)
    Param.VoxelMethod = 'centroid';
    Param.VoxelVerbose = 1;
    Param.UseAdaptiveVoxelDownsample = 0; % range-adaptive voxel downsample
    Param.AdaptiveVoxelRangeEdges = [0,15,30,inf];
    Param.AdaptiveVoxelSizes = [0.05,0.10,0.15];
    Param.UseGlobalVoxelDensityFilter = 0; % world-frame coarse voxel density filter
    Param.UseCoarseOccupancyStabilityFilter = 0; % 1: occupancy-value based dynamic-object filtering
    Param.ReprocessObsEachIter = 0; % 1: rerun filtering + observation generation each iteration
    Param.CoarseOccVoxelSize = 1.0; % coarse occupancy voxel size (m)
    Param.CoarseOccFreeProbThreshold = 0.20; % drop hits in voxels with strong free evidence

    Param.LambdaO = 1000; % odometry term weight
    Param.OdomSigma = [1,1,1,1,1,1]; % [sx, sy, sz, sroll, spitch, syaw]

    Param.UseRobustKernel = 0;
    Param.RobustKernel = 'huber';
    Param.RobustDelta = 1.0;
    Param.RobustMinWeight = 0.0;
    Param.RobustUseMAD = 1;

    Param.OptimizerType = 'GN'; % 'GN' or 'LM'
    Param.LMDamping = 1e-3; % LM diagonal damping
    Param.UsePCGSolver = 0; % 1: enable pcg+ichol solver (with warm-start)
    Param.PCGTol = 1e-3; % pcg relative residual tolerance
    Param.PCGMaxIter = 80; % pcg max iterations
    Param.PCGAcceptTolFactor = 1.10; % accept near-convergence when relres <= factor * tol
    Param.LinearSolverRidge = 1e-9; % fallback ridge when solve is unstable

    Param.MinHitCount = 1; % lower bound in Md./N normalization
    Param.UseActiveVoxelOptimization = 0; % optimize only active voxel subset
    Param.ActiveVoxelNeighborLayers = 1; % active subset expansion layers

    Param.UseTruncatedRegionOptimization = 0; % 1: optimize only truncated region
    Param.TruncatedRegionLayers = 2; % dilation layers around occupied cells
    Param.TruncatedOccupancyThreshold = 0; % occupied threshold to define truncation seed
    Param.TruncatedObsClipExtraLayers = 2; % extra clip margin for observations

    Param.UseBlockHashMapInTruncatedMode = 0; % 1: use block-hash map storage in truncated mode
    Param.BlockHashBlockSize = 16; % block side length (voxels)
    Param.BlockHashExtraLayers = 1; % extra region layers for block extraction
    Param.BlockHashInterpPadding = 1; % interpolation padding cells
    Param.BlockHashInitUseAABBCorners = 1; % faster bound init using 8 AABB corners

    Param.VisualizationPC = true; % show/save reconstructed point cloud
    Param.VisualizationOGM = false; % show initialized OGM occupancy
    Param.Evaluation = false;
    Param.SavePCD = true;
end

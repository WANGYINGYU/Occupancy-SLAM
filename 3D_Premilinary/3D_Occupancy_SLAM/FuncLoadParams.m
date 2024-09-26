function Param = FuncLoadParams()
    Param.FileDict = './Results/Voxgraph';
    Param.MaxRange =30;
    Param.MinRange = 0.2;
    Param.Scale = 1;
    Param.SampleDistance = Param.Scale;
    Param.MaxIter = 30;
    Param.SmoothWeight = 0.01;
    Param.PoseThreshold = 0.00001;
    Param.ObsThreshold = 0.001;


    Param.ValOddHit = 0.847297860387203;
    Param.ValOddFree = -0.405465108108164;

    Param.PreProcess = 0;
    Param.DownSampling = 0; 
    Param.SampleRate = 1;

    Param.LambdaO = 10000;
    Param.InfMatO = [1,1,1,1,1,1];

    % Param.ScaleX = 0.5;
    % Param.ScaleY = 0.5;
    % Param.ScaleZ = 1;
    Param.SelectGradientThreshold = 0;
    Param.MultiRatio = 5;
end
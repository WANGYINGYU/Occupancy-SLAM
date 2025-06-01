function Param = FuncLoadParams()
    Param.FileDict = './Results';
    Param.MaxRange =30;
    Param.MinRange = 0.2;
    Param.Scale = 1; % Resolution of the occcupancy grid map, tune to larger if you want higher speed and lower memory consumptipn
    Param.SampleDistance = Param.Scale;

    % Optimization Solver Parameters
    Param.MaxIter = 50;
    Param.SmoothWeight = 0.001; % smoothing weight, adjusting as resolution change 
    Param.PoseThreshold = 0.00001;
    Param.ObsThreshold = 0.001;

    Param.ValOddHit = 0.847297860387203;
    Param.ValOddFree = -0.405465108108164;

    Param.PreProcess = true;

    Param.LambdaO = 1000; % Weight of odometry term
    Param.InfMatO = [1,1,1,1,1,1]; % Information matrix of odometry inputs

    Param.Visualization = false; % Time consuming, turn off
    Param.Evaluation = false;
    Param.SavePCD = true;
end
function [OdomSigma, Info] = FuncResolveOdomSigma(Param, Pose)
% Resolve odometry sigma to 6-DoF form [sx sy sz sroll spitch syaw].
% Auto mode: user specifies translation sigma only; rotation sigma is
% inferred from trajectory step-length leverage.

if nargin < 2
    Pose = [];
end

Info = struct('Auto', false, 'LeverageLength', nan);

% Manual mode fallback.
if isfield(Param,'OdomSigma') && numel(Param.OdomSigma) >= 6
    OdomSigma = double(Param.OdomSigma(1:6));
else
    OdomSigma = [0.1,0.1,0.1,1*pi/180,1*pi/180,1*pi/180];
end

UseAuto = isfield(Param,'AutoSetOdomSigmaFromTranslation') && Param.AutoSetOdomSigmaFromTranslation ~= 0;
if ~UseAuto
    return;
end

% Translation sigma from user input.
SigmaT = [0.1; 0.1; 0.1];
if isfield(Param,'OdomSigmaTranslation') && ~isempty(Param.OdomSigmaTranslation)
    SigmaTRaw = double(Param.OdomSigmaTranslation(:));
    if numel(SigmaTRaw) == 1
        SigmaT(:) = SigmaTRaw(1);
    elseif numel(SigmaTRaw) >= 3
        SigmaT = SigmaTRaw(1:3);
    end
end
SigmaT = max(SigmaT, 1e-6);

% Leverage length (meters): median step of current trajectory.
Leverage = nan;
if isfield(Param,'OdomSigmaLeverageLength') && ~isempty(Param.OdomSigmaLeverageLength) && Param.OdomSigmaLeverageLength > 0
    Leverage = double(Param.OdomSigmaLeverageLength);
elseif ~isempty(Pose) && size(Pose,1) >= 2 && size(Pose,2) >= 3
    dP = diff(double(Pose(:,1:3)),1,1);
    Step = sqrt(sum(dP.^2,2));
    Step = Step(isfinite(Step) & (Step > 1e-6));
    if ~isempty(Step)
        Leverage = median(Step);
    end
end
if ~(isfinite(Leverage) && Leverage > 0)
    Leverage = 1.0;
end

% Equivalent scale: 1 rad at leverage L corresponds to L meters.
SigmaR = mean(SigmaT) / Leverage;
SigmaR = max(SigmaR, 0.2*pi/180);
SigmaR = min(SigmaR, 8.0*pi/180);

OdomSigma = [SigmaT(:)', SigmaR, SigmaR, SigmaR];
Info.Auto = true;
Info.LeverageLength = Leverage;
end

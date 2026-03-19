function [Weights, ScaleUsed] = FuncRobustWeights(Residual, Method, Delta, MinWeight, UseMAD)
% Compute robust weights for residual vector.
% Method: 'none' | 'huber' | 'cauchy'

if nargin < 2 || isempty(Method)
    Method = 'none';
end
if nargin < 3 || isempty(Delta)
    Delta = 1.5;
end
if nargin < 4 || isempty(MinWeight)
    MinWeight = 0.0;
end
if nargin < 5 || isempty(UseMAD)
    UseMAD = true;
end

r = full(double(Residual(:)));
Weights = ones(size(r));
ScaleUsed = 1.0;

if isempty(r)
    return;
end

switch lower(Method)
    case 'none'
        return;
    case {'huber','cauchy'}
        % continue
    otherwise
        error('Unknown robust kernel: %s', Method);
end

if UseMAD
    nz = abs(r) > 1e-12;
    rnz = r(nz);
    if isempty(rnz)
        return;
    end
    medr = median(rnz);
    madv = median(abs(rnz - medr));
    ScaleUsed = 1.4826 * madv;
    if ~isfinite(ScaleUsed) || ScaleUsed < 1e-12
        ScaleUsed = std(rnz);
    end
    if ~isfinite(ScaleUsed) || ScaleUsed < 1e-12
        ScaleUsed = 1.0;
    end
end

rnorm = abs(r) / ScaleUsed;

switch lower(Method)
    case 'huber'
        id = rnorm > Delta;
        Weights(id) = Delta ./ rnorm(id);
    case 'cauchy'
        Weights = 1 ./ (1 + (rnorm / Delta).^2);
end

Weights(~isfinite(Weights)) = 1.0;
Weights = max(Weights, MinWeight);

end

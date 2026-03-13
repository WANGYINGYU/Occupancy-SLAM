function FuncShowOGMMap(Map, FigId)
% Visualize initialized dense OGM occupied voxels in world coordinates.

if nargin < 2 || isempty(FigId)
    FigId = 6;
end

if ~isfield(Map,'Grid') || isempty(Map.Grid)
    fprintf('[ShowOGM] skip: Map has no dense Grid.\n');
    return;
end

Grid = Map.Grid;
if isfield(Map,'N') && ~isempty(Map.N)
    Den = max(Map.N, 1);
    GridNorm = Grid ./ Den;
else
    GridNorm = Grid;
end

GridNorm = max(min(GridNorm, 60), -60);
Prob = 1 ./ (1 + exp(-GridNorm));
OccMask = Prob > 0.55;

if ~any(OccMask(:))
    fprintf('[ShowOGM] no occupied voxels above confidence threshold.\n');
    return;
end

if isfield(Map,'Scale') && ~isempty(Map.Scale)
    Scale = Map.Scale;
else
    Scale = 1;
end

if isfield(Map,'Origin') && ~isempty(Map.Origin)
    Origin = Map.Origin(:);
else
    Origin = [0;0;0];
end

[OccRow, OccCol, OccHei] = ind2sub(size(OccMask), find(OccMask));

Xo = (double(OccCol) - 1) * Scale + Origin(1);
Yo = (double(OccRow) - 1) * Scale + Origin(2);
Zo = (double(OccHei) - 1) * Scale + Origin(3);
Co = Prob(OccMask);

MaxShow = 300000;
NumOcc = numel(Xo);
if NumOcc > MaxShow
    Id = randperm(NumOcc, MaxShow);
    Xo = Xo(Id);
    Yo = Yo(Id);
    Zo = Zo(Id);
    Co = Co(Id);
end

figure(FigId);
clf;
set(gcf, 'Color', [1 1 1]);
scatter3(Xo, Yo, Zo, 7, Co, 'filled', 'MarkerFaceAlpha', 0.88, 'MarkerEdgeAlpha', 0.05);
axis equal;
axis tight;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('Initialized OGM Occupied Voxels (%d)', numel(Xo)));
colormap(turbo);
caxis([0.55, 1.0]);
cb = colorbar;
cb.Label.String = 'Occupancy Probability';
grid on;
box on;
view(3);
drawnow;
end

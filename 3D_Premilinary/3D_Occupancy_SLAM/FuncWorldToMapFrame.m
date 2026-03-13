function PMap = FuncWorldToMapFrame(PWorld, WorldToMapR, WorldToMapT)
% Apply fixed world->map transform on 3xN or Nx3 points.

if isempty(PWorld)
    PMap = PWorld;
    return;
end

if nargin < 2 || isempty(WorldToMapR)
    WorldToMapR = eye(3);
end
if nargin < 3 || isempty(WorldToMapT)
    WorldToMapT = zeros(3,1);
end

[nRow, nCol] = size(PWorld);
if nRow == 3
    PMap = WorldToMapR * (double(PWorld) - WorldToMapT);
elseif nCol == 3
    PMap = (WorldToMapR * (double(PWorld)' - WorldToMapT))';
else
    error('PWorld must be 3xN or Nx3.');
end
end

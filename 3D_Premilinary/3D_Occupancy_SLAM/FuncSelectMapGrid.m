function SelectMap = FuncSelectMapGrid(SelectMap,Param)

Size_i = SelectMap.Size_i;
Size_j = SelectMap.Size_j;
Size_h = SelectMap.Size_h;

SelectCoords = Param.SelectCoords;
SelectId = sub2ind(size(SelectMap.Grid),SelectCoords(:,1),SelectCoords(:,2),SelectCoords(:,3));
AllId = 1:(Size_i*Size_j*Size_h);
OutId = setdiff(AllId, SelectId);

SelectMap.N(OutId) = nan;
SelectMap.Grid(OutId) = nan;

Dgrid = griddedInterpolant(SelectMap.Grid);
Ngrid = griddedInterpolant(SelectMap.N);

SelectMap.DgridG = Dgrid;
SelectMap.NgridG = Ngrid;

[Gdu,Gdv,Gdz] = gradient(SelectMap.Grid);
Gdugrid = griddedInterpolant(Gdu);
Gdvgrid = griddedInterpolant(Gdv);
Gdzgrid = griddedInterpolant(Gdz);

SelectMap.DgridGu = Gdugrid;
SelectMap.DgridGv = Gdvgrid;
SelectMap.DgridGz = Gdzgrid;


end
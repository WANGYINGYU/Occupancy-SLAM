function Map = FuncMapGrid(Map)

Dgrid = griddedInterpolant(Map.Grid);
Ngrid = griddedInterpolant(Map.N);

Map.DgridG = Dgrid;
Map.NgridG = Ngrid;

[Gdu,Gdv,Gdz] = gradient(Map.Grid);
Gdugrid = griddedInterpolant(Gdu);
Gdvgrid = griddedInterpolant(Gdv);
Gdzgrid = griddedInterpolant(Gdz);

Map.DgridGu = Gdugrid;
Map.DgridGv = Gdvgrid;
Map.DgridGz = Gdzgrid;





end
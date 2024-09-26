function FuncShow3DOccupancyMap(OccupancyMap)

[HitRow, HitCol, HitHeight] = ind2sub(size(OccupancyMap), find(OccupancyMap > 0));

figure;
scatter3(HitRow, HitCol, HitHeight, 'filled');
xlabel('Row');
ylabel('Column');
zlabel('Height');
title('3D Occupancy Map');

end 
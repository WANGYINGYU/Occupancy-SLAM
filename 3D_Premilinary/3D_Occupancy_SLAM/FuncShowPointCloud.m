function FuncShowPointCloud(Pose,Scan,Iter,Param)

numScans = numel(Pose);
ptsCell   = cell(numScans,1);
parfor i = 1:length(Pose)
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    xyz = single(Scan{i});
    Si = Ri*xyz+Posei(1:3)';
    ptsCell{i}   = Si.';
end
allPts = vertcat(ptsCell{:});
ptCloud = pointCloud(allPts);

if Param.SavePCD 
    filename = fullfile(Param.FileDict, sprintf('%03d.pcd', Iter) );
    pcwrite(ptCloud, filename, 'Encoding', 'binary');
end

if Param.Visualization 
    figure(Iter)
    pcshow(ptCloud);
    title(sprintf('Global point cloud (iter %d)', Iter));
end

end
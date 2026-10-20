% Independent MATLAB validation against ORIGINAL MAT GT, not converted GT.
base = '/media/yingyuwang/Data/experiments/sdf_ba/codex_runs';
converted = fullfile(base,'occupancy_matlab_conversion_20261020_01','converted');
results = fullfile(base,'occupancy_matlab_all_20261020_01');
scenes = jsondecode(fileread(fullfile(converted,'inventory.json')));
rows = {}; max_difference = 0;
for s = 1:numel(scenes)
    scene = scenes(s).scene;
    count = scenes(s).frames;
    original = load(fullfile('/home/yingyuwang/Downloads',[scene '_Data'],sprintf('Pose_GT_Simu%d.mat',count)),'Pose_GT');
    for c = 1:numel(scenes(s).cases)
        item = scenes(s).cases(c);
        folder = fullfile(results,scene,item.name);
        gt = original.Pose_GT(item.scan_start+1:end,:);
        ini = readmatrix(fullfile(folder,'optimized','initial','poses.txt'));
        fin = readmatrix(fullfile(folder,'optimized','joint','poses.txt'));
        assert(isequal(size(ini),size(gt)) && isequal(size(fin),size(gt)));
        expected = jsondecode(fileread(fullfile(folder,'evaluation','metrics.json')));
        phi = atan2(sin(gt(1,3)-ini(1,3)),cos(gt(1,3)-ini(1,3)));
        R = [cos(phi),-sin(phi);sin(phi),cos(phi)];
        translation = gt(1,1:2)' - R*ini(1,1:2)';
        for method = {'initial','joint'}
            key = method{1};
            if strcmp(key,'initial'), est=ini; else, est=fin; end
            xy=(R*est(:,1:2)' + translation)';
            err = xy(2:end,:) - gt(2:end,1:2);
            heading = atan2(sin(est(2:end,3)+phi-gt(2:end,3)),cos(est(2:end,3)+phi-gt(2:end,3)))*180/pi;
            position_rmse=sqrt(sum(err(:).^2)/size(err,1));
            coordinate_rmse=sqrt(mean(err(:).^2));
            heading_rmse=sqrt(mean(heading.^2));
            % Independent 2D rigid fit using atan2 of cross/dot sums, no SVD.
            x=est(:,1:2)-mean(est(:,1:2)); y=gt(:,1:2)-mean(gt(:,1:2));
            fit_phi=atan2(sum(x(:,1).*y(:,2)-x(:,2).*y(:,1)),sum(x(:).*y(:)));
            fit_R=[cos(fit_phi),-sin(fit_phi);sin(fit_phi),cos(fit_phi)];
            xy=(fit_R*x')'+mean(gt(:,1:2));
            fit_err=xy(2:end,:)-gt(2:end,1:2);
            fit_rmse=sqrt(sum(fit_err(:).^2)/size(fit_err,1));
            ref=expected.methods.(key);
            difference=max(abs([position_rmse-ref.translation_m.rmse,coordinate_rmse-ref.coordinate_rmse_m,heading_rmse-ref.heading_deg.rmse,fit_rmse-ref.secondary_independently_fitted_se2.translation_m.rmse]));
            assert(difference < 1e-10);
            max_difference=max(max_difference,difference);
            rows{end+1}=struct('scene',scene,'case',item.name,'method',key,'position_rmse_m',position_rmse,'paper_coordinate_rmse_m',coordinate_rmse,'heading_rmse_deg',heading_rmse,'fitted_position_rmse_m',fit_rmse,'difference_from_python',difference);
        end
    end
end
rows=[rows{:}];
out=getenv('SDF_BA_RUN_ROOT');
fid=fopen(fullfile(out,'matlab_evaluation.json'),'w');
fwrite(fid,jsonencode(struct('original_mat_gt',true,'rows',rows,'max_difference',max_difference,'matlab_version',version)),'char');fclose(fid);
writetable(struct2table(rows),fullfile(out,'matlab_evaluation.csv'));
fprintf('Independent MATLAB GT evaluation: %d rows, max difference %.17g\n',numel(rows),max_difference);

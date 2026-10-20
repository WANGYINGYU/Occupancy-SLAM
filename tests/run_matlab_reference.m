% Run the supplied MATLAB numerical functions with explicit noisy inputs.
% Originals stay untouched. The launcher supplies a copy with display/Info I/O disabled.
root=getenv('SDF_BA_RUN_ROOT'); addpath(fullfile(root,'matlab_source'));
scene=getenv('SIM_SCENE'); case_name=getenv('SIM_CASE');
converted_root=getenv('SIM_CONVERTED_ROOT');assert(~isempty(converted_root),'SIM_CONVERTED_ROOT must be supplied');
converted=fullfile(converted_root,scene);
receipt=jsondecode(fileread(fullfile(converted,'conversion.json')));
item=jsondecode(fileread(fullfile(converted,case_name,'input_manifest.json')));
d=load(receipt.scan_file,'Scan'); Scan=d.Scan(item.scan_start+1:end); clear d;
Pose=readmatrix(fullfile(converted,case_name,'Pose.txt'));
g=load(receipt.ground_truth_file,'Pose_GT');Pose_GT=g.Pose_GT(item.scan_start+1:end,:);clear g;
assert(size(Pose,1)==numel(Scan));
param=loadParam(); param.FILE_DIRECTORY=root;
requested=str2double(getenv('SIM_MAX_ITER'));if isfinite(requested),param.MAX_ITER=requested;end
% Output directory and external initialization are the only experiment-level overrides.
save(fullfile(root,'parameters.mat'),'param');
fid=fopen(fullfile(root,'parameters.json'),'w');fwrite(fid,jsonencode(param));fclose(fid);
writematrix(Pose,fullfile(root,'input_poses.txt'),'Delimiter','space');
writematrix(Pose_GT,fullfile(root,'ground_truth.txt'),'Delimiter','space');
% Exact supplied samples, without rebuilding free-space observations from endpoints.
fid=fopen(fullfile(root,'samples.bin'),'w','ieee-le');fwrite(fid,uint64(numel(Scan)),'uint64');
for k=1:numel(Scan)
 fwrite(fid,uint64(size(Scan{k}.xy,1)),'uint64');
 fwrite(fid,Scan{k}.xy','double');fwrite(fid,Scan{k}.Odd(:),'double');
end
fclose(fid);
Odom=zeros(size(Pose,1)-1,3); LowScan=FuncLowSampleScan(Scan,param);
Map=FuncCreateGridMap(round(param.Size_i/param.Rate),round(param.Size_j/param.Rate),param.Scale*param.Rate,param);
Map=FuncInitialiseGridMap(Map,Pose,LowScan); HH2=FuncMapConst(Map);
Map=FuncSmoothN2(Map,10,HH2);Map=FuncMapGrid(Map,param);
save(fullfile(root,'initial_state.mat'),'Map','Pose','-v7.3');
[Pose,Reason,Info,index]=FuncLeastSquares(Map,Pose,Pose_GT,Scan,LowScan,Odom,HH2,param);
writematrix(Pose,fullfile(root,'final_poses.txt'),'Delimiter','space');
fid=fopen(fullfile(root,'completion.json'),'w');fwrite(fid,jsonencode(struct('completed',true,'reason',Reason,'scene',scene,'case',case_name,'gt_used_as_initial',false)));fclose(fid);

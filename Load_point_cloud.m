function pc = Load_point_cloud(file_path, frame)
% Here, data contains 4*num values, where the first 3 values correspond to
% x,y and z, and the last value is the reflectance information. All scans
% are stored row-aligned, meaning that the first 4 values correspond to the
% first measurement.


% load point cloud (taken from demo run_demoVelodyne)
fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',file_path,frame-1),'rb');
velo = fread(fid,[4 inf],'single')';
velo = velo(1:20:end,:); % remove every 5th point for display speed
fclose(fid);

pc = pointCloud(velo(:,1:3));
  
  
end


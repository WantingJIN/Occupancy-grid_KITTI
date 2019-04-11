% The main function for occupancy grid map
% Author: Wanting JIN and Xirui LIU
%Time: 14/01/2018
clc;
clear all;
close all;

%%add the data and develop kit into path
addpath(genpath('./devkit'));
addpath(genpath('./data'))

%%Initialization the path
file_path='data/2011_09_26_drive_0046_sync/';
calib_path = 'data/2011_09_26_calib/calib_imu_to_velo.txt';
image_path = 'data/2011_09_26_drive_0046_sync/image_02/data/';

oxts = loadOxtsliteData(file_path);
pose = convertOxtsToPose(oxts);
tracklets = readTracklets([file_path '/tracklet_labels.xml']);
%calibrition matrix from IMU to lidar
T_IMU2velo = loadCalibrationRigid(calib_path);

%% read images
img_path_list = dir(strcat(image_path,'*.png'));
img_num = length(img_path_list);
Img=cell(1,img_num);
if img_num > 0
    for j = 1:img_num
        image_name = img_path_list(j).name;
        image = imread(strcat(image_path,image_name));
        Img{j}=image;
    end
end


%% Build occupancy grid
% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 3;

% 2. Indicate where you will put the origin in pixels
param.origin = [100*param.resol,100*param.resol]';

% 3. Log-odd parameters
param.lo_occ = 5;
param.lo_free = 0.5;
param.lo_max = 100;
param.lo_min = -100;
map_size = [200*param.resol, 300*param.resol];  % define a matrix of 200*300 to store the occupancy grid map
myMap = zeros(map_size);
for i=1:size(pose,2)
    %display camera images
    subplot(4,2,7:8)
    imshow(Img{i});
    % load point cloud
    pc = Load_point_cloud(file_path,i);
    subplot(4,2,5:6)
    pcshow(pc);
    hold on
    set(gca, 'View', [0, 90])
    xlim([-60,60])
    ylim([-60,60])
    title('Instantaneous Point Cloud')
    l=4;b=1.5; h = 1.5;
     plot3([-l/2,l/2,l/2,-l/2,-l/2],[-b/2,-b/2,b/2,b/2,-b/2],[0,0,0,0,0], ...
        'b','LineWidth',1);
    xlabel('X[meters]')
    ylabel('Y[meters]')
    hold off
    pause(0.1);

    
    % show occupancy grid 
    tic
    fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',file_path,i-1),'rb');
    velo = fread(fid,[4 inf],'single')';
    velo = velo(1:20:end,:); % take only 1/20 of the total data
    fclose(fid);
    %pose
    scan = velo(:,1:3);   
    lidar_pose = T_IMU2velo*pose{i};
    myMap=occGridMapping(myMap,scan,lidar_pose,param,T_IMU2velo);
    %   figure
    subplot(4,2,1:4)
    imagesc(myMap);
    colormap('gray');
    title('Occupancy Map of Garage')
    hold on
    set(gca,'YDir','normal')
    draw_tracklets(i,tracklets,pose{i},param.origin,param.resol,T_IMU2velo)
    hold on
    drawCarTra(i,pose,param.origin,param.resol,T_IMU2velo)
    hold on
    pbaspect([1,1,1])
    pause(0.1);
    toc
    F(i) = getframe(gcf);
end

%% Create video

occ_grid_video = VideoWriter('occupancy_grid.avi');
occ_grid_video.FrameRate = 9.74;
% open the video writer
open(occ_grid_video);
% write the frames to the video
for i=1:length(F)
    writeVideo(occ_grid_video, F(i));
end
% close the writer object
close(occ_grid_video);

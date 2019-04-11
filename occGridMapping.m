% FUNCTION 
% 

function myMap=occGridMapping(myMap,local_occs,pose_robot,param,T_IMU2velo)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 

% % the number of grids for 1 meter.
 resol = param.resol;
% % the initial map size in pixels
%  myMap = zeros(param.size);
% % the origin of the map in pixels
 origin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

lidarn = size(local_occs,1); % number of rays per timestamp
% N = size(ranges,2); % number of timestamp
% calib_R = [9.999976e-01 7.553071e-04 -2.035826e-03;
%     -7.854027e-04 9.998898e-01 -1.482298e-02;
%     2.024406e-03 1.482454e-02 9.998881e-01];
% calib_T = [-8.086759e-01 ;3.195559e-01; -7.997231e-01];
% T_IMU2velo = [calib_R calib_T;0 0 0 1];

pose_lidar= T_IMU2velo*pose_robot;

    
    % coordinate of robot in real world
     x = pose_lidar(1,4);
     y = pose_lidar(2,4);

    % local coordinates of occupied points in real world
%     local_occs = [ranges(:,i).*cos(scanAngles+theta), -ranges(:,i).*sin(scanAngles+theta)];

    % coordinate of robot in metric map
    grid_rob = ceil(resol * [x; y]);

    % calc coordinates of occupied and free points in metric map
    for j=1:lidarn
%         real_occ = local_occs(j,:) + [x, y]; % global coordinate of occ in real world
        real_occ = pose_lidar * [local_occs(j,:)';1];
        grid_occ = ceil(resol * real_occ(1:2,:)); % coordinate of occ in metric map

        % coordinates of free in metric map (by breshnham's algorithm)
        [freex, freey] = bresenham(grid_rob(1),grid_rob(2),grid_occ(1),grid_occ(2));

        % convert coordinate to offset to array
        free = sub2ind(size(myMap),freey+origin(2),freex+origin(1));
        occ = sub2ind(size(myMap), grid_occ(2)+origin(2), grid_occ(1)+origin(1));

        % update metric map
        myMap(free) = myMap(free) - lo_free;
        myMap(occ) = myMap(occ) + lo_occ;
    end
% end
myMap(myMap < lo_min) = lo_min;
myMap(myMap > lo_max) = lo_max;
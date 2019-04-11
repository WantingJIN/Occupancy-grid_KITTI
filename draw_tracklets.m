function draw_tracklets(frame, tracklets, pose,origion,resol,T_IMU2velo)
for it = 1:numel(tracklets) 
  % shortcut for tracklet dimensions
  w = tracklets{it}.w;
  h = tracklets{it}.h;
  l = tracklets{it}.l;

  % set bounding box corners
  corners(it).x = [l/2, l/2, -l/2, -l/2]; % front/back
  corners(it).y = [w/2, -w/2, -w/2, w/2]; % left/right
  corners(it).z = [0,0,0,0];
  
  % get translation and orientation
  t{it} = [tracklets{it}.poses(1,:); tracklets{it}.poses(2,:); tracklets{it}.poses(3,:)];
  rz{it} = wrapToPi(tracklets{it}.poses(6,:));
%   occlusion{it} = tracklets{it}.poses(8,:);
end
  for it = 1:numel(tracklets)
    
    % get relative tracklet frame index (starting at 0 with first appearance; 
    % xml data stores poses relative to the first frame where the tracklet appeared)
    pose_idx = frame-tracklets{it}.first_frame; % 0-based => 1-based MATLAB index

    % only draw tracklets that are visible in current frame
    if pose_idx<1 || pose_idx>(size(tracklets{it}.poses,2))
      continue;
    end

    % compute 3d object rotation in velodyne coordinates
    % VELODYNE COORDINATE SYSTEM:
    %   x -> facing forward
    %   y -> facing left
    %   z -> facing up
    R = [cos(rz{it}(pose_idx)), -sin(rz{it}(pose_idx)), 0;
         sin(rz{it}(pose_idx)),  cos(rz{it}(pose_idx)), 0;
                             0,                      0, 1];

    % rotate and translate 2D bounding box in velodyne coordinate system
    corners_2D      = R*[corners(it).x;corners(it).y;corners(it).z];
    corners_2D(1,:) = corners_2D(1,:) + t{it}(1,pose_idx);
    corners_2D(2,:) = corners_2D(2,:) + t{it}(2,pose_idx);
    corners_2D(3,:) = corners_2D(3,:) + t{it}(3,pose_idx);
    % transform from velodyne frame into global frame
    corners_2D_global = pose*T_IMU2velo*[corners_2D;ones(1,size(corners_2D,2))];
    %change the size and origion according to occupancy grid map
    corners_2D_grid(1:2,:) = corners_2D_global(1:2,:).*resol + origion; 

    
   
line([corners_2D_grid(1,1),corners_2D_grid(1,2)],[corners_2D_grid(2,1),corners_2D_grid(2,2)],'Color','green')
line([corners_2D_grid(1,1),corners_2D_grid(1,4)],[corners_2D_grid(2,1),corners_2D_grid(2,4)],'Color','green')
line([corners_2D_grid(1,3),corners_2D_grid(1,2)],[corners_2D_grid(2,3),corners_2D_grid(2,2)],'Color','green')
line([corners_2D_grid(1,3),corners_2D_grid(1,4)],[corners_2D_grid(2,3),corners_2D_grid(2,4)],'Color','green')

  end
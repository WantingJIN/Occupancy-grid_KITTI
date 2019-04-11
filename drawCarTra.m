function drawCarTra(frame,pose,origion,resol,T_IMU2velo)
% Draws the path of the vehicle till current frame and the vehicle
% Transfer the pose of the robot to grid frame
     path = zeros(frame,2);
    for i = 1:frame
        path(i,:) = pose{i}(1:2,4)'.*resol+origion';
    end

    plot(path(:,1),path(:,2),'LineWidth',1,'Color','yellow');
    hold on
    % Plot the vehicle
    l = 4;
    w = 1.5;
    h = 1.5;

    % set bounding box corners
    scale = 1;
    corners_x = scale*[l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]; % front/back
    corners_y = scale*[w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]; % left/right
    corners_z = scale*[0,0,0,0,h,h,h,h];
    
    corners = [corners_x; corners_y; corners_z];
    corners = [corners; ones(1,size(corners,2))];
    
    % transform from velodyne frame into global frame
    corners = pose{frame}*T_IMU2velo*corners;
    
    %change the size and origion according to occupancy grid map
    corners(1:2,:) = corners(1:2,:).*resol + origion;
    
line([corners(1,1),corners(1,2)],[corners(2,1),corners(2,2)],'Color','red')
line([corners(1,1),corners(1,4)],[corners(2,1),corners(2,4)],'Color','red')
line([corners(1,3),corners(1,2)],[corners(2,3),corners(2,2)],'Color','red')
line([corners(1,3),corners(1,4)],[corners(2,3),corners(2,4)],'Color','red')    
    
end


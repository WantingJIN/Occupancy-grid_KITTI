# Occupancy-grid_KITTI
The objectives of this project is to test occupancy grids mapping algorithms on the 3D Lidar data from the KITTI dataset,Remember that in occupancy grid mapping approaches, the pose of the robot must be known. You will therefore need to use the precise localization data also available in the KITTI dataset.

The data to test the code can be found in http://www.cvlibs.net/datasets/kitti/raw data.php
The KITTI development kit is available in the webset to process the raw data

Needed Data:
Lidardata(velodyne points): the coordinates of the detecting object in lidar frame V eloTobs
GPS=IMUdata(oxts): the GPS data in global frame GTIMU
Calib imu to velo: transformation matrix from IMU to VELO IMUTV elo
T racklet labels: labeled 3D location and rotation
Image: picture taken at each timestamp

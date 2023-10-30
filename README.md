# Mapora
Mapora is an open-source georeferenced point cloud generating tool.
Basic idea is putting each individual point cloud onto the pose which are matched with time in nanosecond precision.

Additionally, Mapora has a rosbag parser tool for generating point clouds with rosbags.

One of the important point for exporting tightly coupled point clouds with Mapora is having
high accurate GNSS/INS positions. To provide that, it is highly recommended to use a post-processing
software for GNSS/INS positions.

The other important issue is having precise LiDAR-IMU calibration angles. If not, the possibility of
having bad features in map is very high.


Some of the views of the point clouds can be seen below.

<p align='center'>
    <img src="images/mapora_building.png" alt="mapora_building" width="200"/>
    <img src="images/mapora_cars.png" alt="mapora_cars" width="200"/>
    <img src="images/mapora_environment.png" alt="mapora_environment" width="200"/>
    <img src="images/mapora_map.png" alt="mapora_map" width="200"/>
</p>

## Mapora Tools
Mapora has 2 different executable. One of them is working with specific sensors with post-processed GNSS/INS data
and the other works with rosbag includes PointCloud2, NavSatFix and Imu topics.

- [Mapora](docs/mapora_readme.md)
- [Mapora Rosbag](docs/mapora_rosbag_readme.md)
- [LiDAR-IMU Calibration](docs/lidar_imu_calibration.md)


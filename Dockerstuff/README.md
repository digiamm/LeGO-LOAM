# LeGO-LOAM docker

## How to use

```bash
# login
bash run.sh /path-to-data

# test if opengl is working 
glxgears

# Run LeGO-LOAM
source ~/catkin_ws/devel/setup.bash
roslaunch lego_loam run.launch
```


Open another terminal
```bash
# login into same container
bash login.sh

# Play rosbag
rosbag play *.bag --clock --topic /velodyne_points /imu/data
## or if you wanna remap topic
rosbag play *.bag --clock /os1_cloud_node/points:=/velodyne_points
rosbag play *.bag --clock /os_cloud_node/points:=/velodyne_points
```

# Save map
```
rosrun pcl_ros pointcloud_to_pcd input:=/laser_cloud_surround
```

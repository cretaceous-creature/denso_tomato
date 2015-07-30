# rgbtocloud

## Note

This needs TF from `$(param camera_root_frame)` to `camera_depth_optical_frame`.

## I/O

### publish

* `/boxsize (jsk_recognition_msgs/BoundingBoxArray)`
* `/centroid (geometry_msgs/PoseStamped)`

### subscribe

* `/camera_remote/depth_registered/points (sensor_msgs/Pointcloud2)`
* `/camera1_remote/depth_registered/points (sensor_msgs/Pointcloud2)`

### param

* `~/camera_root_frame` : root frame of camera, it must be connected to `camera_depth_optical_frame`.
* `/MyParam` : rmin, rmax, (g), (b), distmin, distmax
* `/Boxsize`
* `/Clusternum`
* `/Centroid`
* `/gvector`

### tf

* `/tomato (parent: /camera_rgb_optical_frame)`


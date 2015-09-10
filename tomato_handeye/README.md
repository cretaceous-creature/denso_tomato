# tomato_handeye

## lsd_slam

### start capture

```
$ roslaunch tomato_handeye start_capture_croping.launch 
```

### start lsd_slam

```
$ rosrun lsd_slam_core live_slam /image:=/camera_out/image_rect /camera_info:=/camera_out/camera_info
```

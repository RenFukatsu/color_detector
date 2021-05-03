# color_detector

## Environment
- Ubuntu 20.04
- ROS Noetic
- OpenCV 4
- PCL 1.10
- [theta_s_ros](https://github.com/RenFukatsu/theta_s_ros.git)

## Install and Build
```
sudo apt install ros-noetic-realsense2-camera \
                 ros-noetic-rgbd-launch \
                 ros-noetic-image-transport \
                 ros-noetic-image-transport-plugins
cd catkin_ws/src
git clone https://github.com/RenFukats/color_detector.git
cd ..
rosdep install -i --from-paths src
catkin build
```

## Topics
### Subscribed topics
- /equirectangular/image_raw (sensor_msgs/Image)
- /camera/depth_registered/points (sensor_msgs/PointCloud2)

### Published topics
- /target/angle ([color_detector_msg/TargetAngleList](https://github.com/RenFukatsu/color_detector/blob/master/color_detector_msgs/msg/TargetAngleList.msg))
- /target/position ([color_detector_msg/TargetPosition](https://github.com/RenFukatsu/color_detector/blob/master/color_detector_msgs/msg/TargetPosition.msg))
#### for debug
- /color_detector/masked/$COLOR/cloud (sensor_msgs/PointCloud2)
- /color_detector/masked/$COLOR/image_raw (sensor_msgs/Image)
- /color_detector/target/$COLOR/cloud (sensor_msgs/PointCloud2)
- /color_detector/target/$COLOR/image_raw (sensor_msgs/Image)

## How to Use
Theta S in live mode and then
```
roslaunch color_detector run_color_detector_node.launch
```

### with bagfile
```
roslaunch color_detector run_color_detector_node.launch is_bag:=true
```

## How to Debug
- terminal 1
  ```
  rviz
  ```
  1. Fixed Frame `map` -> `camera_color_optical_frame`
  1. Add topics
- terminal 2
  ```
  rosrun rqt_reconfigure rqt_reconfigure
  ```
  1. Optimize the parameters.
  1. Write parameters to `~/catkin_ws/src/color_detector/color_detector_params/config/config.yaml`

## Parameters
- `TOLERANCE` : If the distance between each point is less than `TOLERANCE`, they are in the same group.
- `MIN_CLUSTER_SIZE` : Do not publish if the size of the target cluster is less than `MIN_CLUSTER_SIZE`
- `MAX_CLUSTER_SIZE` : If the number of masked point clouds is more than `MAX_CLUSTER_SIZE`, reduce the number of points to be less than `MAX_CLUSTER_SIZE` to improve the computational complexity.
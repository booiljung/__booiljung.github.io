# Ubuntu 18.04 ROS melodic에 RealSense 설치

## Debian 저장소로부터 설치

Ubuntu 16.04의 ROS kenetic에 RealSense를 설치하려면 아래처럼 합니다.

- D200

```sh
sudo apt install ros-kinetic-librealsense ros-kinetic-realsense-camera
```

Realsense2를 설치하려면 아래와 같습니다.

- D300
- D435

```
sudo apt install ros-kinetic-librealsense2 ros-kinetic-realsense2-camera
```

## 실행

터미널에서

```
roscore
```

하고, 새 터미널에서 Realsense의 실행은

- D200

```sh
roslaunch realsense_camera r200_nodelet_default.launch # r200
```

Realsense2의 실행은

- D300
- D435

```
roslaunch realsense2_camera <launch 파일>
```

를 하는데 `<launch 파일>`의 목록은 아래와 같습니다.

```
demo_pointcloud.launch
rs_aligned_depth.launch
rs_from_file.launch
rs_t265.launch
demo_t265.launch
rs_camera.launch
rs_multiple_devices.launch
rs_d400_and_t265.launch
rs_rgbd.launch
opensource_tracking.launch
rs_d435_camera_with_model.launch
rs_rtabmap.launch
```

## 참조

- [github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros?fbclid=IwAR3ZCG4d4KQJLcIVMTxKt0QN-sQ48L9N9OxRThwwTyXgdbw4hijlrW-arlI)
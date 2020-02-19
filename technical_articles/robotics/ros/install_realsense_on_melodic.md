# Ubuntu 18.04 ROS melodic에 RealSense 설치

기록을 위해 남깁니다.

## 저장소로부터 설치하는 방법

Ubuntu 16.04의 ROS kenetic에 RealSense를 설치하려면 아래처럼 해왔습니다.

```sh
sudo apt install ros-kinetic-librealsense ros-kinetic-realsense-camera
```

입니다.

Ubuntu 18.04에 ROS melodic을 설치할 수 있습니다.

그런데

```sh
sudo apt install ros-melodic-librealsense ros-melodic-realsense-camera
```

으로 설치하려고 하면 패키지가 없다는 오류를 내고 설치 되지 않습니다. 구글링을 하면 많은 질문들이 올라오고 따라서 시도를 해보지만 해결 되지 않습니다.

ROS melodic에서는 RealSense2를 설치해야 합니다.

```sh
sudo apt install ros-melodic-librealsense2 ros-melodic-realsense2-camera
```

테스트

```
roscore
```

```
roslaunch realsense2_camera rs_camera.launch
```

## 빌드하여 설치하는 방법

ros-realsense2 소스코드는 [이곳](https://github.com/IntelRealSense/realsense-ros)에 있습니다. 빌드 방법에 대한 문서가 있지만 준비해야 할 의존성이 있습니다.

```
sudo apt install ros-melodic-ddynamic-reconfigure
```

`catkin_ws`폴더가 없으면 폴더를 만듭니다.

```
mkdir -p ~/catkin_ws/src
```

Github에서 소스코드를 `~/catkin_ws/src/`에 다운로드 합니다.

```
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd ~/catkin_ws/src/
catkin_init_workspace
cd ~/catkin_ws
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

테스트 합니다.

```
roscore
```

```
roslaunch realsense2_camera rs_camera.launch
```

## 참조

- [github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros?fbclid=IwAR3ZCG4d4KQJLcIVMTxKt0QN-sQ48L9N9OxRThwwTyXgdbw4hijlrW-arlI)
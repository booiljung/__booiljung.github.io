# Ubuntu 18.04 ROS melodic에 RealSense 설치

기록을 위해 남깁니다.

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


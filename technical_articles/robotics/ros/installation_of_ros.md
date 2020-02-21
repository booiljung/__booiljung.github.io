# Installation of ROS on Ubuntu

이 글은 우분투 18.04와 bash를 사용하는 경우에 한정합니다.

우분투에 ROS 소프트웨어 패키지 리파지토리와 키를 등록 합니다.

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

패키지 관리자를 업데이트 합니다.

```sh
sudo apt update
```

ROS melodic 또는 ROS kinetic을 설치합니다.

```sh
sudo apt install ros-melodic-desktop-full
sudo apt install ros-kinetic-desktop-full
```

ROS 의존성을 자동으로 해결 하도록 합니다.

```sh
sudo rosdep init
```

출력되는 메시지를 따라서 업데이트 합니다.

```sh
rosdep update
```

환경 변수를 등록 합니다.

```sh
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 참조

- http://wiki.ros.org/melodic/Installation/Ubuntu
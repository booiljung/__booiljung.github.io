# Turtlebot3 on ROS kinetic

2020년 2월 현재 ROS melodic에서는 Turtlebot3가 패키지 빌드 없이 가능하지 않습니다. Turtlebot3를 시뮬레이션하려면 [Ubuntu 16.04에 ROS kinetic을 설치](installation_of_ros.md)해야 합니다.

## 터틀봇3 설치

여기서,

- 리모트 PC는 개발용 PC를 말합니다. amd64 기반에 우분투를 설치하고, 개발을 하기도 합니다. 

- Turtlebot3 PC 또는 SBC는 터틀봇3에 부착되어 터틀봇을 제어하는 Single Board Computer를 말합니다.

먼저 리모트 PC에 관련 패키지를 설치 합니다.

```
sudo apt-get update
sudo apt-get upgrade

wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh

chmod 755 ./install_ros_melodic.sh

bash ./install_ros_melodic.sh

sudo apt install -y ros-melodic-joy \
	ros-melodic-teleop-twist-joy \
	ros-melodic-teleop-twist-keyboard \
	ros-melodic-laser-proc \
	ros-melodic-rgbd-launch \
	ros-melodic-depthimage-to-laserscan \
	ros-melodic-rosserial-arduino \
	ros-melodic-rosserial-python \
	ros-melodic-rosserial-server \
	ros-melodic-rosserial-client \
	ros-melodic-rosserial-msgs \
	ros-melodic-amcl \
	ros-melodic-map-server \
	ros-melodic-move-base \
	ros-melodic-urdf \
	ros-melodic-xacro \
	ros-melodic-compressed-image-transport \
	ros-melodic-rqt-image-view \
	ros-melodic-gmapping \
	ros-melodic-navigation \
	ros-melodic-interactive-markers
```

터틀봇3 패키지 소스코드를 클론하여 빌드 합니다.

```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws
catkin_make
```

SBC에 터틀봇을 설치 합니다.

[SBC 구성에 따른 설치 방법 참조](http://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-ubuntu-mate)

설치가 되었으면 각 컴퓨터에서 IP 주소를 설정합니다.

리모트 PC에서 IP주소를 설정합니다. `ROS_HOSTNAME`  리모트 PC에서 터틀봇가 연결된 NIC의 IP 주소를 설정합니다. 컴퓨터는 다수의 NIC가 부착되어 있을 수 있기 때문입니다. `ROS_MASTER_URI`는 리모트 PC의 URI를 설정합니다. 정확히 말하면 `roscore`가 사용할 URI 입니다.

```
export ROS_HOSTNAME=<리모트 PC IP address here>
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
```

다음은 터틀봇3 SBC의 네트워크를 설정합니다. `ROS_HOSTNAME`에는 터틀봇 NIC의 IP 주소를 설정합니다.  `ROS_MASTER_URI`에는 `roscore`가 구동되는 마스터 URI를 지정합니다.

```
export ROS_HOSTNAME=<Trutlebot PC IP address here>
export ROS_MASTER_URI=http://<Remote PC IP address here>:11311
```

## 시뮬레이션

설치가 되었으면 시뮬레이션을 구동해 보겠습니다.

언제나 그렇지만, 리모트 PC에 `roscore`가 구동 되어야 합니다. 첫번째 터미널에서,

```
roscore
```

를 하고, 두번째 터미널에서

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_fake turtlebot3_fake.launch
```

를 해줍니다. 그러면 RViz을 보실 수 있습니다.

![image-20200221213532041](/home/booil/.config/Typora/typora-user-images/image-20200221213532041.png)

자세한 내용은 [이곳](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation) 참조하세요.

## 참조

- [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
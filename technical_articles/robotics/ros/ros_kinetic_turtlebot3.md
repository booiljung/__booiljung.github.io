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

chmod 755 ./install_ros_kinetic.sh

bash ./install_ros_kinetic.sh

sudo apt install -y ros-kinetic-joy \
	ros-kinetic-teleop-twist-joy \
	ros-kinetic-teleop-twist-keyboard \
	ros-kinetic-laser-proc \
	ros-kinetic-rgbd-launch \
	ros-kinetic-depthimage-to-laserscan \
	ros-kinetic-rosserial-arduino \
	ros-kinetic-rosserial-python \
	ros-kinetic-rosserial-server \
	ros-kinetic-rosserial-client \
	ros-kinetic-rosserial-msgs \
	ros-kinetic-amcl \
	ros-kinetic-map-server \
	ros-kinetic-move-base \
	ros-kinetic-urdf \
	ros-kinetic-xacro \
	ros-kinetic-compressed-image-transport \
	ros-kinetic-rqt-image-view \
	ros-kinetic-gmapping \
	ros-kinetic-navigation \
	ros-kinetic-interactive-markers
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

## ros에서 시뮬레이션

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

이제 키보드로 컨트롤을 해보겠습니다. 터미널을 새로 하나 열어서,

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

를 하면 turtlebot3_teleop_keyboard  노드가 실행됩니다. 터미널에 표시된 

```
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop
```

를 보고, 터미널에서 해당 키를 누르면 터틀봇을 이동 할 수 있습니다.

다음, 새 터미널에서

```
rqt_graph
```

를 하여 노드와 토픽의 관계를 확인 할 수 있습니다.

<img src="/home/booil/.config/Typora/typora-user-images/image-20200221222915232.png" alt="image-20200221222915232" style="zoom:67%;" />

RViz의 Display에 TF를 추가하면 

![image-20200221223636853](/home/booil/.config/Typora/typora-user-images/image-20200221223636853.png)

TF를 시각적으로 볼 수 있습니다.

다음, 새 터미널을 열어서

```
roslaunch rqt_tf_tree rqt_tf_tree
```

를 하면

![image-20200221223445267](/home/booil/.config/Typora/typora-user-images/image-20200221223445267.png)

tf 구조를 볼 수 있습니다.

## Gazebo를 통한 시뮬레이션 1

Gazebo의 설치는 [이곳의 문서를 따라하면](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) 설치 할 수 있습니다.

Gazebo에서 ROS를 시뮬레이션하는데는 `gazebo_ros_pkgs`와 `turtlebot3_gazebo`를 필요로 하는데, 앞에서 이미 설치하였습니다.

실행중인 터미널이 있다면 모두 닫습니다. 그리고,  새 터미널을 열고 

```
roscore
```

를 시작 합니다. 새 창을 열어서 터틀봇모델은 `waffle` 로 하고, 빈 환경에 터틀봇을 올리겠습니다.

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Gazebo가 실행되고 성안에 검은 터틀봇이 갇혀 있습니다. 새 터미널을 열어서

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

키보드로 로봇을 제어 할 수도 있습니다.

## Gazebo를 통한 시뮬레이션 2

터틀봇3 시뮬레이션은 충돌, 포즈추정, 센서, 영상, 관성센서 등을 사용할 수 있습니다. 이 시뮬레이션을 하려면

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```

를 실행합니다. 이때 새 터미널을 열어서

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

를 실행하여 위치, 센서 등을 RViz에서 확인 할 수 있습니다.

자세한 내용은 [이곳](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation) 참조하세요.

## 참조

- [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)

- ROS 로봇 프로그래밍, 2017, 표윤석, 조한철, 정려운, 임태훈.
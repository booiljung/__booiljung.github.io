# Turtlebot3 on ROS melodic

## Installation

Installation of dependency on Remote PC (Developer PC).

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

Build Turtlebot 3.

```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws
catkin_make
```

Installation of dependency on Turtlebot3 Board (SBC).

[SBC 구성에 따른 설치 방법 참조](http://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-ubuntu-mate)

Configure IP address and URI on Remote PC.

```
export ROS_HOSTNAME=<Remote PC IP address here>
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
```

Configure IP address and URI on Trutlebot PC.

```
export ROS_HOSTNAME=<Trutlebot PC IP address here>
export ROS_MASTER_URI=http://<Remote PC IP address here>:11311
```

## Virtual Robot

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_fake turtlebot3_fake.launch
```

자세한 내용은 [이곳](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation) 참조.

## 참조

- [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
ROS 이해를 돕기 위한

# Turtlebot3 에 대한 이해

2020년 3월 4일 코로나-19가 기승을 부리고 있다.

## 라이센스

아파치 라이센스다.

## 매뉴얼

소스코드를 보기 전에 매뉴얼을 보자. 사용 방법을 알아야 소스코드를 이해 할 수 있다.

### [Overview](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

Turtlebot3는

- Burger
- Waffle
- Waffle Pi

 이렇게 3종류가 있다.

Turtlebot3는[ROBOTIS](http://en.robotis.com/subindex/dxl_en.php)의 [DYNAMIXEL 스마트 액추에이터](http://en.robotis.com/shop_en/list.php?ca_id=2010)를 주행용으로 채용 했다. <<< 자세한 부품 모델은 Part list를 보라.

핵심 기술은

- SLAM
- Navigation,
- Manipulation

이다.

방에서 돌아다니기 위해 SLAM 알고리즘을 사용한다.

노트북, 게임패드, 안드로이드 스마트폰으로 원격 제어 할 수 있다. 

사람의 다리를 따라 다니게 할 수도 있다.

OpenMANIPULATOR와 같은 manipulator를 부착하여 사물을 조작 할 수 있는 mobile manipulator로 사용할 수 있다.

OpenMANIPULATOR는 Waffle와 Waffle Pi와 호환되는 장점이 있다.

[소개 영상](https://www.youtube.com/watch?time_continue=3&v=9OC3J53RUsk&feature=emb_logo)

### [Features](http://emanual.robotis.com/docs/en/platform/turtlebot3/features/#worlds-most-popular-ros-platform)

- [ROBOTIS 360 Laser Distance Sensor LDS-01 (LIDAR)](http://www.robotis.us/360-laser-distance-sensor-lds-01-lidar/)를 채용 했다.

- [DYNAMIXEL XM 스마트 액추에이터](http://en.robotis.com/shop_en/list.php?ca_id=202020)를 주행용으로 채용 했다. <<< DYNAMIXEL XM시리즈는 로봇 세트보다 비싼데 확인 필요.

- 오픈소스 SBC 인 OpenCR1.0을 채용 했다.
  - OpenCR1.0은 IMU를 내장하고 있다.
  - 3.3V, 5V, 12V 파워를 공급해야 한다.
- Burger는 향상된 360 LiDAR와 9축 IMU를 사용한다.
- Waffle은 추가적으로 Intel RealSense R200을 장착 할 수 있다.
- Waffle Pi는 Rapsberry Pi용 카메라를 사용 한다.
- 모든 것이 오픈 소스다.

### [Specifications](http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/)

### [Part list](http://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/)

### [Basic Operation](http://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/)

Topic monitor는 rqt를 사용합니다.

- `/battery_state`  토픽은 배터리의 전압과 잔량 같은 메시지를 나타 낸다.
- `/diagnostics` 토픽은 MPU9250, DYNAMIXEL-X, HLS-LFCD-LDS, 배터리, OpenCR 등 부품의 연결 상태를 나타 낸다.
- `/odm` 토픽은 odometry를 나타낸다.
- `/sensor_state` 토픽 배터리와 토크의 인코더 값을 나타낸다.
- `/scan` 토픽은 최대각, 최소각, 최대 거리, 최소 거리 같은 LDS 데이터를 나타낸다.

### [Teleoperation](http://emanual.robotis.com/docs/en/platform/turtlebot3/teleoperation/)

PS3, XBOX 360, ROBOTIS RC100 등을 사용합니다.

키보드는  `turtlebot3_teleop_key` 노드를 원격 컴퓨터에 런치 한다.

```
  w
a s d		s 대신 space 사용 가능.
  x
```

를 사용합니다. 

ROBOTIS RC100은 [BT410 모듈](http://emanual.robotis.com/docs/en/parts/communication/bt-410/)을 사용하여 블루투스로 연결됩니다.

- 터틀봇이 Slave, RC100이 Master 입니다.

- `turtlebot_core` 노드가 `/cmd_vel` 토픽을 생성하므로 별도의 노드를 생성하는 실행을 하지 않아도 됩니다.

안드로이드 앱에서 토픽 이름을 변경해야 합니다.

립모션도 지원 됩니다.

### [Basic Examples](http://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/)

#### Move using interactive Markers

```
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

```
$ roslaunch turtlebot3_example interactive_markers.launch
```

```
$ rosrun rviz rviz -d `rospack find turtlebot3_example`/rviz/turtlebot3_interactive.rviz
```

#### Obstacle Detection

```
roslaunch turtlebot3_example turtlebot3_obstacle.launch
```

#### Point Operation

```
$ roslaunch turtlebot3_example turtlebot3_pointop_key.launch
```

#### Patrol

```
$ rosrun turtlebot3_example turtlebot3_server
```

```
$ roslaunch turtlebot3_example turtlebot3_client.launch
```

### [Addtional Sensors](http://emanual.robotis.com/docs/en/platform/turtlebot3/additional_sensors/)

추가 센서를 붙일 수 있다.

- IR
- 초음파
- 스위치

#### Bumper

- [Touch_sensor (TS-10)](http://emanual.robotis.com/docs/en/parts/sensor/ts-10/)

```
$ roslaunch turtlebot3_example turtlebot3_bumper.launch
```

```
$ rosrun rosserial_python serial_node.py __name:=turtlebot3_core _port:=/dev/ttyACM0 _baud:=115200
```

#### IR

- [IR Sensor (IRSS-10)](http://emanual.robotis.com/docs/en/parts/sensor/irss-10/)

```
$ roslaunch turtlebot3_example turtlebot3_cliff.launch
```

```
$ rosrun rosserial_python serial_node.py __name:=turtlebot3_core _port:=/dev/ttyACM0 _baud:=115200
```

#### Ultrasonic

- [Ultrasonic sensor (HC-SR04)]()

```
$ roslaunch turtlebot3_example turtlebot3_sonar.launch
```

```
$ rosrun rosserial_python serial_node.py __name:=turtlebot3_core _port:=/dev/ttyACM0 _baud:=115200
```

#### Illumination

- [LDR sensor (Flying-Fish MH-sensor)]()

```
$ roslaunch turtlebot3_example turtlebot3_illumination.launch
```

```
$ rosrun rosserial_python serial_node.py __name:=turtlebot3_core _port:=/dev/ttyACM0 _baud:=115200
```

#### LED

- [led (led101)]()
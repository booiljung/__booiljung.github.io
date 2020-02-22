# ROS kinetic SLAM

터틀봇3 가상 SLAM 입니다.

먼저 이전에 [터틀봇3가 설치](ros_kinetic_turtlebot3.md)되어 있어야 합니다.

먼저 터미널을 열어 Gazebo를 실행 합니다.

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

다시 새 터미널을 열어 SLAM을 실행 합니다.

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

다시 새 터미널을 열어 RViz을 실행 합니다.

```
export TURTLEBOT3_MODEL=waffle
rosrun rviz rviz -d 'rospack find turtlebot3_slam'/rviz/turtlebot3_slam.rviz
```

다시 새 터미널을 열어 키보드로 터틀봇 원격 조종할 수 있게  합니다.

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

다시 새 터미널을 열어 지도를 출력 합니다.

```
rosrun map_server map_saver -f ~/map
```

그러면 지정한 경로에 맵 파일을 저장 합니다.

```
[ INFO] [1582347538.575060451]: Waiting for the map
[ INFO] [1582347538.791379139]: Received a 384 X 384 map @ 0.050 m/pix
[ INFO] [1582347538.791429740]: Writing map occupancy data to /home/사용자/map.pgm
[ INFO] [1582347538.797961412, 779.613000000]: Writing map occupancy data to /home/booil/map.yaml
[ INFO] [1582347538.798145648, 779.613000000]: Done
```

## 참조

- [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)

- ROS 로봇 프로그래밍, 2017, 표윤석, 조한철, 정려운, 임태훈.
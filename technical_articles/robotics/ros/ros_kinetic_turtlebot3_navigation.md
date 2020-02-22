# ROS kinetic navigation

터틀봇3 가상 내비게이션 입니다.

먼저 이전에 [터틀봇3가 설치](ros_kinetic_turtlebot3.md)되어 있어야  하고 [SLAM을 하여 지도 파일을 확보](ros_kinetic_turtlebot3_slam.md)해야 합니다.

먼저 터미널을 열어 Gazebo를 실행 합니다.

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

다시 새 터미널을 열어 맵파일 경로에 주의하여 내비게이션을 실행 합니다.

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/사용자/map.yaml
```

다시 새 터미널을 열어 RViz을 실행 합니다.

```
export TURTLEBOT3_MODEL=waffle
rosrun rviz rviz -d 'rospack find turtlebot3_navigation'/rviz/turtlebot3_nav.rviz
```

여기서 `2D Nav Goal`을 누르면 화살표가 표시되고 화살표로 목적지를 지정하면 로봇이 장애물을 피하며 이동하게 됩니다. 

## 참조

- [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)

- ROS 로봇 프로그래밍, 2017, 표윤석, 조한철, 정려운, 임태훈.
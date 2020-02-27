# ROS kinetic에서 MoveIt!

이 글은 "ROS 로봇 프로그래밍, 2017, 표윤석, 조한철, 정려운, 임태훈"를 하면서 빠진 내용을 보완 하는 과정에서 작성 되었습니다.

## 설치

먼저 [ROS OpenManipulator 튜토리얼](ros_kinetic_open_manipulator.md)의 설치와 튜토리얼이 준비 되어 있어야 합니다.

## RViz

RViz  튜토리얼입니다.

### Launch

[여기](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)를 참조하여 튜토리얼을 시작 합니다.

```
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
```

## [MoveIt! Setup Assistant Tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)

이 도구는 Semantic Robot Description Format (SRDF) 파일을 생성합니다. SRDF 파일에 대한 자세한 내용은 [URDF/SRCF](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html)를 참조 합니다.





## 구성

다음, 터미널에서 MoveIt! Setup Assistant를 실행합니다.

```
 roslaunch moveit_setup_assistant setup_assistant.launch
```

그러면 Start 페이지를 볼 수 있습니다.

<img src="/home/booil/.config/Typora/typora-user-images/image-20200223162556767.png" alt="image-20200223162556767" style="zoom:75%;" />

### Create New MoveIt Configuation Package

Create New MoveIt Configuation Package를 하면 다음 페이가 표시 됩니다.

<img src="moveit_on_ros_kinetic.assets/image-20200225222231955.png" alt="image-20200225222231955" style="zoom:75%;" />

"Load a URDF or COLLADA Robot Model"에는 Brows 버튼을 클릭하여 `~/catkin_ws/src/open_manipulator/open_manipulator_description/urdf/open_manipualtor.urdf.xacro`를 선택해주고, Load Files 버튼을 클릭 합니다. 그러면, 우측에 작은 모델이 표시 될 것입니다.

![image-20200226180122599](moveit_on_ros_kinetic.assets/image-20200226180122599.png)

보기에 불편하니 렌더링 창 크기를 조정하고 당겨서 보겠습니다. 마우스 좌측을 누른채 드래그하면 모델을 중심으로 카메라 회전, 우측 버튼은 카메라 전후 이동, 중앙 버튼은 카메라의 상하좌우 이동입니다.

![image-20200226180258504](moveit_on_ros_kinetic.assets/image-20200226180258504.png)

### Self-Collisions

이 표시되고, 좌측 패인에서 Self-Collisions 페이지를 선택 합니다.

![image-20200226180525064](moveit_on_ros_kinetic.assets/image-20200226180525064.png)

Sampling Density는 충돌 검사시 정밀도를 나타내며 값이 클수록 디테일하게 충돌 검사를 합니다. 컴퓨터가 허용하고 모델의 복잡도 낮다면 높은 값을 주어 정밀하게 충돌 검사를 할 수 있습니다.

그리고, Generate Collision Matrix 버튼을 클릭합니다. 그러면 다음과 같습니다.

![image-20200226180753519](moveit_on_ros_kinetic.assets/image-20200226180753519.png)

표에서 관절을 선택하면 녹색으로 표시 됩니다.

### Virtual Joints

Virtual Joins 페이지는 기저와 기준 좌표계 사이의 가상 조인트를 제공합니다.

![image-20200226180943708](moveit_on_ros_kinetic.assets/image-20200226180943708.png)

아래에서 Add Virtual Joint를 클릭 합니다.

![image-20200226181050521](moveit_on_ros_kinetic.assets/image-20200226181050521.png)











### Planning Groups

Planning Groups 페이지는 매니퓰레이터를 그룹으로 나누어 각각의 모션 플래닝을 제공해 줍니다.

![image-20200225224917439](moveit_on_ros_kinetic.assets/image-20200225224917439.png)

여기서 Add Group 버튼을 클릭하면 페이지 내용이 다음과 같이 바뀝니다.

![image-20200225225014463](moveit_on_ros_kinetic.assets/image-20200225225014463.png)

Group Name에 arm을 입력하고,

Kinematic Solver에 kdl_kinematics_plugin/KDLKinematicsPlugin을 선택하고, 

Add Joint 버튼을 클릭하여 관절 단위로 그룹을 나눕니다.

그러면 Define Planning Groups 페이지로 바뀝니다.

![image-20200225225422340](moveit_on_ros_kinetic.assets/image-20200225225422340.png)

그러면, joint1, joint2, joint3, joint4, end_effector_joint를 선택 합니다. (여러개를 선택하려면 Ctrl 키를 누른채 마우스를 클릭 합니다.)

![image-20200225225604130](moveit_on_ros_kinetic.assets/image-20200225225604130.png)

5개의 관절이 선택되면 중앙의 > 버튼을 클릭 합니다. 그러면 Selected Joints에 선택된 관절 목록이 표시 됩니다.

![image-20200225225700305](moveit_on_ros_kinetic.assets/image-20200225225700305.png)

저장을 누르면 Planning Groups 페이지로 되돌아가고 arm 관절 그룹을 확인 할 수 있습니다.

![image-20200225225832009](moveit_on_ros_kinetic.assets/image-20200225225832009.png)

### Robot Poses

좌측에서 Robot Poses 페이지를 선택하면 다음과 같습니다.

![image-20200225225929735](moveit_on_ros_kinetic.assets/image-20200225225929735.png)

이 페이지에는 로봇의 특별한 포즈를 구성하여 등록 할 수 있습니다. 아래에서 Add Pose 버튼을 클릭합니다.

그러면,

![image-20200225230206155](moveit_on_ros_kinetic.assets/image-20200225230206155.png)

Pose Name에 포즈 이름을 지정하고 모든 관절값을 0으로 그대로 두고, Save 버튼을 클릭하여 저장합니다.

![image-20200225230222361](moveit_on_ros_kinetic.assets/image-20200225230222361.png)

포즈 목록에 추가된 zero_arm을 확인 할 수 있습니다.

![image-20200225230304933](moveit_on_ros_kinetic.assets/image-20200225230304933.png)

다시 새 포즈를 하나 추가 합니다. 적당한 Pose Name과 관절값을 주고 Save를 합니다.

![image-20200225230502530](moveit_on_ros_kinetic.assets/image-20200225230502530.png)

포즈 이름을 선택하면 포즈를 확인 할 수 있습니다. 

![image-20200225230638110](moveit_on_ros_kinetic.assets/image-20200225230638110.png)

### End Effect

좌측에서 End Effect 페이지를 선택합니다.

![image-20200225230911659](moveit_on_ros_kinetic.assets/image-20200225230911659.png)

Gripper에 대한 Planning Group이 존재해야만 등록이 가능합니다.

### Passive Joints

![image-20200226002202451](moveit_on_ros_kinetic.assets/image-20200226002202451.png)

모션 플래닝에서 제외되는 관절을 지정합니다. gripper_sub를 모션 프래닝에서 제외 해보도록 하겠습니다.

![image-20200226002246626](moveit_on_ros_kinetic.assets/image-20200226002246626.png)

### Author Information

![image-20200226002426587](moveit_on_ros_kinetic.assets/image-20200226002426587.png)

패키지를 생성하는 사용자와 이메일을 입력 합니다.

### Configuration Files

![image-20200226002529596](moveit_on_ros_kinetic.assets/image-20200226002529596.png)

Brows 버튼을 클릭하여 `open_manipulator_moveit_example`폴더를 생성하고 Choose합니다.

<img src="moveit_on_ros_kinetic.assets/image-20200226002733426.png" alt="image-20200226002849215" style="zoom:50%;" /> <img src="moveit_on_ros_kinetic.assets/image-20200226002849215.png" alt="image-20200226002849215" style="zoom:60%;" />

![image-20200226003038165](moveit_on_ros_kinetic.assets/image-20200226003038165.png)

그리고, Generate Package를 합니다.





## 참조

- [MoveIt](https://moveit.ros.org/install/)
- [MoveIt! Setup Assistant Tutorial](http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html)

- ROS 로봇 프로그래밍, 2017, 표윤석, 조한철, 정려운, 임태훈.
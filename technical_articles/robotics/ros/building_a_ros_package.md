# ROS 패키지 만들기

## catkin 패키지

패키지를 catkin 패키지로 간주하려면 몇 가지 요구 사항을 충족해야 합니다.

- 패키지는 catkin 호환 `package.xml` 파일을 포함해야합니다.
  - 해당 `package.xml` 파일은 패키지에 대한 메타 정보를 제공합니다.

- 패키지에는 catkin을 사용하는 `CMakeLists.txt`가 포함되어야 합니다.
  - catkin 메타 패키지인 경우 관련 상용구 `CMakeLists.txt` 파일이 있어야 합니다.

- 각 패키지에는 자체 폴더가 있어야 합니다
       - 즉, 중첩된 패키지나 동일한 디렉토리를 공유하는 여러 패키지가 없어야 합니다.

가장 간단한 패키지는 다음과 같은 구조를 가질 수 있습니다.

```
my_package/
  CMakeLists.txt
  package.xml
```

## catkin workspace의 패키지

catkin 패키지 작업에 권장되는 방법은 catkin **workspace**를 사용하는 것이지만 catkin 패키지를 독립형으로 빌드 할 수도 있습니다. 최소한의 작업 공간은 다음과 같습니다.

```
workspace_folder/        -- workspace
  src/                   -- source space
    CMakeLists.txt       -- catkin에 의해 제공되는 최고 레벨 CMake 파일 
    package_1/
      CMakeLists.txt     -- package_1 용 CMakeLists.txt 파일
      package.xml        -- package_1 용 페키지 메니피스트 파일
    ...
    package_n/
      CMakeLists.txt     -- package_n 용 CMakeLists.txt 파일
      package.xml        -- package_n 용 페키지 메니피스트 파일
```

이 학습서를 계속하기 전에 catkin 용 작업 공간 작성 학습서에 따라 빈 catkin 작업 공간을 작성하십시오.

## catkin 패키지 만들기

이 학습서에서는 `catkin_create_pkg` 스크립트를 사용하여 새 catkin 패키지를 작성하는 방법과 작성된 후 catkin 패키지로 수행 할 수있는 작업을 보여줍니다.

먼저 catkin 용 workspace 작성 학습서에서 작성한 catkin 작업 공간의 소스 공간 디렉토리로 변경하십시오.

```sh
cd ~/catkin_ws/src
```

이제 `catkin_create_pkg` 스크립트를 사용하여 `std_msgs`, `roscpp` 및 `rospy`에 따라 `beginner_tutorials`라는 새 패키지를 작성합니다.

```sh
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

성공하면

```
Created file beginner_tutorials/package.xml
Created file beginner_tutorials/CMakeLists.txt
Created folder beginner_tutorials/include/beginner_tutorials
Created folder beginner_tutorials/src
Successfully created files in /home/<사용자>/<경로>/catkin_src/src/beginner_tutorials. Please adjust the values in package.xml.
```

가 표시될 것입니다.

그러면 `catkin_create_pkg`에 제공 한 정보로 부분적으로 채워진 `package.xml` 및 `CMakeLists.txt`가 포함 된 `beginner_tutorials` 폴더가 생성됩니다.

`catkin_create_pkg`를 사용하려면 `package_name` 및 선택적으로 해당 패키지가 종속 된 종속성 목록을 제공해야합니다.

```sh
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

## 참조

- [ROS catin tutorials](http://wiki.ros.org/catkin/Tutorials)
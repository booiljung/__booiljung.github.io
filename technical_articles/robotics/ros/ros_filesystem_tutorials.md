# ROS filesystem tutorials on Ubuntu 18.04

이 글은 Ubuntu 18.04, bash에서 ROS melodic distro를 사용합니다. 다른 운영체제나 셸, ROS distro를 사용하려면 참조의 원문을 보시기 바랍니다.

튜토리얼을 위한 패키지를 설치 합니다. Ubuntu 18.04 Bionic에는 ROS-Melodic distro가 호환 됩니다.

```sh
sudo apt-get install ros-melodic-ros-tutorials
```

ROS에서, **Package**란 ROS 코드의 소프트웨어 구성 단위입니다. 각 패키지에는 라이브러리, 실행 파일, 스크립트 또는 기타 아티팩트가 포함될 수 있습니다. **매니페스트**(package.xml)란 패키지에 대한 기술(description)입니다. 패키지 간의 종속성을 정의하고 버전, 관리자, 라이센스 등과 같은 패키지에 대한 메타 정보를 저장하는 역할을 합니다.

코드는 많은 ROS 패키지에 분산되어 있습니다. `ls` 및 `cd`와 같은 명령 줄 도구를 탐색하는 것은 매우 불편한 조작이므로ROS는 편리한 도구를 제공합니다.

##### rospack (ros + pack)

`rospack`을 사용하면 패키지에 대한 정보를 얻을 수 있습니다. 이 튜토리얼에서는 패키지에 대한 경로를 반환하는 `find` 옵션만 다룰 것입니다. 사용 방법은 다음과 같습니다.

```sh
rospack find [package_name]
```

예를 들면

```sh
rospack find roscpp
```

결과는

```sh
/opt/ros/melodic/share/roscpp
```

일 것입니다.

##### roscd (ros + cd)

`roscd`는 `rosbash` 일부부분으로 디렉토리(cd)를 패키지 또는 스택으로 직접 변경할 수 있습니다. 사용방법은 다음과 같습니다.

```sh
roscd [locationname[/subdir]]
```

에를 들어 `roscpp` 패키지 디렉토리로 변경되었는지 확인하려면 다음과 같이 합니다.

```sh
roscd roscpp
```

그러면

```sh
booil@booil-desktop:/opt/ros/melodic/share/roscpp$
```

처럼 경로가 표시될 것입니다.

ROS 패키지 경로를 확인하려면 `ROS_PACKAGE_PATH`  환경변수를 사용합니다.

```sh
echo $ROS_PACKAGE_PATH
```

`roscd`에서 얻은 ROS 경로와 일치하는 것을 확인 할 수 있습니다. `ROS_PACKAGE_PATH`에는 ROS 패키지가 콜론으로 구분 된 디렉토리 목록이 포함되어야합니다. 일반적인 `ROS_PACKAGE_PATH`는 다음과 같습니다.

```sh
/opt/ros/melodic/share/
```

다른 환경 경로와 마찬가지로 `ROS_PACKAGE_PATH`에 각 경로를 콜론 ':'으로 구분하여 추가 디렉토리를 추가 할 수 있습니다.

##### roscd (ros + cd)

`roscd`는 패키지 또는 스택의 하위 디렉토리로 이동할 수도 있습니다.

```sh
roscd roscpp/cmake
pwd
```

하면

```
/opt/ros/melodic/share/roscpp/cmake
```

가 표시 될 것입니다.

##### roscd log

`roscd log`는 ROS가 로그 파일을 저장하는 폴더로 이동합니다. 아직 ROS 프로그램을 실행하지 않은 경우 아직 존재하지 않는다는 오류가 발생합니다.

```
No active roscore
bash: cd: /home/<사용자>/.ros/log: No such file or directory
```

이전에 일부 ROS 프로그램을 실행 한 경우 다음을 시도하십시오.

```sh
roscd log
```

##### rosls (ros + ls)

rosls는 rosbash의 일부분입니다. 절대 경로가 아닌 이름으로 패키지에 직접 `ls`를 허용합니다. 사용방법은 다음과 같습니다.

```sh
rosls [locationname[/subdir]]
```

예를 들어 `roscpp_tutorials`의 내용을 보려면

```sh
rosls roscpp_tutorials
```

하면

```
cmake  launch  package.xml  srv
$
```

가 표시될 것입니다.

## 탭 완성

전체 패키지 이름을 입력하는 것은 짜증나는 일입니다. 가령 `roscpp_tutorials`는 상당히 긴 이름입니다. 다행히 일부 ROS 도구는 탭 완료를 지원합니다.

```sh
roscd roscpp_t
```

쯤을 입력하고 탭키를 누르면 bash 에서 처럼 나머지 패키지 이름이 완성 될 것입니다.

## 참조

- [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
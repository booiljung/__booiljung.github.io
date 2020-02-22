# ROS melodic에서 UVC 카메라 캘리브레이션

## 설치

먼저 [카메라 패키지](ros_melodic_uvc_camera.md)들이 설치되어 있어야 한다.

카메라 캘리브레이션 패키지를 설치 한다.

```
sudo apt install ros-melodic-camera-calibration
```

첫 터미널에서 

```
roscore
```

두번째 터미널에서 카메라를 실행한다.

```
rosrun uvc_camera uvc_camera_node
```

세번째 카메라에서 캘리브레이션 패키지를 실행 한다.

```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/image_raw camera:=/camera
```

체스 보드를 움직이면 X, Y, Size, Skew 바가 채워진다. 필요한 이미지들이 획득되면 CALIBRATE 버튼이 활성화 된다.

캘리브레이션이 완료되면 SAVE를 눌러서 캘리브레이션 정보 파일을 저장하고 압축을 해제 한다.

```
cd /tmp
tar -xvzf calibrationdata.tar.gz
mv ost.txt ost.ini
```

카메라 파라미터 파일을 생성한다.

```
rosrun camera_calibration_parsers convert ost.ini camera.yaml
```

~/.ros/camera_info 폴더에 저장하여 ROS에서 참조하게 한다.

```
mkdir ~/.ros/camera_info
mv camera.yaml ~/.ros/camera_info/
```

## 참조

- ROS 로봇 프로그래밍, 2017, 표윤석, 조한철, 정려운, 임태훈.


# OpenVSLAM

공식 홈페이지는 [openvslam.readthedocs.io](https://openvslam.readthedocs.io) 입니다.

## 특징

- Visual SLAM 이다.
- 단안, 스테레오, RGBD 카메라 지원 한다.
- 맵을 저장하고 로드 할 수 있다.
- 이전에 만들어진 맵기반에서 새이미즈로 로컬라이징을 할 수 있다.

지원 알고리즘

- ORB-SLAM
- ProSLAM
- UcoSLAM

지원 카메라 모델

- Perspective
- Fisheye
- Equirectangular (360 카메라)

지원 시스템

- ROS

## 설치

설치는 [이 안내서](https://openvslam.readthedocs.io/en/master/installation.html)를 참조 합니다. 주의할 점은 [커스터마이징 된 DBoW2](https://github.com/shinsumicco/DBoW2)를 설치해야 합니다. [dorian3d의 DBoW2](https://github.com/dorian3d/DBoW2)를 설치하면 OpenVSLAM 빌드에 실패 합니다.

###  의존성

OpenVSLAM에 필수

- eigen 3.3.0 이상
- g2o
- suitesparse
- dbow2 (modified by shinsumicco)
- yaml-cpp 0.6.0 이상
- opencv 3.3.1 이상
  - Pangolin Viewer

Pangolin Viewer에 필수

- Pangolin
- GLEW

SocketViewer 에 필수

- Node.js 6 이상
- Protobuf 3 이상
- npm 3.5.2

추가

- google-glog

## 참조

- [OpenVSLAM](https://openvslam.readthedocs.io/en/master/index.html)
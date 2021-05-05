# ORB2 SLAM

ORB SLAM2는 일안 카메라, 양안 카메라, RGB-D 카메라를 지원하며, 예제 데이터셋으로 [KITTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php), [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset), [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)를 지원한다. ROS를 지원하며 ROS가 아닌 플랫폼도 지원한다. 라이선스는 GPLv3이며, 상업용으로 사용할 경우 허가가 필요하다.

## 빌드

연구자들에 의한 저장소 [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) 는 유지 관리가 되지 않고 있다. 오래된 Eigen3로 롤백해야 하기도 하며, `usleep`이 정의되지 않았다는 오류도 표시한다. 이를 수정하기 위한 Pull Request는 묵살 되고 있다.

그래서, [ivalab/gf_orb_slam2](https://github.com/ivalab/gf_orb_slam2)를 빌드 해보기로 한다.

```sh
cd ~/catkin_ws/src
git clone https://github.com/ivalab/gf_orb_slam2.git
git clone https://github.com/ivalab/ORB_Data.git
cd gf_orb_slam2
./build_dep.sh
./build_supports.sh
```

ORB vocabulary

```
./tools/bin_vocabulary
```

with GPU

```sh
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -march=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG -march=native"
catkin build --this
```

without GPU

```sh
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -march=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG -march=native"
catkin build --this -DENABLE_CUDA_IN_OPENCV=False
```













## 참조

- [[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**)](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)

- [[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017](https://128.84.21.199/pdf/1610.06475.pdf)

- [[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012](http://doriangalvez.com/papers/GalvezTRO12.pdf)


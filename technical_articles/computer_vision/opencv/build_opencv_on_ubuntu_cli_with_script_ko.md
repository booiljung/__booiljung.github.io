[Up](./index.md)

# 우분투에서 OpenCV를 자동 빌드하는 스크립트

[외국 커뮤니티에도 질문이 많은 관계로 영어로 번역해 두었습니다](build_opencv_on_ubuntu_cli_with_script_en.md)

`cmake-gui`를 사용하여 OpenCV를 설치하는 방법은 [이 문서를](build_opencv_with_cmake.md)를 참조합니다. 하지만, 운영체제를 설치할때마다 `cmake-gui`로 빌드하고 설치하는 것은 번거롭습니다.

이 글은 우분투 CLI에서 OpenCV를 다운로드하여 설치하는 스크립트를 설명합니다. 이 스크립트를 복사한 후 변수나 빌드 디렉토리를 변경하고 `opencv_debian.sh` 같은 파일로 저장 한 후

```
bash opencv_debian.sh
```

로 실행하면 자동으로 진행 됩니다. 자신에 맞게 변경된 빌드 스크립트는 github 등에 올려두면 편리 합니다.

먼저 eigen3를 빌드하거나 설치합니다.

```
sudo apt install libeigen3-dev -y
```

전체 스크립트 내용입니다.

```
pushd .

sudo apt update
sudo apt upgrade -y
sudo apt install build-essential -y
sudo apt install git cmake cmake-gui -y

mkdir -p ~/linspace
mkdir -p ~/linspace/opencv.github
rm -rf ~/linspace/opencv.github/opencv
rm -rf ~/linspace/opencv.github/opencv_contrib
rm -rf ~/linspace/opencv.github/build.linux

cd ~/linspace/opencv.github
git clone --recursive https://github.com/opencv/opencv.git
if [ "$?" != "0" ]; then
	echo "Cannot clone opencv" 1>&2
    popd
	exit 1
fi
cd opencv
git checkout tags/4.3.0
if [ "$?" != "0" ]; then
	echo "Cannot checkout opencv" 1>&2
    popd
	exit 1
fi

cd ~/linspace/opencv.github
git clone --recursive https://github.com/opencv/opencv_contrib.git
if [ "$?" != "0" ]; then
	echo "Cannot clone opencv_contrib" 1>&2
    popd
	exit 1
fi
cd opencv_contrib
git checkout tags/4.3.0
if [ "$?" != "0" ]; then
	echo "Cannot checkout opencv_contrib" 1>&2
    popd
	exit 1
fi

cd ~/linspace/opencv.github
mkdir -p build.linux
cd build.linux

cmake ~/linspace/opencv.github/opencv \
    -D Eigen3_DIR=/usr/local/share/eigen3/cmake \
    -D BUILD_PERF_TESTS=False \
    -D BUILD_TESTS=False \
    -D BUILD_opencv_python_tests=False \
    -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=True \
    -D INSTALL_C_EXAMPLES=True \
    -D BUILD_opencv_ts=False \
    -D BUILD_JAVA=False \
    -D BUILD_PACKAGE=False \
    -D WITH_1394=False \
    -D WITH_GSTREAMER=False \
    -D WITH_LAPACK=False \
    -D WITH_VTK=False \
    -D BUILD_opencv_world=True
if [ "$?" != "0" ]; then
	echo "Cannot cmake" 1>&2
    popd
	exit 1
fi

# cmake 3.13 or later
#cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False -B ~/linspace/opencv.github/opencv.build.linux -S ~/linspace/opencv.github/opencv

make
if [ "$?" != "0" ]; then
	echo "Cannot make" 1>&2
    popd
	exit 1
fi

sudo make install

popd
```

## 스크립트 내용 설명

먼저 우분투 소프트웨어 패키지들을 업데이트 합니다.

```
sudo apt update
sudo apt upgrade -y
```

빌드 도구들을 설치 합니다.

```
sudo apt install build-essential -y
sudo apt install git cmake cmake-gui -y
```

홈의 `linspace/opencv.github` 폴더가 작업할 폴더 입니다. 우분투의 경우 반드시 EXT4 파일 시스템에서 빌드를 해야 합니다. OpenCV는 `cmake`를 사용하고 `cmake`는 링크를 사용합니다. 우분투에서 다른 종류의 파일 시스템을 사용하면  `cmake`가 링크를 사용할 수 없어 오류가 발생 합니다.

```
mkdir -p ~/linspace
mkdir -p ~/linspace/opencv.github
```

이미 폴더가 있다면 제거합니다.

```
rm -rf ~/linspace/opencv.github/opencv
rm -rf ~/linspace/opencv.github/opencv_contrib
rm -rf ~/linspace/opencv.github/build.linux
```

빌드의 원래의 폴더로 되돌아 가기 위해 현재 디렉토리를 푸시 합니다.

```
pushd .
```

OpenCV 소스 코드를 github에서 다운로드 합니다. 이 예제는 4.3.0를 빌드 할 것입니다. 다른 버전을 사용하려면 github에서 [opencv/release](https://github.com/opencv/opencv/releases)를 확인하고 버전을 변경 합니다.

```
cd ~/linspace/opencv.github
git clone --recursive https://github.com/opencv/opencv.git
if [ "$?" != "0" ]; then
	echo "Cannot clone opencv" 1>&2
    popd
	exit 1
fi
cd opencv
git checkout tags/4.3.0
if [ "$?" != "0" ]; then
	echo "Cannot checkout opencv" 1>&2
    popd
	exit 1
fi
```

OpenCV contribution을 github에서 다운로드 합니다. 이 예제도 4.3.0를 빌드 할 것입니다. 다른 버전을 사용하려면 github에서 [opencv_contrib/release](https://github.com/opencv/opencv_contrib/releases)를 확인하고 버전을 변경 합니다.

```
cd ~/linspace/opencv.github
git clone --recursive https://github.com/opencv/opencv_contrib.git
if [ "$?" != "0" ]; then
	echo "Cannot clone opencv_contrib" 1>&2
    popd
	exit 1
fi
cd opencv_contrib
git checkout tags/4.3.0
if [ "$?" != "0" ]; then
	echo "Cannot checkout opencv_contrib" 1>&2
    popd
	exit 1
fi
```

빌드를 수행할 폴더를 만들고 해당 폴더로 이동합니다. 이 예제에서 폴더 이름은 `build.linux`로 소스 코드 폴더에 만들지 않고 별도로 분리된 폴더입니다.

```
cd ~/linspace/opencv.github
mkdir -p build.linux
cd build.linux
```

`cmake`로 Configuration을 하고 Generation을 합니다. `-D`  인수로 몇가지 변수를 변경 하였습니다.

```
cmake ~/linspace/opencv.github/opencv \
    -D Eigen3_DIR=/usr/local/share/eigen3/cmake \
    -D BUILD_PERF_TESTS=False \
    -D BUILD_TESTS=False \
    -D BUILD_opencv_python_tests=False \
    -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=True \
    -D INSTALL_C_EXAMPLES=True \
    -D BUILD_opencv_ts=False \
    -D BUILD_JAVA=False \
    -D BUILD_PACKAGE=False \
    -D WITH_1394=False \
    -D WITH_GSTREAMER=False \
    -D WITH_LAPACK=False \
    -D WITH_VTK=False \
    -D BUILD_opencv_world=True
if [ "$?" != "0" ]; then
	echo "Cannot cmake" 1>&2
    popd
	exit 1
fi
```

`cmake 3.13` 또는 이후를 사용하고 있다면 `-B`로 빌드 폴더를 지정하고, `-S`로 소스 폴더를 지정할 수 있습니다. 2020년 2월 현재 우분투의 패키지 관리자는 `3.10`을 가지고 있습니다.

```
# cmake 3.13 or later
#cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False -B ~/linspace/opencv.github/opencv.build.linux -S ~/linspace/opencv.github/opencv
```

생성이 되었으면 빌드하고 설치 합니다.

```
make
if [ "$?" != "0" ]; then
	echo "Cannot make" 1>&2
    popd
	exit 1
fi

sudo make install
```

원래의 폴더로 되돌아 갑니다.

```
popd
```






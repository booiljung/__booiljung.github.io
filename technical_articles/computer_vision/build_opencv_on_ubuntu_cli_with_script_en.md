[Up](./index.md)

# Script to automatically build OpenCV on Ubuntu

[한국어, Korean](build_opencv_on_ubuntu_cli_with_script_ko.md)

See [this article](build_opencv_with_cmake.md) for how to build and install OpenCV using `cmake-gui`. However, every time you install an operating system, installing with `cmake-gui` is cumbersome.

This article describes a script for downloading, building and installing OpenCV on Ubuntu CLI. After copying this script, change the variable or build directory and save it as a file like `opencv_debian.sh` and

```
sh opencv_debian.sh
```

run it automatically. It is convenient to put the build script on github.

Here is the complete script:

```
sudo apt update
sudo apt upgrade -y
sudo apt install build-essential -y
sudo apt install git cmake cmake-gui -y
sudo apt install libeigen3-dev -y

mkdir -p ~/linspace
mkdir -p ~/linspace/opencv.github
rm -rf ~/linspace/opencv.github/opencv
rm -rf ~/linspace/opencv.github/opencv_contrib
rm -rf ~/linspace/opencv.github/opencv.build.linux

pushd

git clone --recursive https://github.com/opencv/opencv.git ~/linspace/opencv.github/opencv
cd ~/linspace/opencv.github/opencv
git checkout tags/4.1.2

git clone --recursive https://github.com/opencv/opencv_contrib.git ~/linspace/opencv.github/opencv_contrib
cd ~/linspace/opencv.github/opencv_contrib
git checkout tags/4.1.2

mkdir -p ~/linspace/opencv.github/opencv.build.linux
cd ~/linspace/opencv.github/opencv.build.linux

cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False ~/linspace/opencv.github/opencv

# cmake 3.13 or later

#cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False -B ~/linspace/opencv.github/opencv.build.linux -S ~/linspace/opencv.github/opencv

make
sudo make install

popd
```

## Detailed comment of the script

First update the Ubuntu software packages.

```
sudo apt update
sudo apt upgrade -y
```

Install the build tools.

```
sudo apt install build-essential -y
sudo apt install git cmake cmake-gui -y
```

Install the libraries that OpenCV requires.

```
sudo apt install libeigen3-dev -y
```

If you have additional libraries, install them now.

The `linspace/opencv.github` folder in your home directory is the folder to work with. For Ubuntu, you must build on the EXT4 file system. OpenCV uses `cmake` and` cmake` uses links. If you use a different kind of file system on Ubuntu, `cmake` will not use the link and you will get an error.

```
mkdir -p ~/linspace
mkdir -p ~/linspace/opencv.github
```

If there is already the same folder, remove it.

```
rm -rf ~/linspace/opencv.github/opencv
rm -rf ~/linspace/opencv.github/opencv_contrib
rm -rf ~/linspace/opencv.github/opencv.build.linux
```

Push the current directory to return to the original folder after the build.

```
pushd
```

Download the OpenCV source code from github. This example will build 4.1.2. To use a different version, check [opencv/release](https://github.com/opencv/opencv/releases) on github and change the version.

```
git clone --recursive https://github.com/opencv/opencv.git ~/linspace/opencv.github/opencv
cd ~/linspace/opencv.github/opencv
git checkout tags/4.1.2
```

Download the OpenCV contribution from github. This example will also build 4.1.2. To use a different version, check [opencv_contrib/release](https://github.com/opencv/opencv_contrib/releases) on github and change the version.

```
git clone --recursive https://github.com/opencv/opencv_contrib.git ~/linspace/opencv.github/opencv_contrib
cd ~/linspace/opencv.github/opencv_contrib
git checkout tags/4.1.2
```

Create a folder to perform the build and make it the current folder. In this example, the folder name is `opencv.build.linux`, which is a separate folder instead of being created in the source code folder.

```
mkdir -p ~/linspace/opencv.github/opencv.build.linux
cd ~/linspace/opencv.github/opencv.build.linux
```

Configure with 'cmake` and generate. Some variables have been changed with the `-D` argument.

```
cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False ~/linspace/opencv.github/opencv
```

If you are using `cmake 3.13` or later, you can specify the build folder with` -B` and the source folder with `-S`. As of February 2020, Ubuntu's package manager has `3.10`.

```
# cmake 3.13 or later
#cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False -B ~/linspace/opencv.github/opencv.build.linux -S ~/linspace/opencv.github/opencv
```

Was it created? Now build and install.

```
make
sudo make install
```

Return to the original folder.

```
popd
```






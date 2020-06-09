[Up](./index.md)

# Script to automatically build OpenCV on Ubuntu

[한국어, Korean](build_opencv_on_ubuntu_cli_with_script_ko.md)

See [this article](build_opencv_with_cmake.md) for how to build and install OpenCV using `cmake-gui`. However, every time you install an operating system, installing with `cmake-gui` is cumbersome.

You may find it convenient to copy the script from this article, save it to a file such as `opencv_debina.sh`, change the variable value or folder name according to your needs, and write it to github.

And

```
sh opencv_debian.sh
```

run it automatically. It is convenient to put the build script on github.

Install the libraries that OpenCV requires.

```
sudo apt install libeigen3-dev -y
```

If you have additional libraries, install them now.

Here is the complete script:

```
# eigen3_ubuntu.sh#first!

pushd .

sudo apt update
sudo apt upgrade -y
sudo apt install build-essential -y
sudo apt install git cmake cmake-gui -y
sudo apt install libeigen3-dev -y

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

The `linspace/opencv.github` folder in your home directory is the folder to work with. For Ubuntu, you must build on the EXT4 file system. OpenCV uses `cmake` and` cmake` uses links. If you use a different kind of file system on Ubuntu, `cmake` will not use the link and you will get an error.

```
mkdir -p ~/linspace
mkdir -p ~/linspace/opencv.github
```

If there is already the same folder, remove it.

```
rm -rf ~/linspace/opencv.github/opencv
rm -rf ~/linspace/opencv.github/opencv_contrib
rm -rf ~/linspace/opencv.github/build.linux
```

Push the current directory to return to the original folder after the build.

```
pushd
```

Download the OpenCV source code from github. This example will build 4.1.2. To use a different version, check [opencv/release](https://github.com/opencv/opencv/releases) on github and change the version.

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

Download the OpenCV contribution from github. This example will also build 4.1.2. To use a different version, check [opencv_contrib/release](https://github.com/opencv/opencv_contrib/releases) on github and change the version.

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

Create a folder to perform the build and make it the current folder. In this example, the folder name is `opencv.build.linux`, which is a separate folder instead of being created in the source code folder.

```
cd ~/linspace/opencv.github
mkdir -p build.linux
cd build.linux
```

Configure with 'cmake` and generate. Some variables have been changed with the `-D` argument.

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

If you are using `cmake 3.13` or later, you can specify the build folder with` -B` and the source folder with `-S`. As of February 2020, Ubuntu's package manager has `3.10`.

```
# cmake 3.13 or later
#cmake -D BUILD_PERF_TESTS=False -D BUILD_TESTS=False -D BUILD_opencv_python_tests=False -D OPENCV_EXTRA_MODULES_PATH=~/linspace/opencv.github/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=True -D BUILD_opencv_ts=False -D BUILD_JAVA=False -D BUILD_PACKAGE=False -D WITH_GSTREAMER=False -D WITH_LAPACK=False -D WITH_VTK=False -B ~/linspace/opencv.github/opencv.build.linux -S ~/linspace/opencv.github/opencv
```

Was it created? Now build and install.

```
make
if [ "$?" != "0" ]; then
	echo "Cannot make" 1>&2
    popd
	exit 1
fi

sudo make install
```

Return to the original folder.

```
popd
```






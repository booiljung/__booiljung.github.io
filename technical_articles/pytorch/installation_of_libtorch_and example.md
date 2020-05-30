[Up](index.md)

# Ubuntu에 C++을 위한 LibTorch를 설치하고 예제를 테스트 해보자.

2020년 5월 29일

Ubuntu에서 C++로 LibTorch를 실험해 보려고 하는데, 직접 해가며 정리를 합니다. 임베디드 환경에서 사용하기 위해 준비하는 과정입니다.

먼저 libtorch 다운로드 합니다. 먼저 [pytorch.org](https://pytorch.org/)에서 개발 환경과 언어 버전을 선택하면 다운로드 경로를 얻을 수 있습니다. 저는 아래와 같은 옵션을 주어 다운로드 하기로 하였습니다.

저는 Stable, Linux, LibTorch, C++/Java, CPU를 선택했습니다. GPU 지원 라이브러리를 받으려면 CUDA 버전을 선택 하면 됩니다. GPU 버전에 따라 문제가 발생 할 수 있으니 첫 실험은 CPU 버전으로 합니다. 동작 여부만 확인할 것이니까요.

![image-20200529222014137](installation_of_libtorch_and%20example.assets/image-20200529222014137.png)

얻은 경로를 주어 wget으로 다운로드 할 것 입니다. 저는 유저 홈 `~` 에서 다운로드 할 것입니다.

``` sh
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
```

다운로드가 시작 되었습니다.

```
--2020-05-29 22:22:59--  https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
Resolving download.pytorch.org (download.pytorch.org)... 99.86.144.18, 99.86.144.71, 99.86.144.56, ...
Connecting to download.pytorch.org (download.pytorch.org)|99.86.144.18|:443... connected.
HTTP request sent, awaiting response... 200 OK
Length: 107915014 (103M) [application/zip]
Saving to: ‘libtorch-cxx11-abi-shared-with-deps-1.5.0+cpu.zip’

libtorch-cxx11-abi-shared-with-dep 100%[===============================================================>] 102.92M  10.4MB/s    in 11s     

2020-05-29 22:23:10 (9.76 MB/s) - ‘libtorch-cxx11-abi-shared-with-deps-1.5.0+cpu.zip’ saved [107915014/107915014]
```

다운로드가 완료 되면 압축을 해제 합니다. 파일명 지정시 URL의 일부 문자가 + 문자로 디코딩 된것에 주의 합니다.

```sh
unzip libtorch-cxx11-abi-shared-with-deps-1.5.0+cpu.zip
```

압축이 해제 됩니다.

```
Archive:  libtorch-cxx11-abi-shared-with-deps-1.5.0+cpu.zip
   creating: libtorch/
   creating: libtorch/bin/
   creating: libtorch/lib/
  inflating: libtorch/lib/libfbgemm.a  
  inflating: libtorch/lib/libgmock.a  
  inflating: libtorch/lib/libprotobuf.a  
  ...
  inflating: libtorch/include/c10d/HashStore.hpp  
  inflating: libtorch/include/c10d/FileStore.hpp  
  inflating: libtorch/include/c10d/Store.hpp
```

이제 예제를 작성하는데 cmake와 빌드 도구들이 설치 되어 있어야 합니다.

```sh
sudo apt update
sudo apt upgrade
sudo apt install -y build-essential automake cmake cmake-gui git
```

프로젝트 폴더를 만듭니다.

```sh
git init torchapp
```

여러 예제들을 연습할 것이기 때문에 예제를 서브프로젝트에 만듭니다.

```
git init 001_example
cd 001_example
```

저는 VSCode로 편집을 합니다.

`~/torchapp/001_example`에 `./gitignore` 파일을 작성합니다.

```
# build folder
build

# Prerequisites
*.d

# Compiled Object files
*.slo
*.lo
*.o
*.obj

# Precompiled Headers
*.gch
*.pch

# Compiled Dynamic libraries
*.so
*.dylib
*.dll

# Fortran module files
*.mod
*.smod

# Compiled Static libraries
*.lai
*.la
*.a
*.lib

# Executables
*.exe
*.out
*.app
```

`CMakeLists.txt`를 작성합니다.

```cmake
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
set(NAME 001_example)
project(${NAME})

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(${NAME} main.cpp)
target_link_libraries(${NAME} "${TORCH_LIBRARIES}")
set_property(TARGET ${NAME} PROPERTY CXX_STANDARD 14)

# The following code block is suggested to be used on Windows.
# According to https://github.com/pytorch/pytorch/issues/25457,
# the DLLs need to be copied to avoid memory errors.
if (MSVC)
    file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
    add_custom_command(
        TARGET
        ${NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TORCH_DLLS}
        $<TARGET_FILE_DIR:${NAME}>
        )
endif (MSVC)
```

예제 코드 `main.cpp`를 작성합니다.

```c++
#include <torch/torch.h>
#include <iostream>

int main() {
	torch::Tensor tensor = torch::rand({2, 3});
	std::cout << tensor << std::endl;
}
```

3개의 파일이 작성되었으면 빌드를 합니다.

```
mkdir build
cd build
```

cmake로 Configure를 할때 LibTorch 경로를 지정해 주어야 합니다.

```
cmake 
```


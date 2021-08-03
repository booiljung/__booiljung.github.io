# CMake 간단한 소개서

CMake는 [Kitware]()에서 개발하고 있으며, [cmake.org](https://cmake.org/)에서 학습하고 다운로드 할 수 있습니다. 전통적인 C/C++   빌드 도구는 make입니다. make는 Makefile을 작성해 주어야 하는데, Makefile 작성시 의존관계를 개발자가 직접 기술해야 하며, 개발자는 정확한 의존성을 위해 소스코드를 들여다보며 많은 노동을 투입하여 작성해야 합니다. Makefile 작성시 실수로 의존관계를 빠뜨리면 변경이 업데이트 되지 않거나, 컴파일 오류나 런타임 오류가 발생하기도 합니다. 특히 런타임 오류는 그 원인을 파악하기도 어렵습니다. make 사용시 clean 구성을 자주 사용하기도 하는 이유기도 합니다. CMake는 소스파일을 파싱하여 의존관계를 파악하여 자동으로 Makefile들이 작성되도록 돕는 것이 1차 목표 입니다.

## 단계

CMake를 사용하는 것은 `cmake`에 의해 `CMakeLists.txt`를 파싱하거나 `Makefile`들을 생성하는 구성(Configure) 단계와, `make`에 의한 빌드 (Build) 단계로 나눠 볼 수 있습니다.

구성단계는 `CMakeLists.txt`를 파싱하고 실행하여 `Makefile` 이나 `Ninja`, `vsproj`등을 생성하는 단계이며, 이 파일들을 자동으로 구성하는 것이 `cmake`의 기본 목표 입니다. 이 글은  `Makefile`만을 구성하는 과정을 다룹니다.

#### 이 글의 빌드 경로

빌드를 하면 많은 중간 파일들이 생성됩니다. 이 중간 파일들이 프로젝트 소스 파일과 섞이는 것을 막기 위해,  별도의 폴더를 만들고, 빌드를 합니다. 프로젝드 폴더에 `build`폴더를 만들고, 프로젝트 경로를 지정하여 빌드하는 것이 일반적입니다.

```sh
cd project_dir
mkdir build
cmake ..
make
sudo make install
```

그런데 저는 `build`폴더가 프로젝트와 섞이는게 싫어서 별도의 경로에 빌드 합니다.

```sh
mkdir project_dir.build
cd project_dir.build
cmake project_dir
make
sudo make install
```

---

CMake는 Configure와 Generate 2단계로 구성 됩니다.

### 구성(Configure) 단계

기본적으로 make를 위해 아래처럼 명령합니다.

```
cd project_dir
mkdir build
cmake ..
```

또는

```sh
mkdir building_path
cd building_path
cmake project_path
cmake ..
```

이글의 주제는 아니지만 참고로 다른 빌드 시스템을 위한 CMake 옵션은 다음과 같습니다.

Visual Studio

```powershell
cmake ./project_path/ -G "visual Studio 15 2017 Win64"
```

Unix/Linux Ninja

```bash
cmake ./project_path/ -G Ninja
```

XCode

```bash
cmake ./project_path/ -G XCode
```

### Build 단계

`build` 단계는 다른 외부 도구를 사용합니다. `Makefile`을 생성하였으면 `make`를 실행 합니다.

```sh
make
```

### 설치 단계

##### Unix/Linux

```sh
sudo make install
```

## 변종 (Variant)

기본적으로 4 가지 변종을 지원하며, 필요시 사용자 정의 변종을 정의할 수 있습니다.

- Debug: 디버깅을 위한 빌드.
- Release: 배포를 위한 빌드.
- RelWithDebInfo: 배포를 위한 빌드이지만 디버깅 정보 포함.
- MinSizeRel: 배포를 위한 빌드인데 최소 크기로 최적화.

선택한 변종에 따라 컴파일 및 링크 옵션은 추가 합니다. 빌드 목적별로 빌드 스크립트를 각각 따로 작성하는 수고들 덜어 줍니다.

## 스크립트

CMake는 별도로 지정하지 않는다면 프로젝트 폴더 내에 있는 `CMakeLists.txt`파일을 파싱하고 실행 합니다.

### 표기법

- `cmake` 명령과 변수이름은 대소문자를 구분하지 않습니다.
- 내장 상수는 반드시 대문자입니다.
- 파일이름은 각 운영체제 기준을 따릅니다. Windows, macos는 구분하지 않으며, Unix나 Unix Like 운영체제는 대소문자를 구분합니다.

이 예제에서는 소문자로 표시한 스크립트는 대문자나 소문자로 작성할 수 있습니다. 대문자로 표시한 스크립트는 반드시 대문자로 작성해야 합니다.

예를 들어

```cmake
project(my_project)
```

이렇게 소문자로 작성하였으면,

```cmake
PROJECT(my_project)
```

아래 `STATIC`처럼 대문자로 표시한 스크립트는 반드시 대문자만으로 기술해야 합니다.

```cmake
add_library(my_lib STATIC ${source})
```

### 주석

```cmake
# 주석은 #문자로 지정합니다.
```

### 버전 제약: cmake_minimum_required()

```cmake
cmake_minimum_required(VERSION 3.10)
```

위 예제의 스크립트는 3.10 이상 버전의 cmake를 필요로 합니다.

### 프로젝트 이름: project()

프로젝트의 이름을 지정합니다:

```cmake
project(my_project)
```

프로젝트에서 언어와 버전을 지정할 수도 있습니다:

```cmake
prject(my_project LANGUAGE CXX VERSION 1.2.3)
```

## 변수

### 변수 만들기: set()

`set` 명령으로 변수이름과 값을 지정합니다:

```cmake
set(my_variable value)
```

### 변수의 값 참조: ${variable}

`${}`안에 변수이름을 넣으면 변수의 그 값을 참조할 수 있습니다:

```cmake
${my_variable}
```

예를 들어 파일이름 목록을 `SOURCE`라는 변수에 넣고 다른 명령에서 참조 하겠습니다:

```cmake
set (SOURCE file1.c file2.cpp ...)
```

이 `SOURCE`  변수에 있는 파일 이름들을 `add_library()`라는 명령에 전달 합니다:

```cmake
set (SOURCE file1.c file2.cpp ...)
add_library(${PROJECT_NAME} SHARED ${SOURCE})
```

### 모든 출력 메시지 표시 옵션: CMAKE_VERBOSE_MAKEFILE

`CMAKE_VERBOSE_MAKEFILE`라는 내장 변수에 `true`를 지정하면 Makefile을 구성하는 단계에서 이 모든 출력 메시지들을 표시하도록  Makefile을 생성합니다:

```cmake
set (CMAKE_VERBOSE_MAKEFILE true/false)
```

빠른 문제 해결을 위해 이 변수의 값은 기본적으로 켜는 것이 좋습니다.

## 컴파일러 지정

컴파일러의 경로를 `CMAKE_C_COMPILER` 내장 변수에 지정합니다:

```cmake
set(CMAKE_C_COMPILER c_compiler_path)
```

### 컴파일 옵션

C/C++ 컴파일러의 옵션을 지정 합니다:

```cmake
add_compile_options(-g -Wall ... )
```

### 빌드 구성별 컴파일 옵션

`CMAKE_C_FLAGS_빌드구성`이라는 내장 변수에 C 컴파일 플래그(옵션)을 지정합니다:

```cmake
set(CMAKE_C_FLAGS_빌드구성 ...)
```

로 지정합니다.

다음은 `Release`구성을 위한 C 컴파일 옵션을 지정 합니다:

```cmake
set(CMAKE_C_FLAGS_RELEASE "-DMY_DEFINE 3 -DYOUR_DEFINE 4" ...)
```

옵션이 여러개인 경우 `" "`로 감싸 주어야 합니다.

### 컴파일 중 사용할 전처리 매크로 추가

C 컴파일러 옵션 `-D`에 해당 합니다:

```cmake
add_definitions(-Dmy_define=value -Dyour_define=value -Dhis_define=value ...)
```

### 컴파일 중 사용할 헤더 디렉토리 추가

C 컴파일러 옵션 `-I`에 해당 합니다:

```cmake
include_directories(my_include_directory your_include_directory his_include_directory ...)
```

### 링크에 사용할 라이브러리 디렉토리 추가

링커 옵션중 `-L`에 해당 합니다:

```cmake
link_directories(my_lib_dir your_lib_dir ...)
```

### 링크에 사용할 라이브러리 파일 지정

링커 옵션중 `-l`에 해당 합니다:

```cmake
link_libraries(mine yours his her ...)
```

이 명령 인자들을 `-`에서 시작하도록 하여 직접 링크 옵션을 지정할 수도 있습니다:

```cmake
link_libraries(libmine libyours libhis libher -static ...)
```

여기서 `-static`을 지정하여 공유 라이브러리를 사용하지 않도록 지정 하였습니다.

### 빌드 구성별 링크 옵션: CMAKE_EXE_LINKER_FLAGS_빌드구성

`CMAKE_EXE_LINKER_FLAGS_빌드구성`이라는 내장 변수에 구성별 링크 옵션을 지정합니다:

```cmake
set(CMAKE_EXE_LINKER_FLAGS_빌드구성 ...)
```

```cmake
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-DCONFIG_DEBUG -Wl,-whole-archive")
```

옵션이 여러개인경우 `" "`로 감싸 주어야 합니다. 

### 실행 파일 출력 경로: RUNTIME_OUTPUT_DIRECTORY

내장 변수 `RUNTIME_OUTPUT_DIRECTORY`에 빌드 완료한 실행 바이너리를 저장할 디렉토리를 지정합니다:

```cmake
set(RUNTIME_OUTPUT_DIRECTORY output/bin )
```

### 라이브러리 출력 경로: LIBRARY_OUTPUT_DIRECTORY

내장 변수 `LIBRARY_OUTPUT_DIRECTORY`에 빌드 완료한 실행 바이너리를 저장할 디렉토리를 지정합니다.

```cmake
set(LIBRARY_OUTPUT_DIRECTORY my_lib_output_path)
```

### 아카이브  출력 경로: ARCHIVE_OUTPUT_DIRECTORY

빌드 완료한 static 라이브러리를 저장할 디렉토리를 지정합니다.

```cmake
set(ARCHIVE_OUTPUT_DIRECTORY my_archive_output_path)
```

## 구성 타겟

### 실행 파일 타겟: add_executable()

```cmake
add_executable(my_executable main.cpp data.cpp network.cppp ...)
```

### 라이브러리 타겟: add_library()

```cmake
add_library(my_lib main.cpp data.cpp network.cpp ...)
```

#### 정적 라이브러리: add_library(STATIC)

```cmake
add_library(my_lib STATIC main.cpp data.cpp network.cpp ...)
```

#### 공유 라이브러리: add_library(SHARED)

```cmake
add_library(my_lib SHARED main.cpp data.cpp network.cpp ...)
```

### 빌드에 필요한 라이브러리 패키지 설치

check_include_file_cxx() 를 사용하여 include 할 header file을 찾아본 후, 없는 경우 라이브러리 설치합니다. find_library()를 사용하여 link할 library file을 검색하고, 없는 경우 라이브러리 설치 합니다. [출처](https://m.blog.naver.com/PostView.nhn?blogId=likeafree&logNo=221475154955&proxyReferer=https:%2F%2Fwww.google.com%2F)

```cmake
set (CMAKE_REQUIRED_INCLUDES include ../include ../framework)
include_directories ( ${CMAKE_REQUIRED_INCLUDES} )

include(CheckIncludeFileCXX)
check_include_file_cxx(glog/logging.h HAVE_H)
if (NOT HAVE_H)
	message(STATUS "install libgoogle-glog-dev")
	execute_process(COMMAND "sudo apt-get install libgoogle-glog-dev")
endif (NOT HAVE_H)

find_library(HAVE_LIB NAMES jsoncpp PATHS /usr/lib/)
if (NOT HAVE_LIB)
	message(STATUS "install libjsoncpp-dev")
	execute_process( COMMAND "sudo apt-get install libjsoncpp-dev")
endif (NOT HAVE_LIB)

```

### 사용자 정의 타겟

```cmake
add_custom_target(my_target ...)
```

예) 다음 구문은 ESP8266 프로젝트에서 생성한 바이너리(app.bin)를 Flashing하기 위한 `make flash` 매크로를 정의합니다. [예제 출처](https://www.tuwlab.com/ece/27260)

```cmake
add_custom_target ( flash
        COMMENT "Flashing binary"
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND python esptool.py write_flash app.bin
        DEPENDS app.out
        )
```

### 타겟간 의존성 정의

`add_executable`, `add_library`, `add_custom_target` 명령으로 정의한 타겟들간의 의존성을 정의 합니다.

```cmake
add_dependencies(my_taget your_target his_target ...)
```

`my_target`은 `your_target`과 `his_target`에 의존하게 됩니다.

예) 다음 명령은 flash(매크로)가 app.out에 의존적임을 명시합니다. 이렇게 작성하고 'make flash'를  실행하면 app.out이 최신인지 여부를 먼저 검사해서 필요시 app.out을 먼저 빌드하고 flash 매크로를 실행합니다. [예제 인용](https://www.tuwlab.com/ece/27260#)

```cmake
`ADD_DEPENDENCIES ( flash app.out )`
```

## 타겟별 빌드 설정

타겟이 다수이고 타겟별로 빌드 옵션을 지정할 경우 사용합니다.

###  타겟별 컴파일 옵션: target_compile_options()

```cmake
target_compile_options(my_target PUBLIC option1 option2 option3 ... )
```

여기서 `PUBLIC`은 전역 컴파일 옵션입니다. 이외에 `INTERFACE`, `PRIVATE`가 있습니다.

### 타겟별 전처리 정의: target_compile_definitions()

```cmake
target_compile_definitions(my_target PUBLIC macro1 macro2=value macro3=value ...)
```

컴파일러에 `-D` 옵션을 추가한것과 같습니다. `add_definition()`과 달리 `-D`로 시작 하지 않습니다.

### 타겟별 헤더파일 경로 지정: target_include_directories()

```cmake
target_include_directories(my_target PUBLIC include_dir1 include_dir2 ...)
```

### 타겟별 라이브러리 경로 지정: target_link_libraries()

```cmake
target_link_libraries(my_target PUBLIC lib_dir1 lib_dir2 ...)
```

## 파일 구성: configure_file()

변수의 값을 참조하여 템플릿 파일의 일부를 치환하여 파일을 출력 합니다.

```cmake
configure_file(template_filename output_filename)
```

템플릿 파일은 `$variable$` 또는 `@variable@`으로 변수이름을 지정합니다.

예를 들어 다음과 같은 템플릿 파일 `version.h.in`이 있습니다.

```c++
#define __VERSION__ "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.${VERSION_TWEAK}"
```

그리고 `CMakeLists.txt`에 다음 변수들이 정의되어 있습니다.

```cmake
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 45)
set(VERSION_TWEAK 32)
```

그러면 `version.h`는 아래와 같이 출력됩니다.

```c++
#define __VERSION__ "1.0.45.32"
```

## 커스텀 명령

### 커스텀 출력 파일 생성

```cmake
add_custom_command(
    OUTPUT output_file_list
    [COMMENT output_message]
    [DEPENDS dependency_list]
    [WORKING_DIRECTORY working_directory]
    COMMAND command
    [COMMAND command]
    ...
)
```

다음은 파이썬을 실행하여 아이콘 파일을 생성합니다.

```cmake
add_custom_command(
	OUTPUT app.ico
	COMMENT "Generating app.ico file"
    DEPENDS app_icon.png
    COMMAND python myicon.py app.ico app_icon.png
	)
```

### 커스텀 타겟 생성

```cmake
add_custom_command(
	TARGET target_name
	PRE_BUILD|PRE_LINK|POST_BUILD 중 선택
	[COMMENT output_message]
	[WORKING_DIRECTORY working_directory]
	COMMAND command
	[COMMAND command]
    ...
)
```

`PRE_BUILD|PRE_LINK|POST_BUILD`중 하나를 선택하는 옵션이 있습니다. 빌드전, 링크전, 빌드 후 실행할 명령을 지정 합니다.

## 서브 디렉토리: add_subdirectory()

서브 디렉토리에는 별도의 `CMakeLists.txt`파일이 존재해야 합니다.

```cmake
add_subdirectory(my_subdirectory)
```

현재 디렉토리의 하위 디렉토리가 아니라면 빌드 디렉토리를 지정해야 합니다.

```cmake
add_subdirectory(../other_directory/subdirectory ./build/other_subdirectory)
```

## 메시지: message

`message`는 구성 단계에 콘솔에 메시지나 변수를 표시합니다.

```cmake
message(type message)
```

type은 다음중 하나이며, 생략 할 수 있습니다.

- `STATUS`: 상태 메시지이며 메시지 앞에 `--`가 추가되어 표시 됩니다.
- `WARNING`: 경고 메시지이며, 구성을 계속 진행 합니다.
- `AUTHOR_WARNING`:  프로젝트 개발자용 경고 메시지이며, 구성을 계속 진행 합니다.
- `SEND_ERROR`: 오류 메시지를 출력하고, 계속 진행 하지만, `Makefile`은 생성하지 않습니다.
- `FATAL_ERROR`: 오류 메시지를 출력하고, 구성을 즉시 중단 합니다.
- `type`이 없으면 중요한 정보임을 나타내며, 구성을 계속 진행 합니다.

`make` 중에 메시지를 표시하려면, `add_custom_command()`명령으로 `Makefile`에 메시지를 추가해야 합니다.

## 설치: install

```
make install
```

을 하였을때 수행할 동작을 정의 합니다.

```cmake
install(TARGETS target_list
        RUNTIME DESTINATION bin_file_installation_path
        LIBRARY DESTINATION lib_file_installation_path
        ARCHIVE DESTINATION archive_file_installation_path
        )
```

설치 경로가 동일하다면 아래처럼 간단히 정의 할 수 있습니다.

```cmake
install(TARGETS target_list DESTINATION installation_path)
```

다음 예제는 타겟을 `/usr/local`에 설치 합니다.

```cmake
install(TARGETS target1 target2
        RUNTIME_DESTINATION /usr/local/bin
        ARCHIVE_DESTINATION /usr/local/lib
        )
```

#### 설치 기본 디렉토리: CMAKE_INSTALL_PREFIX

내장 변수 `CMAKE_INSTALL_PREFIX`에 `install()` 명령에서 사용할 기본 경로를 지정합니다. 지정하지 않으면 기본 값은 `/usr/local` 입니다.

다음은 예제는

```cmake
install(TARGETS target1 target2
        RUNTIME_DESTINATION /usr/world/bin
        ARCHIVE_DESTINATION /usr/world/lib
        )
```

다음처럼 기술 할수 있습니다.

```cmake
set(CMAKE_INSTALL_PREFIX /usr/world)
```

## Boost 단위 테스트

프로젝트 CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 3.0)
project(project_name)
SET(PROJECT_VERSION_MAJOR 0)
SET(PROJECT_VERSION_MINOR 1)
enable_testing()

find_package(
    Boost COMPONENTS
    filesystem
    program_options
    system
    ...
    REQUIRED
)

set(
    libs
    pthread
    stdc++fs
    boost_filesystem
    boost_program_options
    ...
)

add_library(
    library_name
    STATIC
    src/sample.cpp
)

#
# unit test directory
#

add_subdirectory(test)

#
# capture_stereo_save_main
#

add_executable(
    executable_name
    src/sample_main.cpp
)

target_link_libraries(
    executable_name
    ${libs}
    library_name
)
```

`test` 하위 폴더를 만들고 단위 테스트를 위한 CMakeLists.txt를 작성합니다:

```cmake
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

add_executable(test_name test1.cpp)
target_link_libraries(test_name library_name ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_definitions(-DBOOST_TEST_DYN_LINK)
add_test(NAME test1 COMMAND test_name)
```

테스트 코드에는 `#include <unit_test.hpp>`앞에 `#define BOOST_TEST_MODULE ****`을 정의합니다:

```c++
#define BOOST_TEST_MODULE test_path
#include <boost/test/unit_test.hpp>

#include "../src/path.hpp"


BOOST_AUTO_TEST_CASE(test_path)
{
    BOOST_CHECK( add( 2,2 ) == 4 );  
    BOOST_REQUIRE( add( 2,2 ) == 4 );
    if( add( 2,2 ) != 4 )
        BOOST_ERROR( "Ouch..." );
    if( add( 2,2 ) != 4 )
        BOOST_FAIL( "Ouch..." );

    if( add( 2,2 ) != 4 )
        throw "Ouch...";

    BOOST_CHECK_MESSAGE( add( 2,2 ) == 4, "add(..) result: " << add( 2,2 ) );
    BOOST_CHECK_EQUAL( add( 2,2 ), 4 );
}
```

빌드:

```sh
mkdir -p build
cd build
cmake ..
make
```

테스트:

```
make test
```

## 데비안 패키지 생성

CPack를 사용하여 데비안 패키지를 생성할 수 있습니다. [출처](https://m.blog.naver.com/PostView.nhn?blogId=likeafree&logNo=221475154955&proxyReferer=https:%2F%2Fwww.google.com%2F)

```cmake
set(VERSION "0.0.1")
set(CPACK_PACKAGE_RELEASE 2)
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME "_NAME_")
set(CPACK_PACKAGE_CONTACT "developer")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${CMAKE_CURRENT_SOURCE_DIR}/postinst)

include(CPack)
```

이렇게 하고

```sh
make package
```

를 하면 데비안 패키지가 만들어 집니다.

### 데비안 패키지에서 postinst

데비안 패키지는 패키지 설치 전/후에 스크립트를 실행할 수 있습니다. 위 예제에서는 `CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA` 명령을 사용하여 설치 후에 postinst 스크립트가 실행될 수 있도록 설정할 수 있습니다.

#### postinst 예제

- 데비안 패키지는 root 권한으로 설치되어 소유자를 user로 변경합니다.

- 실행에 필요한 라이브러리가 설치되어 있는지 확인하고 없는 경우 설치합니다.

```sh
STR_USERS=$(users)
USERNAME=${STR_USERS%% *}

if [ "$USERNAME" != "root" ]; then
    echo $USERNAME
else
    homedir=$HOME
    USERNAME=${homedir##*/}
    echo $USERNAME
fi

echo "change owner to $USERNAME"
sudo chown $(echo $USERNAME|tr -d '\r'):$(echo $USERNAME|tr -d '\r') -R /{install_dir}

CHECKLIB=$(dpkg-query -l | grep libopencv2.4-java | wc -l)
if [ "$CHECKLIB" = "0" ]; then
	echo "install libopencv2.4-java"
	sudo apt-get install libopencv2.4-java
fi
```

## CPack으로 특정 형식 패키지 만들기

특정 형식을 사용하여 패키지를 만들려면 사용할 Generator 를 선택할 수 있으며,  CMake와 비슷하게 **-G** 인수를 지정합니다. [인용](https://riptutorial.com/ko/cmake/example/16826/%EC%82%AC%EC%9A%A9%ED%95%A0-cpack-%EC%83%9D%EC%84%B1%EA%B8%B0-%EC%84%A0%ED%83%9D)

```sh
cpack -G 7Z .
```

이 명령 줄을 사용하면 7-Zip 아카이브 형식을 사용하여 현재 디렉토리에 빌드 된 프로젝트를 패키지화 합니다. 다음은 CPack 버전 3.5 기본적으로 다음 Generator를 지원합니다.

-  `7Z` 7-Zip 파일 형식 (아카이브) 
-  `IFW` Qt Installer 프레임 워크 (실행 파일) 
-  `NSIS` 널 소프트 설치 프로그램 (실행 가능) 
-  `NSIS64` 널 소프트 설치 프로그램 (64 비트, 실행 가능) 
-  `STGZ` Tar GZip 자동 압축 풀기 (압축 파일) 
-  `TBZ2` Tar BZip2 압축 (아카이브) 
-  `TGZ` Tar GZip 압축 (아카이브) 
-  `TXZ` Tar XZ 압축 (아카이브) 
-  `TZ` Tar 압축 압축 (아카이브) 
-  WiX 도구 (실행 가능한 아카이브)를 통한 `WIX` MSI 파일 형식 
-  `ZIP` ZIP 파일 형식 (아카이브) 

명시적인 Generator가 제공되지 않으면 CPack은 실제 환경에 따라 최적의 사용 가능한 전원을 결정하려고 합니다. 예를 들어,  Windows에서 자동 압축 풀기 실행 파일을 만드는 것이 좋으며 적절한 툴셋이 없는 경우 ZIP 압축 파일만 만들면 됩니다.

## CTest

[참조: freehuni's home](http://freehuni.blogspot.com/2017/12/ctest.html)

CTest는 cmake로 구상한 빌드환경에 빌드 및 테스트를 수행하고 그 결과를 dashboard에 게재할 수 있도록 도와줍니다.

### CTest 예제

일반적으로 CMake 프로젝트의 폴더들은

- `lib` 폴더
- `src` 폴더
- `test` 폴더

로 구성 됩니다.

```cmake
cmake_minimum_required(VERSION 2.8)
project(hello_ctest)

add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(test)

include (CTest) # CTest 모듈을 포함합니다.
add_test(test1 test/test_hello1) # 테스트를 수행할 실행파일을 지정합니다.
add_test(test2 test/test_hello2) # 테스트를 수행할 실행파일을 지정합니다.
add_test(test3 test/test_hello3) # 테스트를 수행할 실행파일을 지정합니다.
```

### `lib` 폴더

`lib` 폴더의 `CMakeLists.txt`입니다.

```cmake
cmake_minimum_required(VERSION 2.8)
project(utility)

add_library(utility SHARED utility.cpp)
```

`lib` 폴더의 `utility.cpp`는 아래와 같습니다.

```c++
#include "utility.h"

int g_value = 0;

void setValue(int value)
{
	g_value = value;
}

int getValue(void)
{
	return g_value;
}
```

### `src` 폴더

`src` 폴더의 `CMakeLists.txt` 파일은 아래와 같습니다.

```cmake
cmake_minimum_required(VERSION 2.8)
project(hello_application)

include_directories(../lib)
add_executable(hello main.cpp)

add_dependencies(hello utility)
target_link_libraries(hello utility)
```

`src` 폴더의 `main.cpp` 파일은 아래와 같습니다.

```c++
#include <stdio.h>
#include "utility.h"

int main(int argc, char* argv[])
{
	printf("previous: %d\n", getValue());
	setValue(1000);
	printf("after   : %d\n", getValue());
	return 0;
}

```

### `test` 폴더

`test` 폴더의 `CMakeLists.txt` 파일은 아래와 같습니다.

```cmake
cmake_minimum_required(VERSION 2.8)
project(unit_test)

include_directories(../lib)

add_executable(test_hello1 test1.cpp)
add_dependencies(test_hello1 utility)
target_link_libraries(test_hello1 utility)

add_executable(test_hello2 test2.cpp)
add_dependencies(test_hello2 utility)
target_link_libraries(test_hello2 utility)

add_executable(test_hello3 test3.cpp)
add_dependencies(test_hello3 utility)
target_link_libraries(test_hello3 utility)
```

`test` 폴더의 `test1.cpp`, `test2.cpp`, `test3.cpp`파일은 아래와 같습니다.

```c++
#include <stdio.h>
#include "utility.h"

int main(int argc, char* argv[])
{
	setValue(1000);
	printf("%s evaluate value: %d\n", __FILE__, getValue());
	return 0;
}
```

### 테스트

```sh
mkdir -p build # 빌드 폴더 만들기
cd build
cmake .. # 구성
make -j4 # 컴파일 및 링크
ctest # 테스트
```

`ctest` 결과는 아래와 같습니다.

```
Test project /ctest
    Start 1: test1
100% tests passed, 0 tests failed out of 1
Total Test time (real) =   0.01 sec
```

## CDash

CTest는 빌드 및 테스트 결과를 dashboard에 report 합니다. [my.cdash.org](https://my.cdash.org)를 사용하여 dashboard를 CTest 결과를 Web으로 조회할 수 있습니다.

CDash를 사용하려면 최상위 폴더에 `CTestConfig.cmake` 파일을 추가합니다.

```cmake
set(CTEST_PROJECT_NAME "hello ctest")
set(CTEST_NIGHTLY_START_TIME "01:00:00 UTC")
set(CTEST_DROP_METHOD "http")
set(CTEST_DROP_SITE "my.cdash.org")
set(CTEST_DROP_LOCATION "/submit.php?project=hello+ctest")
set(CTEST_DROP_SITE_CDASH TRUE)
```

여기서 `CTEST_DROP_METHOD`에는 다음 옵션들이 있습니다.

- cp
- ftp
- http
- https
- scp
- xmlrpc

그리고 

```sh
rm -rf build
mkdir build
cd build
cmake ..
# 여기서 make를 하지 않습니다!
ctest -D Experimental # 이렇게 하면 make까지 수행 되며, 미리 make를 하면 보고가 되지 않을 수 있습니다.
```

이제 `Build.xml`, `Configuration.xml`, `Test.xml`이 대시보드 사이트로 전송됩니다.

## 참조

- [CMake Official Reference Documentation](https://cmake.org/documentation)
- [CMake 할때 쪼오오금 도움이 되는 문서](https://gist.github.com/luncliff/6e2d4eb7ca29a0afd5b592f72b80cb5c#cmake-%ED%8A%9C%ED%86%A0%EB%A6%AC%EC%96%BC-lv2)
- [CGold: The Hitchhiker’s Guide to the CMake](https://cgold.readthedocs.io/en/latest/)
- [CMake and Visual Studio](https://cognitivewaves.wordpress.com/cmake-and-visual-studio/)
- [CMake 튜토리얼](https://www.tuwlab.com/27234)
- [CMake 사용법](https://m.blog.naver.com/PostView.nhn?blogId=likeafree&logNo=221475154955&proxyReferer=https:%2F%2Fwww.google.com%2F)
- [CTest](http://freehuni.blogspot.com/2017/12/ctest.html)
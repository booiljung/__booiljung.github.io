[Up](index.md)

# Ubuntu, VSCode에서 Catch2 사용법

2020년 5월 25일

C++에는 많은 단위테스트 도구들이 있습니다. 그중 Catch2가 선호도가 높습니다. ROS 패키지들도 Catch2를 사용하기도 합니다. 여기서 글에서 개발 환경은 Ubuntu, git, Visual Studio Code, cmake 입니다.

먼저 `Catch2` 소스코드를 다운로드 합니다.

```sh
cd ~
git clone https://github.com/catchorg/Catch2
```

오피셜 문서는 카피를 하라고 하는데 확인해 보니 `CMakeLists.txt` 파일이 있었고, cmake를 해보니 빌드 및  설치가 되었습니다.

```sh
cd Catch2
mkdir build
cd build
cmake ..
make
```

빌드에 성공하면 설치를 합니다.

```
sudo make install
```

특별히 설치 경로를 지정하지 않으면 기본 경로인  `/us/local/include/catch2`에 설치됩니다. 설치에 성공하였으면 예제 코드 `010_test_case.cpp`를 작성해 봅니다.  이 코드는 오피셜 예제로 제공되며 파일명을 c 규칙으로 변경 하였습니다.

```c++
// Let Catch provide main():
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

int factorial( int number )
{
//  return number <= 1 ? number : Factorial( number - 1 ) * number;  // fail
    return number <= 1 ? 1      : Factorial( number - 1 ) * number;  // pass
}

TEST_CASE("Factorial of 0 is 1 (fail)", "[single-file]")
{
    REQUIRE( factorial(0) == 1 );
}

TEST_CASE( "Factorials of 1 and higher are computed (pass)", "[single-file]" )
{
    REQUIRE( factorial(1) == 1 );
    REQUIRE( factorial(2) == 2 );
    REQUIRE( factorial(3) == 6 );
    REQUIRE( factorial(10) == 3628800 );
}
```

이 코드는 

```sh
g++ -std=c++11 -Wall -o 010_test_case 010_test_case.cpp && 010_test_case --success
```

하면 빌드합니다. Catch2는 헤더파일로만 구성되기 때문에 별도의 라이브러리 파일을 링크를 요구하지 않습니다. 기본 설치 폴더 `/usr/local/include/catch2/`에 설치하였을 경우 `-I` 옵션은 필요하지 않습니다. 실행은

```sh
./010_test_case --reporter compact --success
```

로 할 수 있습니다.

CMake로 빌드하는 경우 `CMakeLists.txt`는 아래와 같습니다.

```sh
set(name 010_test_case)
project(${name})
cmake_minimum_required(VERSION 3.5)
set(SOURCE 010_test_case.cpp)
add_compile_options(-g -Wall -std=c++11)
add_executable(${name} ${SOURCE})
```

빌드하려면 

```sh
mkdir build
cd build
make
```

그리고 실행합니다.

```sh
./010_test_case --reporter compact --success
```

결과는 

```
010_test_case.cpp:14: failed: factorial(0) == 1 for: 0 == 1
010_test_case.cpp:18: passed: factorial(1) == 1 for: 1 == 1
010_test_case.cpp:19: passed: factorial(2) == 2 for: 2 == 2
010_test_case.cpp:20: passed: factorial(3) == 6 for: 6 == 6
010_test_case.cpp:21: passed: factorial(10) == 3628800 for: 3628800 (0x375f00) == 3628800 (0x375f00)
Failed 1 test case, failed 1 assertion.
```

실패 예제를 확인하였으면 `factorial` 함수를 변경하여 테스트가 성공하도록 시도하고 확인해 보세요.

## 참조

[Catch2](https://github.com/catchorg/Catch2)


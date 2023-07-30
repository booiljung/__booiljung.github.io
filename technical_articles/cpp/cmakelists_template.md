# CMakeLists.txt  템플릿

CMakeLists.txt 기본 템플릿 파일이다. 이 파일에서 확장해 나갈 수 있다.

```cmake
cmake_minimum_required(VERSION 3.0)

set(name "cppdemo")
project(cppdemo)
SET(PROJECT_VERSION_MAJOR 0)
SET(PROJECT_VERSION_MINOR 1)

set(libs    
    pthread
    stdc++fs
)

#
#add_library(mylibs   
#)
#

add_executable(cppdemo src/main.cpp)
target_link_libraries(
    cppdemo
    #mylibs
    ${libs}
)
```


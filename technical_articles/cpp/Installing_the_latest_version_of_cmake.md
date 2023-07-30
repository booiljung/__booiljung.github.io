# Installing the latest version of cmake

먼저 [이 곳](https://cmake.org/download/) 에서 최신 버전의 소스 코드를 다운로드 합니다. 2020년 6월 2일 현재 최신버전은 3.17.3이며 예제로 cmake-3.17.3.tar.gz을 다운로드 하겠습니다.

```sh
wget https://github.com/Kitware/CMake/releases/download/v3.17.3/cmake-3.17.3.tar.gz
```

압축을 해제 합니다.

```sh
tar -xf cmake-3.17.3.tar.gz
cd cmake-3.17.3
```

빌드를 합니다.

```sh
./bootstrap
make -j8
sudo make install
```

## 참조

- [cmake 업그레이드 하기](https://blog.naver.com/PostView.nhn?blogId=chandong83&logNo=221672989050)


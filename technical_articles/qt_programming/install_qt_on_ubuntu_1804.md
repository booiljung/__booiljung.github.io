# Install Qt5 on Ubuntun 18.04 LTS

[이곳](http://download.qt.io/official_releases/qt/)에서 설치할 버전과 URL을 확인 한다.

터미널에서 설치 프로그램을 다운로드 한다.

```
wget http://download.qt.io/official_releases/qt/5.13/5.13.2/qt-opensource-linux-x64-5.13.2.run
```

빌드 도구를 설치한다.

```
sudo apt-get install build-essential
```

폰트를 설치 한다.

```
sudo apt-get install libfontconfig1
```

OpenGL 라이브러리를 설치한다.

```
sudo apt-get install mesa-common-dev
```

추가 패키지를 설치 한다.

```
sudo apt-get install libglu1-mesa-dev
```

qmake 오류를 바로 잡는다.

```
sudo update-alternatives --install /usr/bin/qmake qmake /usr/lib/x86_64-linux-gnu/qt5/bin/qmake 100
```

## 참조

- [Install Qt 5 on Ubuntu](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)


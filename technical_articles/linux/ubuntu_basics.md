# 우분투 사용법

## 설치

영어로 설치하는 편이 좋다. 한국어로 설치하면 디렉토리 이름이 한국어로 만들어지고 터미널에서 사용하기가 불편해진다.

영어로 설치하면 한국어 입력기가 설치되지 않는데 별도로 추가 설치해야 한다.

## 기본 개요

사용자는 user 다.

유저의 홈 디렉토리는 `~` 다.

루트 디렉토리는 `/` 이며, 각 폴더는 `/`로 구분한다. 윈도우는 `\`으로 다릅니다. 리눅스는 드라이브 `C:` 이름이 없다.

파일시스템의 루트는 아래와 같다.

| 디렉토리 이름  | 용도                                                         |
| -------------- | ------------------------------------------------------------ |
| `/bin`         | 리눅스 기본 명령들이 있습니다.                               |
| `/boot`        | 리눅스 커널들이 있습니다.                                    |
| `/dev`         | 장치(device)가 있습니다.                                     |
| `/etc`         | 환경 설정 파일들이 있습니다.                                 |
| `/home`        | 사용자와 서비스들의 계정이 위치 합니다.                      |
| `/lib`         | `/bin`에서 사용하는 라이브러리들이 설치 됩니다.              |
| `/media`       | 추가 디스크드라이브들이 마운트됩니다.                        |
| `/mnt`         | 외부 장치를 마운트 하는데 사용된다.                          |
| `/opt`         | 애드온 소프트웨어들이 설치 된다.                             |
| `/proc`        | 프로세스들을 볼 수 있습니다.                                 |
| `/root`        | 시스템 관리자의 홈 디렉토리입니다.                           |
| `/tmp`         | 임시 파일들을 사용할때 이 폴더를 사용합니다.                 |
| `/usr`         | 프로그램들이 설치됩니다. 라이브러리나 헤더 파일들도 설치됩니다. |
| `/usr/bin`     | 실행 프로그램들이 설치 됩니다.                               |
| `/usr/etc`     | 위 실행프로그램들의 환경 파일들이 있습니다.                  |
| `/usr/include` | C/C++ 헤더파일들이 위치합니다.                               |
| `/usr/lib`     | 라이브러리 파일들이 위치합니다.                              |
| `/usr/local`   | `/lib`와 유사한며 시스템 계정에 의해 추가 설치하면 이곳에 설치됩니다. |
| `/var`         | 로그나 메일, 데이터베이스 등의 동적인 파일들이 위치 합니다.  |

기본적으로 사용자는 `~` 자신의 홈 디렉토리에서만 어떤 작업을 할 수 있다. 다른 디렉토리를 변경하려면 시스템 권한을 요구한다.

**파일 탐색기**

윈도우 파일 탐색기에 해당하는 프로그램은 nautilus(노틸러스)가 설치되어 있다. 탐색기와 동일하게 사용할 수 있다. 추가로 nemo(니모)를 설치 할 수 있다.

**웹브라우저**

기본적으로 Firefox가 설치되어 있다. 추가로 크롬이나 오페라 등을 설치 할 수 있다.

**오피스**

기본적으로 LibreOffice (리브레오피스)가 설치되어 있다.

**GUI 셸**

기본적으로 GUI는 Gnome이 설치되어 있다. 영어 발음상 노움이 맞지만, 공식적으로 그노움이라고 발음하라고 한다.

윈도우키를 누르면 검색 창이 뜬다. 파일명이나 프로그램 이름을 실행하면 빠르게 검색하여 실행 할 수 있다.

윈도우 키를 누르고 `term`을 입력해보자 터미널이 실행 될 것이다.

리눅스 터미널의 기본 셸은 `bash`이며 `bash`는 빠른 작업을 가능하게 해줄 것이다.

**터미널 기본 명령어**

`ls` 를 하여 파일 목록을 볼 수 있다.

`cd /` 하여 루트 디렉토리로 이동 할 수 있다.

`cd ~`  하여 사용자 홈으로 이동할 수 있다.

`rm <file_name>`하여 파일을 삭제할 수 있다. 디렉토리는 삭제되지 않는다.

`mv <file_name> <file_name2>`를 하여 파일을 이동하거나 이름을 변경할 수 있다.

`mkdir <directory_name>`을 하여 디렉토리를 만들 수 있다.

`rm -rf <directory_name>`을 하여 디렉토리를 삭제 할 수 있다.

**bash 셸**

유닉스(+리눅스)에는 여러 셸이 있다. 터미널을 실행하면 기본 설정된 셸이 실행 된다.

탭키를 누르면 현재 디렉토리에서 파일을 자동으로 찾아 표시해준다.

업다운 키를 누르면 이전 명령 내역을 표시해 준다.

## 패키지 설치

리눅스의 장점은 패키지 설치에 있다. 윈도우의 경우 지원이 부족하여 오픈소스 설치가 까다롭고 시간을 소비한다. 개발자들이 리눅스를 선호하는 이유다.

우분투는 데비안에서 갈라져 나왔다. 데비안의 탄생에는 [데이브와 이안의 아름답고도 슬픈 얘기가 있다.](https://joone.net/2020/05/09/33-%EB%A6%AC%EB%88%85%EC%8A%A4-%EB%B0%B0%ED%8F%AC%EB%B3%B8%EA%B3%BC-%EB%8D%B0%EB%B9%84%EC%95%88-%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8/)

데비안은 `apt` 명령을 사용한다.

`apt` 명령을 사용하면 시스템 디렉토리에 설치가 된다. 그래서 `apt` 명령은 시스템 권한을 필요로 하며, `sudo` 명령을 통해서 해야 한다.

`sudo` 명령은 임시로 시스템 권한을 사용할 수 있게 해준다.

패키지를 추가하거나 제거, 검색하는 명령은 `apt-get` 명렁어다.

`sudo apt-get install <패키지네임>`을 하여 데비안 패키지를 설치한다.

`git`을 설치하려면 `sudo apt-get install git`을 하면 된다.

`sudo apt-get remove <패키지네임>`을 하여 데비안 패키지를 제거한다.

`apt-get`은 간단히 `apt`로 축약하여 사용할 수 있다. 

`sudo apt search <패키지네임>`으로 검색 할 수 있다.

`sudo apt update`로 데비안 패키지 저장소에서 정보를 가져와 동기화 할 수 있다.

`sudo apt upgrade`는 설치된 기존 패키지들을 최신 버전으로 업그레이드 한다.

**한국어 입력기 설치하기**

https://booiljung.github.io/technical_articles/linux/ubuntu_korean_fcitx_installation.html

**Visual Studio Code 설치하기**

수동으로 설치할 수도 있다. https://code.visualstudio.com/

자동으로 업그레이드 되게 하려면 터미널에서 설치해야 한다. 다음 스크립트 자동으로 설치하는 스크립트다.

```bash
#!/bin/bash
# vscode
pushd .

mkdir -p ~/.cache
cd ~/.cache

curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
sudo bash -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt install apt-transport-https -y
sudo apt update
sudo apt install code -y # or code-insiders

popd
```

**크롬 브라우저 설치하기**

```bash
#!/bin/bash

wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
sudo bash -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt update
sudo apt install -y google-chrome-stable
sudo rm -rf /etc/apt/sources.list.d/google.list
```


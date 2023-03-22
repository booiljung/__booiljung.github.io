# Dockerfile 작성 패턴

Dockerfile 문법을 설명하는 자료는 많지만, Dockfile을 쉽게 작성하는 방법은 찾기 어렵습니다.

Dockerfile을 작성하려면

- Dockerfile 문법에 대한 이해
- Linux 명령, 파일 시스템, 유저, 패키지 라이브러리에 대한 이해
- 설치하고자하는 애플리케이션에 대한 이해

에 대한 지식들이 필요합니다.

설치하고자 하는 애플리케이션에 대한 이해를 제외하고 패턴이 있다는 것을 깨달았습니다.

*제가 ROS를 다루는 이유로 Ubuntu 기준으로 작성합니다.*

## 시험용 Ubuntu

Dockerfile을 수정 및 빌드해가며 오류를 찾아내기도 하지만 시간이 걸리는 편입니다. 직접 컨테이너에서 설치해가며 관련 패키지 의존성을 확인해야 할 경우도 있습니다.

다음과 같이 짧은 Dockerfile로 우분투 이미지를 만들고 실행합니다.

```dockerfile
FROM ubuntu:20.04

ENTRYPOINT ["/bin/bash"]
```

그리고 빌드하고:

```sh
$ docker build . -t myubuntu
```

실행합니다:

```sh
$ docker run -it myubuntu
```

그러면 컨테이너가 올라오며 컨테이너의 bash 셸로 들어가게 되고 리눅스 명령을 사용할 수 있게 됩니다.

여기에서 의존성을 설치하며 테스트 할 수 있습니다. 확정이 되면 변경 사항을 Dockfile에 기록 하고 Ctrl+C를 눌러서 컨테이너를 종료 합니다.

```dockerfile
FROM ubuntu:20.04

<<< 변경 사항은 이 사이에 추가 될 것입니다.

ENTRYPOINT ["/bin/bash"]
```

변경사항을 다시 `build`하고 `run`하여 Dockerfile을 완성해 갑니다.

마지막에 ENTRYPOINT를 변경하면 bash가 시작되지 않고 해당 명령이 실행 될 것입니다.

## Dockerfile 패턴

Dockerfile 작성 패턴은 몇개 부분으로 나눌 수 있습니다. 

- 베이스 이미지
- 작업 디렉토리
- 사용자 계정
- 패키지 라이브러리 설치
- 소스코드 다운로드 및 빌드
- 네트워크 구성
- 진입점 명령 선언

### 베이스 이미지

만들려는 컨테이너 이미지의 기반이 되는 이미지를 지정합니다.

이 예제는  Ubuntu 20.04로 합니다:

```dockerfile
FROM ubuntu:20.04
```

### 작업 디렉토리

`WORKDIR`은 현재 디렉토리를 지정합니다. 디렉토리가 없으면 빌드시스템이 디렉토리를 새로 만듭니다.

이 예제는 `/app` 디렉토리에 애플리케이션을 설치합니다:

```dockerfile
WORKDIR /app
```

작업 디렉토리는 언제라도 변경 할 수 있습니다.

### 사용자 계정

다음은 `appuser`라는 사용자 계정과 홈 디렉토리를 생성하는 패턴입니다. 어느 컨테이너 이미지의 경우 USERNAME이 사전에 정의된 경우가 있습니다. 여기서는 USER_NAME이라는 변수로 합니다:

```dockerfile
ARG USER_NAME=appuser
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash
```

### 데비안 패키지 설치

데비안 패키지를 설치하는 경우 대화 모드가 되어 Dockerfile 빌드가 중단되는 경우가 있습니다. 이를 방지하고 설치하는 패턴입니다:

```dockerfile
USER root

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update --no-install-recommends \
    && apt upgrade --no-install-recommends -y \
    && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    ca-certificates \
    lsb-release \
    sudo \
    tzdata \
    bash-completion \    
    .... \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-wheel
```

먼저 `USER root`로 하였는데  `root` 권한 -`sudo` 명령이 필요한 경우 `USER root`를 통해 `root` 계정으로 전환한 후에 해당 작업을 해야 합니다.

### 파이썬 패키지 설치

파이썬 패키지를 설치합니다. 이전에 `root` 계정으로 전환 되었으면  반복할 필요는 없습니다.

```dockerfile
USER root

RUN pip3 install numpy
```

### 일반 사용자 계정에 sudo 권한 부여

보안 문제로 root 계정으로 애플리케이션을 설치하는 것은 권장 되지 않으며 빌드 및 설치가 되지 않습니다. 그래서, 일반 사용자 계정으로 설치하고 sudo 권한을 부여하는 패턴 입니다:

```dockerfile
ENV USER=${USER_NAME}

RUN echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}
```

### 설치 디렉토리를 일반 유저 소유로 변경

```dockerfile
RUN chown -R ${USER_NAME}:${USER_NAME} /app
```

### 일반 사용자 계정으로 전환

```dockerfile
USER ${USER_NAME}
```

### 애플리케이션 디렉토리를 사용자 계정에 권한 부여

```dockerfile
RUN mkdir -p /app && sudo chmod -R 777 /app
```

### 소스코드 클론 및 서브 모듈 설치

```dockerfile
WORKDIR /app

RUN git config --global http.postBuffer 1048576000
RUN git clone https://github.com/clone/these .
RUN git checkout Copter-4.3.4 -b mine/new-branch
RUN git submodule update --init --recursive
```

### 소스코드 빌드

소스코드 빌드는 애플리케이션마다 다르므로 다루지 않겠습니다. 다만, 일반 유저로 빌드해서 다뤄야 합니다:

```dockerfile
USER ${USER_NAME}

...  빌드
```

### 데비안 임시 파일 삭제

```dockerfile
# Cleanup
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
```

### 네트워크 리슨 포트 노출

```dockerfile
EXPOSE xxxxx
EXPOSE yyyyy
```

### 환경 변수 및 진입점 스크립트 생성

환경 변수 및 진입 스크립트를 작성합니다.

```dockerfile
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/app_entrypoint.sh

RUN export APP_ENTRYPOINT="/home/${USER_NAME}/app_entrypoint.sh" \
    && echo "#!/bin/bash" > $APP_ENTRYPOINT \
    && echo "set -e" >> $APP_ENTRYPOINT \
    && echo "source /home/${USER_NAME}/.app_env" >> $APP_ENTRYPOINT \
    && echo 'exec "$@"' >> $APP_ENTRYPOINT \
    && chmod +x $APP_ENTRYPOINT \
    && sudo mv $APP_ENTRYPOINT /app_entrypoint.sh
```

### 진입점 명령 선언

컨테이너 시작시 실행할 스크립트와 명령을 지정합니다:

```dockerfile
ENV CCACHE_MAXSIZE=1G
ENTRYPOINT ["/app_entrypoint.sh"]
CMD ["bash"]
```

### 참조

- [Dockerfile reference](https://docs.docker.com/engine/reference/builder/)


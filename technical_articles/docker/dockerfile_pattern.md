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

Dockerfile을 수정 및 빌드해가며 오류를 찾아내기도 하지만 시간이 걸리는 편입니다. 특별한 경우 Ubuntu 가상 머신에서 설치해가며 관련 패키지 의존성을 확인해야 할 경우도 있습니다. 컨테이너와 동일한 환경으로 Unbuntu VM을 만들어 보겠습니다.

먼저 virtualbox를 설치 합니다.

```sh
$ sudo apt install virtualbox -y
```

Virtual Box에 Ubuntu를 다운로드 받아 설치 합니다.

**주의: 이 Ubuntu VM은 보안상 문제가 있으므로 Dockerfile 작성을 위한 시험용으로만 사용해야 합니다.**

Dockerfile 빌드는 root 계정으로 시작되며 다른 사용자 계정은 없는 상태에서 시작 됩니다. Ubuntu를 Dockerfile 빌드와 동일한 사용자로 시작 하도록 준비합니다.

root로 로그인을 위해 root 계정의 비밀번호를 생성합니다.

```sh
ubuntu@bijung:~$ pwd 
/home/bijung

ubuntu@bijung:~$ whoami 
bijung

ubuntu@bijung:~$ sudo passwd root 
[sudo] password for bijung:
New password:  
Retype new password:  
passwd: password updated successfully

ubuntu@bijung:~$ su - root 
Password: 

root@bijung:~$ whoami 
root

root@bijung:~$  
```

ssh로 접속을 허용할 경우 `/etc/ssh/sshd_config` 파일에서 `PermitRootLogin` 항목을 `yes`로 변경합니다:

```sh
$ nano /etc/ssh/sshd_config
PermitRootLogin yes

$ sudo service ssh restart
```

GUI에서 root 계정으로 접속을 허용하려면 `/etc/gdm3/custom.conf` 파일을 수정하여 3개의 항목을 추가 합니다.

```ini
# GDM configuration storage
#
# See /usr/share/gdm/gdm.schemas for a list of available options.

[daemon]
AutomaticLoginEnable=true << 추가
AutomaticLogin=root << 추가

# Uncoment the line below to force the login screen to use Xorg
#WaylandEnable=false

# Enabling automatic login

# Enabling timed login
#  TimedLoginEnable = true
#  TimedLogin = user1
#  TimedLoginDelay = 10

[security]
AllowRoot=true << 추가

[xdmcp]

[chooser]

[debug]
# Uncomment the line below to turn on debugging
# More verbose logs
# Additionally lets the X server dump core if it crashes
#Enable=true
```

`/etc/pam.d/gdm-password`에서 한개 항목을 주석처리 합니다:

```ini
#%PAM-1.0
auth    requisite       pam_nologin.so
#auth	required	pam_succeed_if.so user != root quiet_success << 주석처리
@include common-auth
auth    optional        pam_gnome_keyring.so
@include common-account
# SELinux needs to be the first session rule. This ensures that any 
# lingering context has been cleared. Without this it is possible 
# that a module could execute code in the wrong domain.
session [success=ok ignore=ignore module_unknown=ignore default=bad]        pam_selinux.so close
session required        pam_loginuid.so
# SELinux needs to intervene at login time to ensure that the process
# starts in the proper default security context. Only sessions which are
# intended to run in the user's context should be run after this.
session [success=ok ignore=ignore module_unknown=ignore default=bad]        pam_selinux.so open
session optional        pam_keyinit.so force revoke
session required        pam_limits.so
session required        pam_env.so readenv=1
session required        pam_env.so readenv=1 user_readenv=1 envfile=/etc/default/locale
@include common-session
session optional        pam_gnome_keyring.so auto_start
@include common-password
```

`/etc/pam.d/gdm-autologin`에서 한개 항목을 주석처리 합니다:

```ini
#%PAM-1.0
auth    requisite       pam_nologin.so
#auth	required	pam_succeed_if.so user != root quiet_success <<< 주석처리
auth	optional	pam_gdm.so
auth	optional	pam_gnome_keyring.so
auth    required        pam_permit.so
@include common-account
# SELinux needs to be the first session rule. This ensures that any 
# lingering context has been cleared. Without this it is possible 
# that a module could execute code in the wrong domain.
session [success=ok ignore=ignore module_unknown=ignore default=bad]        pam_selinux.so close
session required        pam_loginuid.so
# SELinux needs to intervene at login time to ensure that the process
# starts in the proper default security context. Only sessions which are
# intended to run in the user's context should be run after this.
session [success=ok ignore=ignore module_unknown=ignore default=bad]        pam_selinux.so open
session optional        pam_keyinit.so force revoke
session required        pam_limits.so
session required        pam_env.so readenv=1
session required        pam_env.so readenv=1 user_readenv=1 envfile=/etc/default/locale
@include common-session
session optional        pam_gnome_keyring.so auto_start
@include common-password
```

이제 재부팅을 하면  root 계정으로 자동 로그인되는 VM 이미지를 얻었습니다. 이 이미지를 클론하여 Dockerfile 작성 시험용으로 사용하시면 됩니다.

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


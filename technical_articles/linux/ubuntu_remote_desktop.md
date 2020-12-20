# 우분투에서 원격데스크탑 설치

Installation of Remote Desktop On Ubuntu 18.04 LTS

## 18.04에서 xrdp 로 접속

간단하게 할 수 있다는 얘기가 있습니다.

```sh
sudo apt install xrdp -y
sudo systemctl enable xrdp
sudo ufw allow 3389/tcp
```

### dock 숨기 문제 해결

원격 접속시 DOCK이 숨김 상태로 전환 되어 있고, 설정에서 설정이 불가능한데, 이는 gnome-tweak-tool 을 설치해서 설정하면 해결 됩니다.

20.04 이후 버전에서는

```sh
sudo apt install dconf-editor -y
dconf-editor
```

하여

`/etc/gnome/shell/extensions/dash-to-dock` 에서  `extend height`를 활성화 합니다.


출처: https://velog.io/@springkim/Ubuntu-18.04-%EC%9B%90%EA%B2%A9-%EB%8D%B0%EC%8A%A4%ED%81%AC%ED%86%B1

## xrdp 로 접속하는 간단하지 않는 방법

### xrdp

우분투에서 원격 데스크탑을 제공하려면 xrdp를 설치해야 합니다.

```sh
sudo apt update
sudo apt upgrade
sudo apt install xrdp -y
```

### mate 데스크탑 환경 설치 (옵션)

```sh
sudo apt-get install mate-core mate-desktop-environment mate-notification-daemon -y
```

### xfce4

xrdp 만으로 접속이 불가능합니다. 추가적으로 xfce4를 설치해야 합니다.

```sh
sudo apt install xfce4 -y
```

### 화면이 나오지 않을 경우

화면이 검은색 또는 파란색으로 냐올 경우 다음을 설치 합니다.

```sh
sudo apt-get install xorgxrdp-hwe-18.04 -y
```

### startwm.sh

- 수정하지 않아도 된다는 얘기가 있다.

`/etc/xrdp/startwm.sh` 파일을 보면 아래와 같습니다.


```
#!/bin/sh
# xrdp X session start script (c) 2015, 2017 mirabilos
# published under The MirOS Licence

if test -r /etc/profile; then
	. /etc/profile
fi

if test -r /etc/default/locale; then
	. /etc/default/locale
	test -z "${LANG+x}" || export LANG
	test -z "${LANGUAGE+x}" || export LANGUAGE
	test -z "${LC_ADDRESS+x}" || export LC_ADDRESS
	test -z "${LC_ALL+x}" || export LC_ALL
	test -z "${LC_COLLATE+x}" || export LC_COLLATE
	test -z "${LC_CTYPE+x}" || export LC_CTYPE
	test -z "${LC_IDENTIFICATION+x}" || export LC_IDENTIFICATION
	test -z "${LC_MEASUREMENT+x}" || export LC_MEASUREMENT
	test -z "${LC_MESSAGES+x}" || export LC_MESSAGES
	test -z "${LC_MONETARY+x}" || export LC_MONETARY
	test -z "${LC_NAME+x}" || export LC_NAME
	test -z "${LC_NUMERIC+x}" || export LC_NUMERIC
	test -z "${LC_PAPER+x}" || export LC_PAPER
	test -z "${LC_TELEPHONE+x}" || export LC_TELEPHONE
	test -z "${LC_TIME+x}" || export LC_TIME
	test -z "${LOCPATH+x}" || export LOCPATH
fi

if test -r /etc/profile; then
	. /etc/profile
fi

test -x /etc/X11/Xsession && exec /etc/X11/Xsession
exec /bin/sh /etc/X11/Xsession
```

여기에서 아래 3개의 `/etc/X11/Xsession` 을 `/usr/bin/startxfce4` 로 변경합니다. 변경 결과는 아래와 같습니다.

```
#!/bin/sh
# xrdp X session start script (c) 2015, 2017 mirabilos
# published under The MirOS Licence

if test -r /etc/profile; then
	. /etc/profile
fi

if test -r /etc/default/locale; then
	. /etc/default/locale
	test -z "${LANG+x}" || export LANG
	test -z "${LANGUAGE+x}" || export LANGUAGE
	test -z "${LC_ADDRESS+x}" || export LC_ADDRESS
	test -z "${LC_ALL+x}" || export LC_ALL
	test -z "${LC_COLLATE+x}" || export LC_COLLATE
	test -z "${LC_CTYPE+x}" || export LC_CTYPE
	test -z "${LC_IDENTIFICATION+x}" || export LC_IDENTIFICATION
	test -z "${LC_MEASUREMENT+x}" || export LC_MEASUREMENT
	test -z "${LC_MESSAGES+x}" || export LC_MESSAGES
	test -z "${LC_MONETARY+x}" || export LC_MONETARY
	test -z "${LC_NAME+x}" || export LC_NAME
	test -z "${LC_NUMERIC+x}" || export LC_NUMERIC
	test -z "${LC_PAPER+x}" || export LC_PAPER
	test -z "${LC_TELEPHONE+x}" || export LC_TELEPHONE
	test -z "${LC_TIME+x}" || export LC_TIME
	test -z "${LOCPATH+x}" || export LOCPATH
fi

if test -r /etc/profile; then
	. /etc/profile
fi

#test -x /etc/X11/Xsession && exec /etc/X11/Xsession
#exec /bin/sh /etc/X11/Xsession
test -x /usr/bin/startxfce4 && exec /usr/bin/startxfce4
exec /bin/sh /usr/bin/startxfce4
```

### 포트 번호 변경

추가적으로 포트 번호를 변경하려면 `/etc/xrdp/xrdp.ini` 파일을 열어서 `ListenPort=<port_number>` 를 변경해 줍니다.

### 테마 변경

저장소 추가 합니다.

```sh
sudo add-apt-repository ppa:noobslab/themes -y # themes
sudo add-apt-repository ppa:noobslab/icons -y # icons
sudo apt-get update
```

그리고 tweak 앱 설치 합니다.

```sh
sudo apt-get install gnome-tweak-tool
```

설치하고 나서 tweak 에서 테마를 변경 할 수 있습니다.


### 색상 프로필 변경

매번 접속시 색상 인증을 요구하는데 색상 프로필을 변경하면 인증을 요구하지 않게 됩니다.
추가적으로 색상 프로필을 변경하려면 `/etc/polkit-1/localauthority/50-local.d/45-allow-colord.pkla` 를 열어서 아래 내용을 추가 합니다.

```
[Allow Colord all Users]
Identity=unix-user:*
Action=org.freedesktop.color-manager.create-device;org.freedesktop.color-manager.create-profile;org.freedesktop.color-manager.delete-device;org.freedesktop.color-manager.delete-profile;org.freedesktop.color-manager.modify-device;org.freedesktop.color-manager.modify-profile
ResultAny=no
ResultInactive=no
ResultActive=yes
```



### mate 터미널

그리고 터미널에서

```sh
add-apt-repository ppa:martinx/xrdp-next 
apt-get update 
apt-get install X11* -y
apt-get install xserver-xorg-core-hwe-18.04 -y
apt-get install xorg-video-abi-24 
apt-get install xorgxrdp -y 
apt-get install tigervnc* -y
echo xterm > ~/.xsession
chmod +x ~/.xsession 
apt-get install xserver-xorg-core 
apt-get install tigervnc*
# echo mate-session > ~/.xsession
apt-get install mate-core 
ufw allow <port_number>/tcp 
service xrdp restart
```

다른 포트를 사용하겠다면 ufw 방화벽 인바운트 포트 번호를 적용해야 합니다.

마지막으로 xrdp 서비스를 재시작 합니다.

```sh
sudo service xrdp restart
```

서비스를 중지하려면 

```sh
sudo service xrdp stop
```

서비스를 시작하려면 

```sh
sudo service xrdp start
```

## VNC로 접속하는 방법

https://z-wony.tistory.com/19



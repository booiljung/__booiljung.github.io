# Ubuntu 에서 VirtualBox VM 자동으로 시작

## Ubuntu 18.04 LTS

`/etc/default/virtualbox`에 추가

```
VBOXAUTOSTART_DB=/etc/vbox
VBOXAUTOSTART_CONFIG=/etc/vbox/autostart.cfg
```

`/etc/default/virtualbox`를 변경

```
#SHUTDOWN=poweroff
SHUTDOWN=acpibutton
```

가상 컴퓨터 이름이 `vm001`이라고 하면

`/etc/systemd/system/vm001` 서비스 파일 생성 

```
[Unit]
Description=vm1
After=network.target virtualbox.service
Before=runlevel2.target shutdown.target
 
[Service]
User=smsoft
Group=vboxusers
Type=forking
Restart=no
TimeoutSec=5min
IgnoreSIGPIPE=no
KillMode=process
GuessMainPID=no
RemainAfterExit=yes
 
ExecStart=/usr/bin/VBoxManage startvm vm001 --type headless
ExecStop=/usr/bin/VBoxManage controlvm vm001 acpipowerbutton
 
[Install]
WantedBy=multi-user.target
```

데몬 리로드

```
sudo systemctl daemon-reload
```

서비스 시작, 재시작, 종료, 상태

```
sudo service vm001 start
sudo service vm001 restart
sudo service vm001 stop
sudo service vm001 status
```

자동 시작

```
sudo systemctl enable vm001
```

## 참조

- https://linux.m2osw.com/autostart-virtualbox-vms

# Ubuntu 18.04 LTS에 samba 설치

## 직접 설치

```
sudo passwd root
...
su
...
sudo apt install samba
sudo apt install system-config-samba # 우분투 18.04에서 오류
```

그리고

구성 폴더에 smb.conf 파일이 없다면 다음 예제를 사용.

원본 파일을 백업

```
sudo cp /etc/samba/smb.conf /etc/samba/smb.conf.back
```

그리고 `/etc/samba/smb.conf`를 편집

다음은 설정에 대한 설명

폴더를 추가하려면 `/etc/samba/smb.conf` 에 다음 내용을 추가

```
[표시 되는 폴더 이름]
comment = Workspace directory
path = /files
valid users = aero # 인증된 유저
admin users = aero
browsable = yes
writeable = yes
public = yes
read only = no
guest ok = no
create mode = 0777
directory mode = 0777
force user = aero
```

사용자 추가

```
sudo useradd <userid> # 유저가 없으면
sudo smbpasswd -a <userid>
```

방화벽 개방

```
sudo ufw allow 137,138/udp
sudo ufw allow 139,445/tcp
```

samba 상태 확인

```
smbstatus
```

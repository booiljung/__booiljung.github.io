# Ubuntu에서 root 계정 자동 로그인

우분투에서 보안 문제로 root 계정 사용이 제한되어 있습니다.

하지만, Dockerfile을 만들때 수동으로 문제를 해결하기 위해 가상머신에서 테스트를 해야 하는 경우 root 계정으로 시작해야 합니다.

## root 계정의 비밀번호를 생성

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

root@bijung:~# whoami 
root

root@bijung:~#  
```

## ssh로 root 계정 접속

`/etc/ssh/sshd_config` 파일에서 `PermitRootLogin` 항목을 `yes`로 변경합니다.

```
$ nano /etc/ssh/sshd_config
PermitRootLogin yes

$ sudo service ssh restart
```

## GUI에서 root 계정으로 접속하기

`/etc/gdm3/custom.conf` 파일을 수정하여 3개의 항목을 추가 합니다.

```
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

```
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

```
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

## 참조

- [우분투 ROOT 접속](https://growupcoding.tistory.com/39)
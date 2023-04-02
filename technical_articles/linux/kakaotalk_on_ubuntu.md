# Kakao Talk on Ubuntu

## PlayOnLinux

도커로 PlayOnLinux 설치는 실패.

PlayOnLinux 설치:

```sh
sudo apt install playonlinux
```

WineHQ 설치:

1. PlayOnLinux 창
   1. Tools
   2. Manage Wine Version
2. Wine versions (x86) 에서 6.14-staging 설치

Kakao Talk 설치 프로그램 다운로드:

```
cd ~/Downloads
wget https://app-pc.kakaocdn.net/talk/win32/KakaoTalk_Setup.exe
```

Kakao Talk 설치:

1. PlayOnLinux 창
   1. Install
2. PlayOnLinux install menu
   1. 아래 Install a non-listed program
3. PlayOnLInux Wizard
   1. Next
   2. Next
4. PlayOnLinux Manual installation
   1. Next
5. Select "Install a program in a virtual drive"
   1. Next
6. Input virtualdrive name
   1. Next
7. What would ...
   1. Check another version of wine
   2. Check Configure wine
   3. Check Install some library
   4. Next
8. Select 6.14-staging
9. Select 32 bits windows intallation
10. Wine configuration
    1. Application > Windows version > Select Windows 10
    2. OK
11. Please make your choice
    1. Select none
    2. Next
12. Please select the installation file to run
    1. Brows and select Kakaotalk setup exe
13. PlayOnLinux 창
    1. Configure
    2. KakaoTalk on left pane
    3. Miscellaneous
    4. Command to exec before running the program > LANG=ko_KR.UTF-8

## 참조

- https://shanepark.tistory.com/328

- https://yulistic.gitlab.io/2016/05/mint-linux-english-system-locale%EC%97%90%EC%84%9C-%EC%99%80%EC%9D%B8%EC%9C%BC%EB%A1%9C-%EC%8B%A4%ED%96%89%ED%95%9C-%EC%B9%B4%EC%B9%B4%EC%98%A4%ED%86%A1-%ED%95%9C%EA%B8%80-%EA%B9%A8%EC%A7%90%EB%AC%B8%EC%A0%9C-kakaotalk-cannot-display-korean-characters-in-english-system-locale./
[Up](index.md)

# Ubuntu에 nvidia 드라이버 설치하기

2020년 5월 30일

먼저 Ubuntu에서 제공되는 오픈소스 드라이버 nouveau가 자동으로 설치되지 않도록 블랙리스트에 등록 합니다.

```
blacklist nouveau
options nouveau modset=0
```

`/etc/modprobe.d/blacklist-nouvegau.conf` 파일에 저장합니다. 처음 Ubuntu를 설치했다면 해당 파일이 없을것이며 수퍼유저 모드로 파일을 열어서 편집하고 저장하면 될 것입니다.

```sh
sudo gedit /etc/modprobe.d/blacklist-nouvegau.conf
```

그리고 업데이트합니다.

```sh
sudo update-initramfs -u
```

그리고 재부팅 합니다.

```sh
reboot
```

재부팅이 완료되면 gdm을 끄고 자동으로 드라이버를 설치합니다.

```sh
sudo service gdm stop
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo ubuntu-drivers autoinstall
sudo reboot
```

재부팅이 끝나면 설치가 되었는지 확인할 수 있습니다.

```sh
nvidia-smi
```

쉽죠?


# Ubuntu Core

[절차](https://ubuntu.com/download/kvm)에 따르면:

[먼저 Ubuntu-one 계정을 만든다.](https://login.ubuntu.com/?_ga=2.254175880.1622768090.1642236891-97240807.1642236891)



SSH-Key를 발급한다:

```sh
ssh-keygen -t rsa -b 4096
```

발급이 되면 `~/.ssh/id_rsa.pub` 파일을 열어서 복사 후 ssh 창에 붙여 넣는다. 이 키를 rsa로 발급하면 `ssh-rsa`로 시작된다.



Ubuntu Core 20 image를 다운로드 하고 압축을 해제 한다:

```sh
unxz ubuntu-core-20-amd64.img.xz
```



kvm을 설치한다:

```sh
sudo apt install qemu-kvm ovmf
```

kvm-ok를 실행한다:

```sh
kvm-ok
```

```
INFO: /dev/kvm exists
KVM acceleration can be used
```

kvm 실행:

```sh
qemu-system-x86_64 -smp 2 -m 2048 -net nic,model=virtio -net user,hostfwd=tcp::8022-:22,hostfwd=tcp::8090-:80 -vga qxl -drive file=/usr/share/OVMF/OVMF_CODE.fd,if=pflash,format=raw,unit=0,readonly=on -drive file=ubuntu-core-20-amd64.img,cache=none,format=raw,id=disk1,if=none -device virtio-blk-pci,drive=disk1,bootindex=1 -machine accel=kvm
```

여기서:

- `localhost:8022` is redirecting to port `22` of the virtual machine for accessing it through SSH
- `localhost:8090` is redirecting to its port `80`
- `-vga qxl` sets the paravirtual graphics driver qxl

처음 부팅시 제법 시간이 걸린다.



부팅이 완료 되면 표시된다:

```
Ubuntu Core 20 on 10.0.15 (tty)
The host key fingerprints are:

    RSA     SHA256:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    ECDSA   SHA256:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    ED25519 SHA256:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

To login:

    ssh <Ubuntu SSO username>@10.0.2.15
    
Personalize your account at https://login.ubuntu.com
```



로컬에 로그인하려면 아래와 같이 한다:

```sh
ssh -p 8022 <Ubuntu SSO username>@localhost
```


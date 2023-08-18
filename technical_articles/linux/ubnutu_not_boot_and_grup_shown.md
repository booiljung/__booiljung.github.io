Ubuntu 20.04

# Ubuntu가 부팅되지 않고 `GRUB>`가 보이면

2023년 8월 18일

주의: 다른 배포판의 경우, 또는 우분투의 경우에도 다른 복구 방법이 있다고 하므로 다른 방법을 시도 하는 것도 추천 합니다. 이 글은 개인적인 성공 사례로 기록해 둡니다.

---

Ubuntu 20.04가 설치된 랩탑에 18.04를 설치하는데, 선택의 실수로 GRUP이 부팅 경로를 찾지 못하는 문제가 발생 하였습니다.

GRUB이 `/boot/grub/grub.cfg` 파일을 제외한 모든 항목을 찾았을 때는 아래와 같이 GRUB2 터미널이 표시 됩니다.

```
grub>
```

**GRUB 폴더를 찾을 수 없거나 손상된 경우에는 아래와 같이 GRUB2 터미널이 표시된다. 이 글에서 아래 문제는 다루지 않는다.**

```
grub rescue>
```

## 복구 절차

1. 먼저 GRUB2에게 부팅 방법을 수동으로 알려주어 부팅을 한 후
2. 부팅 된 우분투에서 GRUB을 복구 하도록 합니다.

## 1. 수동 부팅

먼저 `set`명령으로 현재 상태를 파악 합니다:

```
grub> set
?=0
color_highlight=black/white
color_normal=white/black
default=0
gfxmode=640x480
lang=en_US
local_dir=(hd0,msdos1)/boot/grub/locale
pager=
prefix=(hd0,msdos1)/boot/grub
root=hd0,msdos1
grub> _
```

`/boot/grub/grub.cfg` 파일을 찾지 못하는 경우는 `prefix`가 잘못된 경로를 가르키고 있는 경우입니다.

먼저 목록을 확인 합니다:

```
grub> ls
(hd0) (hd0,gpt?) (hd0,gpt?)
```

**`hd0`는 물리적 디스크 장치, `gpt?`는 파일 시스템에 따라 다르며 파티션을 나타내며 장비의 장치와 파티션 및 파일 시스템에 따라 다르게 표시 될 것입니다.**

어느 장치에 `/boot` 경로가 있는지 확인 합니다. 참고로 이 `/boot` 경로는 리눅스가 설치되면 `/` 에 나타나는 `/boot` 폴더와 일치 합니다.

![image-20230818223212632](/home/bijung/booiljung/booiljung.github.io/technical_articles/linux/ubnutu_not_boot_and_grup_shown.assets/image-20230818223212632.png)

모르겠으면 각 파티션별로 `ls` 명령으로 확인합니다:

```
grub> ls (hd?,gpt?)/boot
...
grub> ls (hd?,gpt?)/boot
...
```

만일 `/boot` 폴더가 있다면 아래처럼 표시 될 것입니다:

```
config-5.19.0-50-generic ㅑnitrd.img memtest86+.bin System.map-6.2.0-26-generic vmlinuz.old config-6.2.0-26-generic   initrd.img-5.19.0-50-generic  memtest86+.elf vmlinuz
efi initrd.img-6.2.0-26-generic memtest86+_multiboot.bin vmlinuz-5.19.0-50-generic grub  initrd.img.old System.map-5.19.0-50-generic vmlinuz-6.2.0-26-generic
```

`/boot` 폴더에 `vmlinuz`와 `initrd` 파일들이 있어야 합니다.

`/boot` 폴더를 찾았으면 `root`경로를 알려 줍니다. 이 작업은 디스크에 저장되지 않습니다:

```
grub> set root=(hd0,gpt?)
```

linux 명령으로 리눅스 커널 이미지 경로와 최상위 마운트 경로가 있는 `/dev` 경로를 지정해 줍니다. 이 작업은 디스크에 저장되지 않습니다. 경로가 제대로 지정않는 경우 부팅이 되지 않고 busybox가 실행 될 것입니다. 이 경우 다시 시작하여 반복 할 수 있습니다:

```
grub> linux /boot/vmlinuz root=/dev/nvme0n1p?
```

마지막으로 초기 마운트 파일 시스템 이미지 경로를 지정해 줍니다:

```
grub> initrd /boot/initrd.img
```

이제 부팅을 시작 합니다:

```
grub> boot
```

부팅이 성공하면 리눅스에서 터미널을 통해 GRUB를 복구 합니다.

## 2. GRUB 복구

리눅스 터미널을 시작 합니다:

```sh
$ update-grub
```

디스크에 GRUB를 재설치 합니다:

```sh
$ grub-install /dev/nvme0n1p?
```

다시 부팅을 해봅니다.

```sh
$ boot
```

## 참조

- https://blog.neonkid.xyz/225
- https://ko.linux-console.net/?p=4119#gsc.tab=0
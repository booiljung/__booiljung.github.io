# Ubuntu 18.04 에 라데온 드라이버 설치

[라데온 다운로드 페이지](https://www.amd.com/en/support)에서 해당되는 드라이버를 다운로드 한다.

다운로드된 파일의 압축을 해제한다.

```sh
tar -xJvf amdgpu-pro_<version or model>.tar.xz
```

압축 해제된 폴더에서

```sh
cd amdgpu-pro_<version or model>
```

설치 스크립트를 실행한다.

```sh
 ./amdgpu-pro-install -y
```

## 참조

- [How to Install The Latest AMD Radeon Drivers on Ubuntu 18.04 Bionic Beaver Linux](https://linuxconfig.org/how-to-install-the-latest-amd-radeon-drivers-on-ubuntu-18-04-bionic-beaver-linux)


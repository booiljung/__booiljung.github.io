[Up](index.md)

# Ubuntu에 CUDA  툴킷 설치하기

2020년 5월 30일

CUDA 툴킷을 설치하기 전에 먼저 [nvidia  드라이버가 설치](./installation_of_nvidia_on_ubuntu.md)되어 있어야 합니다.

CUDA  툴킷 설치 방법은 드라이버와 달리 CUDA 툴킷은 설치가 쉽습니다.

```sh
sudo apt install nvidia-cuda-toolkit
```

버전에 따라 설치 폴더가 다릅니다. 설치가 되었는지  컴파일러 버전을 확인을 해볼 수 있습니다. 

 ```sh
nvcc --version
 ```

설치 위치가 궁금하다면 `type -a`으로 확인 할 수 있습니다.

```sh
type -a nvcc
```

CUDA  드라이버 버전 확인은 

```sh
nvidia-smi
```

로 확인 할 수 있습니다.

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 430.64       Driver Version: 430.64       CUDA Version: 10.1     |
|-------------------------------+----------------------+----------------------+
```

쉽죠?
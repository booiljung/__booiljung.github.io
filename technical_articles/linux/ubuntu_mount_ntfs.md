[Up](./index.md)

# Ubuntu 에서 NTFS 디스크 마운트

우분투에 기본 설치 되어 있는 NTFS 드라이버는 읽기 전용이며, 읽기 전용으로 마운트 됩니다. 읽고 쓰기 마운트가 되게하려면 `ntfs-3g`를 설치해야 합니다.

```
sudo apt install ntfs-3g
```

디스크 포맷이나 디스크의 UUID를 확인하기 위해 `gpartd`도 필요 합니다.

```
sudo apt install gpartd
```



`gpartd`에서 해당 디스크를 선택하고 포맷을 합니다.


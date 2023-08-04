# VirtualBox의 게스트 우분투에서 호스트 우분투의 공유 폴더 액세스

호스트 컴퓨터에 우분투가 설치되어 있고, VM의 게스트 우분투에서 호스트의 공유 파일 액세스 방법

1. VirutalBox 인스턴스 설정에서 Auto-mout, Make Permanent를 활성화 하여 공유 폴더를 설정한다.

2. 게스트 우분투에 VirtualBox Guest Additions 을 설치한다.

3. 수퍼유저 권한으로 `/etc/group` 파일을 편집하여 아래를 수정하거나

   ```
   vboxsf:x:999:<username>
   ```

   또는

   ```sh
   sudo adduser $USER vboxsf
   ```

   를 한다.

4. 링크를 만든다.

   ```sh
   mkdir <대상 경로>
   ln -s <원본 경로> <대상 경로>
   ```

5. 재부팅을 한다.

6. 나중에 링크 삭제시

   ```sh
   unlink <대상 경로>
   ```

## 참조

- https://askubuntu.com/questions/456400/why-cant-i-access-a-shared-folder-from-within-my-virtualbox-machine
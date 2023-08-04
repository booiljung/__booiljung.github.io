# VirtualBox의 게스트 우분투에서 호스트 우분투의 공유 폴더 액세스

호스트 컴퓨터에 우분투가 설치되어 있고, VM의 게스트 우분투에서 호스트의 공유 파일 액세스 방법

1. VirtualBox Guest Additions 을 설치한다.

2. 공유 폴더 액세스 한다.

   ```sh
   sudo usermod -aG vboxsf <username>
   ```

3. 링크를 만든다.

   ```sh
   mkdir <대상 경로>
   ln -s <원본 경로> <대상 경로>
   ```

4. Logout 그리고 로그인 한다.

5. 나중에 링크 삭제시

   ```sh
   unlink <대상 경로>
   ```

## 참조

- https://askubuntu.com/questions/456400/why-cant-i-access-a-shared-folder-from-within-my-virtualbox-machine
# 우분투 16.04 LTS에서 imwheel로 휠 스크롤 단위 설정하기

제가 스컬프트 마우스를 사용하는데 우분투 16.04에서 휠을 한 스텝 돌리면 한페이지가 스크롤 됩니다. 불편합니다. 우분투 18.04 LTS는 마우스 설정 프로그램에서 휠 스크롤 간격을 지정 할 수 있습니다. 하지만, 우분투 16.04 LTS는 없습니다. 우분투 16.04 LTS에서는 imwheel을 사용하여 설정해 주어야 합니다.

먼저 imwheel을 설치 합니다.

```
sudo apt update
sudo apt install imwheel
```

그리고 사용자 홈 디렉토리에 `~/.imwheelrc` 파일을 작성합니다. 이 파일은 imwheel이 참조하게 됩니다.

```
".*"
None,       Up,     Up,     3
None,       Down,   Down,   3
```

여기서 `.*` 애플리케이션 이름입니다. 예를 들어 크롬의 스크롤 간격을 지정하고 싶으면 아래처럼 합니다.

```
".*-chrome*"
None,       Up,     Up,     3
None,       Down,   Down,   3
```

작성이 완료되었으면 저장하고

```
imwheel kill
```

를 합니다.

다른 마우스에서는 잘 될지도 모르겠는데, 스컬프트에서는 되지 않습니다. 스컬프트의 경우 전원을 완전히 끈 후 재부팅을 하는 콜드 부팅을 해주어야 합니다.

- imwheel의 설정을 변경한 경우.
- 동일 기계의 윈도우에서 사용하다가 우분투 16.04 LTS를 사용하는 경우.

원인을 찾아 보지는 않았지만, 마우스 내부의 파라미터가 여러개가 있고 서로 다른 값을 변경하는 것으로 추정 됩니다.

## 참조

- [[Increase mouse wheel scroll speed](https://askubuntu.com/questions/285689/increase-mouse-wheel-scroll-speed)](https://askubuntu.com/questions/285689/increase-mouse-wheel-scroll-speed)
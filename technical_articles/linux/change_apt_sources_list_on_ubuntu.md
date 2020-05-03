# Change apt sources list on Ubuntu

우분투 apt 소스 리스트를 `mirror.kakao.com` 으로 변경하기.

간단히 `/etc/apt/sources.list` 파일을 열어서 `//archive.ubuntu.com/ubuntu`를 검색하여`//mirror.kakao.com/ubuntu`로 치환하면 됩니다. 이렇게 하면 되지만 우분투를 설치 할 때마다 타이핑을 해야 합니다.

리눅스에서 sed 를 사용하여 파일 내의 문자열들을 정규식으로 검색 치환 할 수 있습니다. 다음 명령으로 우분투 저장소를 카카오 미러 서버로 변경 할 수 있으며, 우분투 설치 스크립트를 사용하는 경우 자동화 할 수 있습니다.

```sh
sed -e 's/\/\/archive.ubuntu.com\/ubuntu/\/\/mirror.kakao.com\/ubuntu/g' /etc/apt/sources.list > ~/sources.list && sudo mv ~/sources.list /etc/apt/sources.list
```

파이프라인에 수퍼 유저 권한이 적용되지 않으므로 먼저 유저 홈에 변경된 파일을 생성하고, 다시 원본 파일을 덥어 쓰는 방법을 사용합니다.


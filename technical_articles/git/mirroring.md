# Git Mirroing

로컬의 리모트 저장소 확인

```sh
git remote -v
```

리모트 저장소를 upstream 저장소로 추가

```sh
git remote add upstream https://github.com/mavlink/qgroundcontrol.git
```

upstream 리모트 저장소 추가 확인

```sh
git remote -v
```

upstream 저장소 변경내역 가져오기

```sh
git fetch upstream
```

master로 체크아웃

```sh
git checkout master
```

upstream의 마스트와 머지하기

```sh
git merge upstream/master
```



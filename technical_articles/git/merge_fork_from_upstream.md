# 원본 저장소의 변경 사항을 포크 저장소에 반영하기

원본 저장소를 포크하여 저장소를 만들고 수정해 나가는 경우가 있다.

관례적으로,

원본 저장소를 upstream이라하고,

포크 저장소를 origin이라고 한다.



원본 저장소가 원격 저장소로 등록 되어 있는지 확인 한다.

```
git remote -v
```



없으면 원본 저장소를 upstream으로 추가 한다.

```
git remote add upstream <저장소 URL>
```



원본 저장소 변경 사항을 로컬로 fetch 한다.

```
git fetch upstream <원본원격브랜치>
```



가져온 패치를 머지 한다.

```
git checkout <포크브랜치>
git merge upstream/<원본원격브랜치>
```



머지한 내용을 포크 저장소에 올린다.

```
git push origin <포크브랜치>
```


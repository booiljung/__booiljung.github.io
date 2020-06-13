# Git Cheat Sheet

## 전역 설정

| 명령                                                         | 설명                                       |
| ------------------------------------------------------------ | ------------------------------------------ |
| `git config --global user.name "username"`                   | git 전역 이름 변경                         |
| `git config --global user.email "useremail"`                 | git 전역 이메일 변경                       |
| `git config --global --list`                                 | 전역 설정값 조회                           |
| `git config --global credential.helper 'cache --timeout=<number>'` | 원격 저장소 비밀번호 `<number>`초동안 캐시 |

## 로컬 설정

| 명령                                 | 설명                    |
| ------------------------------------ | ----------------------- |
| `git config user.name "your name"`   | 로컬 저장소 이름 변경   |
| `git config user.email "your email"` | 로컬 저장소 이메일 변경 |
| `git config --list`                  | 로컬 저장소 설정값 조회 |

## 준비

| 명령                               | 설명                       |
| ---------------------------------- | -------------------------- |
| `git init`                         | 폴더 준비                  |
| `git clone <URL>`                  | 저장소 복제                |
| `git clone --depth <number> <URL>` | `<number>`개의 커밋만 복제 |

## 상태

| 명령            | 설명             |
| --------------- | ---------------- |
| `git status`    | 상태 보기        |
| `git status -s` | 자세히 상태 보기 |

## 추적

| 명령                                     | 설명                                         |
| ---------------------------------------- | -------------------------------------------- |
| `git add <file>`                         | 변경한 파일 등록                             |
| `git add .`                              | 전체 파일 등록                               |
| `git add -p <file1> <file2> ...`         | 다수의 파일 등록                             |
| `git add -i`                             | 편집기로 파일 등록                           |
| `git add -u <path1> <path2> <path3> ...` | 추적되는 파일의 변경 사항을 스테이지 합니다. |

## 파일 삭제 및 이름 변경

| 명령                     | 설명           |
| ------------------------ | -------------- |
| `git rm <file>`          | 파일 삭제      |
| `git mv <file1> <file2>` | 파일 이름 변경 |

## 위치

`<posotion>`은 `<tag>`, `<branch>`, `<commit>` 을 의미함.

## 커밋

| 명령                         | 설명                                       |
| ---------------------------- | ------------------------------------------ |
| `git commit -m "message"`    | message로 커밋                             |
| `git commit -am "message"`   | 파일을 모두 추가하며 message로 커밋        |
| `git commit -C HEAD --amend` | 이전 커밋을 수정하고 커밋 메시지를 재사용. |

## 보기

| 명령                                      | 설명                              |
| ----------------------------------------- | --------------------------------- |
| `git show <position>`                     | 위치 보기                          |
| `git blame <file>`                        | 파일 변경 이력 보기                 |
| `git blame <file> -L <startline,endline>` | 파일 라인 범위에서 변경 이력 보기    |
| `git blame <file> -M`                     | 파일의 줄 단위의 복사, 붙여 넣기, 이동 정보를 표시 |
| `git blame <file> -C`                     |  파일의 줄 단위 이동과 원본 파일 정보를 표시       |

## 로그

| 명령 | 설명 |
| ---- | ---- |
| `git log` | 로그 보기 |
| `git log -p` | 변경 사항을 보여주는 패치와 함께 이력을 표시 |
| `git log -<number>` | 마지막 로그 <number>개 보기 |
| `git log --oneline` | 한줄에 로그 보기 |
| `git log --graph --all` | 모든 로그 출력 |
| `git log --graph --all --more=<number>` | 모든 로그를 페이지당 갯수 출력 |
| `git log --decorate` | 음 |
| `git log --since="6 hours"` | 6개월 동안의 커밋 로그를 표시합니다. |
| `git log --before="2 days"` | 5일 전까지의 커밋 로그만을 표시합니다. |
| `git log <position>..<position>` | 두 지점 사이의 커밋 로그를 표시합니다. |
| `git log --stat` | 각 항목마다 영향 받은 줄의 통계 표시 |
| `git log --name-status` | 커밋할 시점의 파일 상태 표시 |
| `git log -C -p -1 <position>` | 로그에서 복사와 붙여 놓은 정보를 표시 |

## 비교

| 명령 | 설명 |
| ---- | ---- |
| `git diff` | 작업과 인덱스 차이점을 표시 |
| `git diff -cached` | 인덱스와 저장소의 차이점을 표시 |
| `git diff HEAD~` | 이전 커밋과 비교 |
| `git diff <start-position>` | 작업트리와 특정 위치간의 차이점을 표시 |
| `git diff <position1> <position2>` | 두 위치 사이의 차이점을 표시 |
| `git diff --stat <position1> <position2>` | 두 위치 사이의 통계를 표시 |
| `git log --pretty=oneline` | 각 로그의 이력을 한줄씩 표시 |


## 원격 저장소

| 명령 | 설명 |
| ---- | ---- |
| `git remote update` | 원격 저장소 정보 가져오기 |
| `git remote add <remote> <remote url>` | 원격 저장소 등록 |
| `git remote` | 원격저장소 목록 |
| `git remote -v` | 원격저장소 목록 자세히 |
| `git remote show` | 원격저장소 보기 |
| `git remote show <remote>` | 원격저장소 보기 |
| `git remote prune <remote>` | 원격 저장소에서 원격 브랜치를 삭제 |
| `git remote rm <remote>` | 원격 저장소를 제거하고 관련 브랜치도 제거 |


## 이동

| 명령 | 설명 |
| ---- | ---- |
| `git checkout <positon>` | 위치로 이동 |
| `git checkout HEAD <file1> <file2> <file3> ...` | 파일을 되돌림 |
| `git checkout -b <branch>` | `git branch + git checkout` 와 동일 |
| `git checkout -b <remote>/<branch>` | 지정한 원격 브랜치로 이동 |
| `git checkout -b <local branch> <remote>/<branch>` | 지정한 원격 브랜치를 새 이름으로 로컬 브랜치로 가져와서 이동 |
| `git checkout -t <remote>/<branch>` | 지정한 원격 브랜치 가져와서 추적 |
| `git checkout --track <remote>/<branch>` | 지정한 원격 브랜치 가져와서 추적 |
| `git checkout <remote>/<branch>` | 추적하지 않고 브랜치를 가져온다. 수정사항 전송 되지 않음. |
| `git checkout -` | 이전 커밋 혹은 브랜치로 |
| `git checkout <tag>` | 지정한 태그의 커밋으로 이동 |


## 브랜치

| 명령 | 설명 |
| ---- | ---- |
| `git branch` | 브랜치 보기 |
| `git branch -a` | 모든 브랜치 보기 |
| `git branch -v` | 브랜치 보기 |
| `git branch -vv` | 트래킹 브랜치 보기 |
| `git branch -r` | 원격 브랜치 보기 |
| `git branch --remote` | 원격 브랜치 보기 |
| `git branch -l` | 로컬 브랜치 보기 |
| `git branch --local` | 로컬 브랜치 보기 |
| `git branch <branch>` | 브랜치 만들기 |
| `git branch <branch> <position>` | 다른 시작 지점에서 새로운 브랜치를 생성 |
| `git branch -f <branch> <position>` | 기존의 위치에 새로운 브랜치로 덮어 쓰기 |
| `git branch -m <branch1> <branch2>` | 브랜치 이름 변경 |
| `git branch --delete <branch>` | 브랜치 삭제. 현재 브랜치는 삭제할 수 없으므로 먼저 다른 브랜치로 이동 해야 함. |
| `git branch -D <branch>` | 강제 삭제. 변경사항이나 커밋들이 남아 있는 경우. |
| `git branch -u <remote>/<branch>` | 원격 저장소로 트래킹 브랜치 설정 |
| `git branch --set-upstream-to <remote>/<branch>` | 원격 저장소로 트래킹 브랜치 설정 |
| `git checkout -b <branch>` | 작업중인 브랜치에서 새로운 브랜치를 생성하고 체크아웃 |
| `git checkout -m <exist branch> <new branch>` | 브랜치를 옮기거나 브랜치 이름을 변경 |
| `git checkout -M <exist branch> <new branch>` | 새로운 브랜치가 없을 경우 브랜치를 옮기거나 브랜치 이름을 변경 |
| `git merge --no-commit <branch>` | 커밋하지 않고 병합 |

`<branch2>`를 `<branch1>`에 병합.

```
git checkout <branch1>
git merge <branch2>
```

##  원격 브랜치

| 명령 | 설명 |
| ---- | ---- |
| `git push <remote> <branch>` | 브랜치를 리모트에 브랜치에 전송 |
| `git push -u <remote> <local branch>` | 브랜치를 리모트에 브랜치에 전송 |
| `git push -u <remote> <local branch>:<remote branch>` | 로컬 브랜치를 원격 브랜치 이름으로 전송. 로컬과 원격 브랜치 이름을 다르게 함.<br>브랜치 이름이 겹칠때. |
| `git push <remote> --delete <branch>` | 원격 브랜치 삭제 |
| `git push <remote> <local branch>:<remote branch>` | 로컬 저장소의 브랜치를 원격 브랜치에 전송 |
| `git checkout -b <remote>/<branch>` | 지정한 원격 브랜치로 이동 |
| `git checkout -b <local branch> <remote>/<branch>` | 지정한 원격 브랜치를 새 이름으로 로컬 브랜치로 가져와서 이동 |
| `git checkout -t <remote>/<branch>` | 지정한 원격 브랜치 가져와서 추적 |
| git checkout --track <remote>/<branch> | 지정한 원격 브랜치 가져와서 추적 |
| `git checkout <remote>/<branch>` | 추적하지 않고 브랜치를 가져온다. 수정사항 전송 되지 않음. |

## 원격 저장소 가져오기

| 명령 | 설명 |
| ---- | ---- |
| `git pull` | `fetch + merge` 와 동일 |
| `git pull <remote>` | `fetch + merge` 와 동일 |
| `git fetch` | 리모트에서 변경사항 가져오기 |
| `git fetch <remote>` | 원격 저장소에서 커밋들을 작업에 합치지 않고 로컬로 가져 오기 |

## 스태시 (임시 저장)

| 명령 | 설명 |
| ---- | ---- |
| `git stash` | 변경 사항을 임시 저장하고 브랜치 변경 |
| `git stash list` | 스태시 된 목록 보기 |
| `git stash show` | 스태시 내용 비교 |
| `git stash show -p` | 스태시 내용 상세 비교 |
| `git stash --keep-index` | 변경 사항을 임시 저장하고 브랜치 변경 (스테이지 영역 파일 제외) |
| `git stash --include-untracked` | 변경 사항을 임시 저장하고 브랜치 변경 (untracked 파일 포함) |
| `git stash save` | 변경 사항을 임시 저장하고 브랜치 변경 여러개의 stash 생성 |
| `git stash save "message"` | 변경 사항을 임시 저장하고 브랜치 변경 여러개의 stash 생성 메시지로 표시 |
| `git stash branch <bransh>` | 브랜치를 생성후 스태시 적용 |
| `git stash pop` | 임시 저장한 작업코드를 워킹 디렉토리로 가져와서 병함. (스태시 삭제 됨) |
| `git stash apply` | 스태시를 브랜치에 적용 (스태시 남아 있음) |
| `git stash apply --index` | 스태이지 영역까지 포함하여 임시 저장한 작업코드 가져와서 병합 |
| `git stash drop` | 스태시 삭제 |

## 작업 청소

| 명령 | 설명 |
| ---- | ---- |
| `git clean` | 추적되지 않는 파일 삭제 |
| `git clean -n` | 추적되지 않는 파일 삭제 전에 실험 |
| `git clean -d` | 추적되지 않는 파일만 별도로 삭제 |
| `git clean -f` | 추적되지 않는 파일 강제 삭제 |
| `git clean -d -f` | 추적되지 않는 파일만 별도로 강제 삭제 |
| `git clean -x` | .gitignore에 등록된 파일까지 삭제 |

## 작업 취소

협업시에는 사용하지 않는다. 협업시에는 revert를 사용할 것.

| 명령 | 설명 |
| ---- | ---- |
| `git reset --soft HEAD~` | add 로 스테이지 상태를 그대로 두고 되돌림. HEAD의 위치만 이동. |
| `git reset --mixed HEAD~` | 스테이지 상태를 unstage 하여 되돌림. 추가로 add를 해야 커밋 가능. |
| `git reset --mixed HEAD~ <file>` | 파일에 대해서만 |
| `git reset HEAD~` | --mixed와 동일 |
| `git reset HEAD~ <fil>` | 파일에 대해서만 --mixed와 동일 |
| `git reset --hard HEAD~` | 작업 내용 삭제하며 완전히 되돌림 |
| `git reset --merge HEAD~` | 병합 취소 |
| `git reset --hard HEAD && git pull` | 강제로 모두 가져오기 |

## 리버트

기존 커밋을 남겨두고 취소에 대한 새로운 커밋을 만든다.

| 명령 | 설명 |
| ---- | ---- |
| `git revert HEAD` | 현재 작업으로 리버트. |
| `git revert <commit>` | 지정한 커밋으로 리버트. |
| `git revert <commit> .. <commit>` | 여러 커밋으로 리버트. |
| `git revert --mainline <number> <merged commit>` | (병합을) 리버트하고 나서 되돌아갈 커밋 |
| `git revert --mainline 1 <commit>` | (병합을) 리버트하고 나서 되돌아갈 커밋 |

## 태그

| 명령 | 설명 |
| ---- | ---- |
| `git tag --list` | 태그 목록. |
| `git tag -l` | 태그 목록. |
| `git tag -a <tag>` | Annotated 태그 달기 |
| `git tag -d <tag>` | 태그 삭제 |
| `git tag <tag>` | light weight 태그 달기 |
| `git show <tag>` | 태그 자세히 보기 |
| `git tag -a <tag> <commit>` | 특정 커밋에 태그 달기 |

##` |   원격 태그

| 명령 | 설명 |
| ---- | ---- |
| `git push <remote> <tag>` | 태그를 리모트에 전송 |
| `git push <remote> --tags` | 전체 태그를 리모트에 전송. |
| `git push <remote> --delete <tag>` | 원격 태그 삭제 |
| `git push <remote> <local tag>:<remote tag>` | 로컬 저장소의 태그를 리모트 저장소 태그로 등록 |

## 서브모듈

| 명령 | 설명 |
| ---- | ---- |
| `git submodule add <remote url> <folder> | |
| `git add .gitmodules | |

## 병합

다른 브랜치를 가져와서 기준 브랜치에 병합함. 리베이스와 다름에 주의.

| 명령 | 설명 |
| ---- | ---- |
| `git merge` | 머지하기 |
| `git merge <branch>` | 지정한 브랜치와 작업을 머지하기 |
| `git merge <branch> --edit` | 병합하며 커밋 메시지 작성 |
| `git merge <remote>/<branch>` | 리모트/브랜치와 작업을 머지하기 |

## 리베이스

베이스 변경. 공통 베이스를 가지는 브랜치.

기준 브랜치를 다른 브랜치에 붙여서 리베이스 함.

| 명령 | 설명 |
| ---- | ---- |
| `git rebase <base branch>` | 리베이스를 함. |
| `git rebase -i HEAD~<number>` | 마지막 커밋 <number>개 묶기  |

`<branch1>`을 `<branch2>`에 리베이스

```
git checkout <branch1>
git rebase <branch2>
```

## merge 및 push에서 충돌 해결


```
git merge --abort # 병합 취소
git fetch
--- 파일 편집 ---
git merge
git branch --merged # 병합한 브랜치 보기
git branch --no-merged # 병합하지 않은 브랜치 보기
```

## rebase에서 충돌 해결


```
git rebase <base branch>
--- 충돌 발생 ---
--- 파일 편집 ---
git add <file>
git rebase --continue # 리베이스 계속
git rebase --abort # 리베이스 중단
```

## 브랜치 파일

```
ls .git/refs/` | 원격 브랜치 정보
```

## 위치 참조

| 명령 | 설명 |
| ---- | ---- |
| `HEAD` | 각 브랜치의 마지막 커밋 |
| `AHEAD` | push 되지 않은 로컬 커밋 |
| `BHEAD` | fetch 되지 않은 원격 커밋 |
| `HEAD~1` | 헤드 이전 커밋 |
| `HEAD~2` | 헤드 이전 이전 커밋 |
| `HEAD~3` | 헤드 이전 이전 이전 커밋 |
| `HEAD^` | 헤드 이전 커밋 |
| `HEAD^^` | 헤드 이전 이전 커밋 |
| `HEAD^^^` | 헤드 이전 이전 이전 커밋 |

## 충돌방지 반복 패턴

```
git pull
-- 파일 변경 ---
git commit -am "message"
git pull
git push
```

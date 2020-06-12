git init # 폴더 준비
git clone <URL> # 저장소 복제
git status # 상태 보기
git status -s # 자세히 상태 보기
git add <file> # 변경한 파일 등록
git add . # 전체 파일 등록
git rm <file> # 파일 삭제
git mv <file1> <file2> # 파일 이름 변경
git show <commit> # 커밋 보기
git blame <file> # 파일 변경 이력 보기
git blame <file> -L <startline,endline> # 파일 라인 범위에서 변경 이력 보기
git blame <file> -M # 파일 복사 이동 이력 보기
git blame <file> -C # 다른 파일에서 복사 이동 이력 보기
git log # 로그 보기
git log -<number> # 마지막 로그 <number>개 보기
git log --oneline # 한줄에 로그 보기
git log --graph --all # 모든 로그 출력
git log --graph --all --more=<number> # 모든 로그를 페이지당 갯수 출력
git log --decorate # 음
git diff HEAD~ # 이전 커밋과 비교.
git remote add origin <remote> # 원격 저장소 등록
git remote # 원격저장소 목록
git remote -v # 원격저장소 목록 자세히
git remote show # 원격저장소 보기
git remote show <remote> # 원격저장소 보기
git checkout <branch|commit> # 브랜치 또는 커밋으로 이동
git checkout -b <branch> # git branch + git checkout
git checkout -b <remote>/<branch> # 지정한 원격 브랜치로 이동
git checkout -b <local branch> <remote>/<branch> # 지정한 원격 브랜치를 로컬 브랜치로 가져와서 이동
git checkout -t <remote>/<branch> # 지정한 원격 브랜치 가져와서 추적
git checkout --track <remote>/<branch> # 지정한 원격 브랜치 가져와서 추적
git checkout - # 이전 커밋 혹은 브랜치로
git checkout <tag> # 지정한 태그의 커밋으로 이동
git branch <branch> [commit_id] # 브랜치 만들기
git branch # 브랜치 보기
git branch -a # 모든 브랜치 보기
git branch -v # 브랜치 보기
git branch -vv # 트래킹 브랜치 보기
git branch -r # 원격 브랜치 보기
git branch -l # 로컬 브랜치 보기
git branch -m <branch1> <branch2> # 브랜치 이름 변경
git branch --delete <branch> # 브랜치 삭제. 현재 브랜치는 삭제할 수 없으므로 먼저 다른 브랜치로 이동 해야 함.
git branch -D <branch> # 강제 삭제. 변경사항이나 커밋들이 남아 있는 경우.
git branch -u <remote>/<branch> # 원격 저장소로 트래킹 브랜치 설정
git branch --set-upstream-to <remote>/<branch> # 원격 저장소로 트래킹 브랜치 설정
git commit -m "message" # 메시지로 커밋
git push -u <remote> <branch> # 브랜치를 리모트에 브랜치에 전송
git push -u <remote> <local branch>:<remote branch> # 로컬 브랜치를 원격 브랜치 이름으로 전송. 로컬과 원격 브랜치 이름을 다르게 함. 브랜치 이름이 겹칠때.
git push <remote> --delete <branch> # 원격 브랜치 삭제
git pull # fetch + merge
git fetch # 리모트에서 변경사항 가져오기
# 임시 저장 (스태시)
git stash # 변경 사항을 임시 저장하고 브랜치 변경
git stash list # 스태시 된 목록 보기
git stash show # 스태시 내용 비교
git stash show -p # 스태시 내용 상세 비교
git stash --keep-index # 변경 사항을 임시 저장하고 브랜치 변경 (스테이지 영역 파일 제외)
git stash --include-untracked # 변경 사항을 임시 저장하고 브랜치 변경 (untracked 파일 포함)
git stash save # 변경 사항을 임시 저장하고 브랜치 변경 여러개의 stash 생성
git stash save "message" # 변경 사항을 임시 저장하고 브랜치 변경 여러개의 stash 생성 메시지로 표시
git stash branch <bransh> # 브랜치를 생성후 스태시 적용
git stash pop # 임시 저장한 작업코드를 워킹 디렉토리로 가져와서 병함. (스태시 삭제 됨)
git stash apply # 스태시를 브랜치에 적용 (스태시 남아 있음)
git stash apply --index # 스태이지 영역까지 포함하여 임시 저장한 작업코드 가져와서 병합
git stash drop # 스태시 삭제
# 작업 청소
git clean # 추적되지 않는 파일 삭제
git clean -n # 추적되지 않는 파일 삭제 전에 실험
git clean -d # 추적되지 않는 파일만 별도로 삭제
git clean -f # 추적되지 않는 파일 강제 삭제
git clean -d -f # 추적되지 않는 파일만 별도로 강제 삭제
git clean -x # .gitignore에 등록된 파일까지 삭제
# 작업 취소 (협업시에는 사용하지 않는다 revert를 사용할 것)
git reset --soft HEAD~ # add 로 스테이지 상태를 그대로 두고 되돌림. HEAD의 위치만 이동.
git reset --mixed HEAD~ # 스테이지 상태를 unstage 하여 되돌림. 추가로 add를 해야 커밋 가능.
git reset --mixed HEAD~ <file> # 파일에 대해서만
git reset HEAD~ # --mixed와 동일
git reset HEAD~ <fil> # 파일에 대해서만 --mixed와 동일
git reset --hard HEAD~ # 작업 내용 삭제하며 완전히 되돌림
git reset --merge HEAD~ # 병합 취소
git reset --hard HEAD && git pull # 강제로 모두 가져오기
# 리버트 기존 커밋을 남겨두고 취소에 대한 새로운 커밋을 만든다.
git revert HEAD # 현재 작업으로 리버트.
git revert <commit> # 지정한 커밋으로 리버트.
git revert <commit> .. <commit> # 여러 커밋으로 리버트.
git revert --mainline <number> <merged commit> # (병합을) 리버트하고 나서 되돌아갈 커밋
git revert --mainline 1 <commit> # (병합을) 리버트하고 나서 되돌아갈 커밋
# 태그
git tag --list # 태그 목록.
git tag -l # 태그 목록.
git tag -a <tag> # Annotated 태그 달기
git tag -d <tag> # 태그 삭제
git tag <tag> # light weight 태그 달기
git show <tag> # 태그 자세히 보기
git tag -a <tag> <commit> # 특정 커밋에 태그 달기
git push <remote> <tag> # 태그를 리모트에 전송
git push <remote> --tags # 전체 태그를 리모트에 전송.
git push <remote> --delete <tag> # 원격 태그 삭제
git push <remote> <local tag>:<remote tag> # 로컬 저장소의 태그를 리모트 저장소 태그로 등록
# 서브모듈
git submodule add <remote url> <folder>
git add .gitmodules

# 설정
git config --global user.name "username" # git 이름 변경
git config --global user.email "email" # git 이메일 변경

# 병합. 다른 브랜치를 가져와서 기준 브랜치에 병합함. 리베이스와 다름에 주의
git merge # 머지하기
git merge <branch> # 지정한 브랜치와 작업을 머지하기
git merge <branch> --edit # 병합하며 커밋 메시지 작성
git merge <remote>/<branch> # 리모트/브랜치와 작업을 머지하기

# 리베이스. 베이스 변경. 공통 베이스를 가지는 브랜치.
# 기준 브랜치를 다른 브랜치에 붙여서 리베이스 함.
git rebase <base branch> # 리베이스를 함.
git rebase -i HEAD~<number> # 마지막 커밋 <number>개 묶기 

# merge 및 push에서 충돌 해결
git merge --abort # 병합 취소
git fetch
파일 편집
git merge
git branch --merged # 병합한 브랜치 보기
git branch --no-merged # 병합하지 않은 브랜치 보기

# rebase에서 충돌 해결
git rebase <base branch>
충돌 발생!
파일 편집
git rebase --continue # 리베이스 계속
git rebase --abort # 리베이스 중단

# 브랜치 파일

ls .git/refs/ # 원격 브랜치 정보

# 태그

HEAD # 각 브랜치의 마지막 커밋
AHEAD # push 되지 않은 로컬 커밋
BHEAD # fetch 되지 않은 원격 커밋
HEAD~1 # 헤드 이전 커밋
HEAD~2 # 헤드 이전 이전 커밋
HEAD~3 # 헤드 이전 이전 이전 커밋
HEAD^ # 헤드 이전 커밋
HEAD^^ # 헤드 이전 이전 커밋
HEAD^^^ # 헤드 이전 이전 이전 커밋


# 충돌방지 반복 패턴
git pull
-- 파일 변경 ---
git commit -am "message"
git pull
git push






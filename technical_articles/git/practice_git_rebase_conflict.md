[Up](index.md)

# git rebase 충돌 해결 연습

이 글은 [git 기본 명령](git_cheatsheet.md)을 숙지한 개발자를 대상으로 합니다.

github에서 gittest를 만듭니다.

## 스텝1

터미널1을 열고

```
# 1
git clone https://github.com/booiljung/gittest gittest1
cd gittest1

```

터미널2를 열고

```
# 2
git clone https://github.com/booiljung/gittest gittest2
cd gittest2

```

## 스텝2

터미널1에서 a.txt 생성하여 리모트에 올립니다.

```
# 1
echo "" > a.txt
git add a.txt
git commit -m "initial commit"
git push

```

터미널2에서 리모트를 받습니다.

```
# 2
git pull

```

터미널2에서 conflict 브랜치를 만들어 a.txt에 `conflict` 를 내용으로 하여 리모트에 올립니다.

```
# 2
git branch conflict
git checkout conflict
echo "conflict" > a.txt
git add a.txt
git commit -m "conflict to a.txt"
git push -u origin conflict

```

## 스텝3

터미널 1에서 새 conflict 브랜치를 받습니다.

```
# 1
git remote update
git checkout -t origin/conflict

```

a.txt가 충돌하도록 master 브랜치에 `master`라고 넣습니다.

```
# 1
git checkout master
echo "master" > a.txt
git add a.txt
git commit -am "master to a.txt"
git push

```

터미널 1에서 conflict를 master에 리베이스를 하겠습니다.

```
# 1
git checkout conflict
git rebase master

```

충돌이 탐지됩니다.

```
First, rewinding head to replay your work on top of it...
Applying: master to a.txt
Using index info to reconstruct a base tree...
M	a.txt
Falling back to patching base and 3-way merge...
Auto-merging a.txt
CONFLICT (content): Merge conflict in a.txt
error: Failed to merge in the changes.
Patch failed at 0001 master to a.txt
Use 'git am --show-current-patch' to see the failed patch

Resolve all conflicts manually, mark them as resolved with
"git add/rm <conflicted_files>", then run "git rebase --continue".
You can instead skip this commit: run "git rebase --skip".
To abort and get back to the state before "git rebase", run "git rebase --abort".

```

a.txt를 보면

```
<<<<<<< HEAD
conflict
=======
master
>>>>>>> master to a.txt
```

a.txt를 수정 후 스테이지하고 리베이스를 계속 합니다.

```
# 1
echo "master & conflict" > a.txt
git add a.txt
git rebase --continue

```
이제 병합하고 전송하여 마무리 합니다.
```
# 1
git checkout master
git merge conflict
git push

```
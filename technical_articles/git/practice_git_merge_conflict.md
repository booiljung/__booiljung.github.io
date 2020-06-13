[Up](index.md)

# git merge 충돌 해결 연습

이 글은 [git 기본 명령](git_cheatsheet.md)을 숙지한 개발자를 대상으로 합니다.

github에서 gittest를 만듭니다.

## 스텝1

터미널1을 열어서

```
# 1
git clone https://github.com/booiljung/gittest gittest1
cd gittest1
#
```

터미널2를 열어서

```
# 2
git clone https://github.com/booiljung/gittest gittest2
cd gittest2
#
```

## 스텝2

터미널1에서

```
# 1
echo "" > a.txt
git add a.txt
git commit -m "initial commit"
git push
#
```

터미널2에서 

```
# 2
git pull
echo "gittest2" > a.txt
git add a.txt
git commit -m "gittest2 to a.txt"
git push
#
```

## 스텝3

터미널 1에서

```
# 1
echo "gittest1" > a.txt
git add a.txt
git commit -am "gittest1 to a.txt"
git push
# 
```

충돌이 탐지됩니다.

```
To https://github.com/booiljung/gittest
 ! [rejected]        master -> master (fetch first)
error: failed to push some refs to 'https://github.com/booiljung/gittest'
hint: Updates were rejected because the remote contains work that you do
hint: not have locally. This is usually caused by another repository pushing
hint: to the same ref. You may want to first integrate the remote changes
hint: (e.g., 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
```

가져와서 병합을 합니다.

```
# 1
git fetch
git merge
#
```
충돌이 표시 됩니다.

```
Auto-merging a.txt
CONFLICT (content): Merge conflict in a.txt
Automatic merge failed; fix conflicts and then commit the result.
```

a.txt 내용은 아래와 같습니다.

```
<<<<<<< HEAD
gittest1
=======
gittest2
>>>>>>> refs/remotes/origin/master
```

편집하여 수정하고 병합을 마무리 합니다.

```
# 1
echo "gittest1 & gittest2" > a.txt
git add a.txt
git merge --continue
#
```

편집창이 열려서 기록을 요구할 것입니다.

```
Merge remote-tracking branch 'refs/remotes/origin/master'

# Conflicts:
#       a.txt
#
# It looks like you may be committing a merge.
# If this is not correct, please remove the file
#       .git/MERGE_HEAD
# and try again.


# Please enter the commit message for your changes. Lines starting
# with '#' will be ignored, and an empty message aborts the commit.
#
# On branch master
# Your branch and 'origin/master' have diverged,
# and have 1 and 1 different commits each, respectively.
#   (use "git pull" to merge the remote branch into yours)
#
# All conflicts fixed but you are still merging.
#
# Changes to be committed:
#       modified:   a.txt
#
```

적당한 메시지로 변경하고 저장 합니다.

그리고 마무리

```
# 1
git push
#
```

## 스텝5

터미널2에서 확인해 봅니다.

```
# 2
git pull
#
```




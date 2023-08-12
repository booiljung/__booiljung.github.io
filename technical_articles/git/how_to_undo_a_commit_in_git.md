[Git Remove Last Commit – How to Undo a Commit in Git](https://www.freecodecamp.org/news/git-remove-last-commit-how-to-undo-a-commit-in-git/)

#  Git 마지막 커밋 제거 - Git에서 커밋을 실행 취소하는 방법

Git은 강력한 도구이자 가장 널리 사용되는 버전 관리 시스템입니다.  개발자와 기술팀이 프로젝트에서 협업하고 함께 작업하는 방식이기도 합니다.

하지만 실수로 파일을 커밋했다가 해당 파일에 오류가 있어서 커밋하지 말았어야 했다는 사실을 알게 되면 어떻게 될까요?

Git을 사용하면 실수를 취소하고 프로젝트의 이전 버전으로 돌아갈 수 있으므로 걱정할 필요가 없습니다.

Git의 가장 유용한 기능 중 하나는 시간이 지남에 따라 프로젝트에 변경한 내용을 되돌릴 수 있는 기능입니다.

이 글에서는 Git 리포지토리의 상태에 따라 Git에서 변경 내용을 실행 취소하는 방법을 알아보세요.

여기서 다룰 내용은 다음과 같습니다:

1. [로컬의 스테이징되지 않은 변경 내용을 실행 취소하는 방법](https://www.freecodecamp.org/news/git-remove-last-commit-how-to-undo-a-commit-in-git/#unstaged)
2. [로컬 스테이지 변경을 실행 취소하는 방법](https://www.freecodecamp.org/news/git-remove-last-commit-how-to-undo-a-commit-in-git/#staged)
3. [로컬 커밋 변경 취소 방법](https://www.freecodecamp.org/news/git-remove-last-commit-how-to-undo-a-commit-in-git/#local-committed)
4. [공개 커밋 변경 취소 방법](https://www.freecodecamp.org/news/git-remove-last-commit-how-to-undo-a-commit-in-git/#public-committed)

## Git에서 로컬의 스테이징되지 않은 변경 내용을 실행 취소하는 방법 

로컬 컴퓨터에서 작업하고 있다고 가정해 봅시다.  로컬에서 파일을 일부 변경하고 저장했지만 이를 취소하고 싶습니다.

이러한 변경 사항을 아직 스테이징하지 않은 경우 `git add` 명령을 사용하지 않은 것입니다.

이 경우 `git restore` 명령을 사용해야 합니다.

구체적으로 `git restore` 명령은 다음과 같습니다:

```git
git restore filename
```

예를 들어 `README.md` 파일이 있는데 실수로 삭제하고 싶은 텍스트를 작성하여 저장했다고 가정해 하겠습니다.

먼저 `git status` 명령을 사용하여 Git 리포지토리의 상태를 볼 수 있습니다.

이 명령은 파일이 스테이징되지 않았는지(아직 `git add`를 사용하지 않았음을 의미) 확인하고 실행 취소할 수 있는 파일을 볼 수 있게 해줍니다:

```git
On branch main
Your branch is up to date with 'origin/main'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   README.md

no changes added to commit (use "git add" and/or "git commit -a")
```

README.md 파일에서 변경 사항을 실행 취소하는 방법은 다음과 같습니다:

```
git restore README.md
```

그런 다음 `git status`를 다시 사용하여 리포지토리의 상태를 확인할 수 있습니다:

```git
On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean
```

이제 가장 최근에 변경한 내용을 성공적으로 삭제하고 프로젝트의 마지막 커밋 버전으로 되돌렸습니다.

## Git에서 로컬 스테이징된 변경 내용을 실행 취소하는 방법 

파일은 `git add` 명령을 사용했을 때 스테이징됩니다.

예를 들어 `README.md` 파일을 로컬에서 변경하고 `git add` 명령을 사용하여 변경 사항을 스테이징 한 후 텍스트에 몇 가지 실수가 있다는 것을 깨달았다고 가정해 보겠습니다.

먼저 `git status`를 실행하여 파일을 스테이징했는지 확인합니다(`git add`를 사용했음을 의미):

```git
On branch main
Your branch is up to date with 'origin/main'.

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
	modified:   README.md
```

`git status`의 출력에서 알 수 있듯이, 다음 명령을 사용하여 변경 사항을 취소할 수 있다:

```git
git restore --staged filename
```

이 명령은 스테이징된 파일의 스테이징을 해제하지만 변경 내용은 유지합니다.

다시 `git status`를 실행해 보겠습니다:

```git
On branch main
Your branch is up to date with 'origin/main'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   README.md

no changes added to commit (use "git add" and/or "git commit -a")
```

이제 변경한 내용을 삭제하고 파일을 원래 내용으로 복원하려면

```git
git restore README.md 
```

마지막으로 ``git status``를 실행해 봅시다:

```git
On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean
```

이제 변경한 내용이 사라지고 파일이 현재 커밋된 버전으로 되돌아갑니다.

## Git에서 로컬 커밋한 변경 내용을 취소하는 방법 

파일을 변경하고 `git add` 명령으로 파일을 스테이징한 다음 `git commit` 명령으로 커밋했다고 가정해 봅시다.

이는 커밋이 로컬에만 존재하고 아직 원격 리포지토리로 푸시되지 않았음을 의미합니다.

먼저 `git status`를 사용하여 파일을 커밋했는지 확인합니다:

```git
On branch main
Your branch is ahead of 'origin/main' by 1 commit.
  (use "git push" to publish your local commits)

nothing to commit, working tree clean
```

다음으로, 마지막 로컬 커밋을 되돌리려면 `git log` 명령을 사용합니다:

가장 마지막 커밋에는 커밋 해시(숫자와 문자로 이루어진 긴 문자열)가 있고 마지막에 `(HEAD -> main)`이 있는데, 이 커밋이 되돌리려는 커밋입니다.

두 번째에서 마지막 커밋은 커밋 해시와 마지막에 `(origin/main)`이 있는데, 이 커밋이 유지하려는 커밋이고 원격 리포지토리에 푸시한 커밋입니다.

그런 다음 다음 명령을 사용하여 커밋을 실행 취소합니다:

```
git reset --soft HEAD~
```

이제 다시 `git log`를 사용해보겠습니다.

커밋 해시와 마지막에 `(HEAD -> main, origin/main)`이 표시되어야 합니다.

마지막으로 커밋한 것은 더 이상 리포지토리 기록에 포함되지 않으며 제거되었습니다.

위의 명령은 모든 것을 실수 또는 실수로 커밋하기 전의 파일 버전으로 복원하고 커밋을 한 번 거슬러 올라갑니다.

다시 `git status`를 확인해보자.

```git
On branch main
Your branch is up to date with 'origin/main'.

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
	modified:   README.md
```

`git reset --soft HEAD~` 명령은 최신 커밋을 취소하지만 변경한 내용은 그대로 유지한다는 점을 기억하자.

## Git에서 공개 커밋한 변경 내용을 취소하는 방법 

파일을 변경한 후 `git add`로 파일을 스테이징하고, `git command`로 커밋하고, `git push`로 원격 리포지토리에 푸시했는데 애초에 그 파일을 커밋하지 말았어야 했다는 사실을 깨달았다면 어떻게 해야 할까요?

그러면 어떻게 해야 할까요?

먼저 `git status`를 사용하여 git 리포지토리의 상태를 확인합니다:

```git
On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean
```

위 섹션에서 각 커밋에는 긴 숫자와 문자로 이루어진 커밋 해시가 있다는 것을 보았습니다.

커밋 해시의 짧은 버전을 보려면 다음 명령을 사용합니다:

```git
git log --online
```

`git log` 명령으로 어떤 커밋을 되돌리려는지 확인할 수도 있습니다.

최신 커밋의 커밋 해시가 `cc3bbf7`이고, 그 뒤에 `(HEAD -> main, origin/main)`이 있고, 커밋 메시지가 "commit README.md file" 이라고 가정해 보자.

특정 커밋을 실행 취소하려면 다음 명령을 사용한다:

```git
git revert cc3bbf7 --no-edit
```

위의 명령은 새 커밋을 생성하고 해당 파일을 변경하지 않은 것처럼 이전 상태로 되돌리면서 변경 내용을 취소한다.

마지막으로 `git push`를 사용하여 원격 브랜치에 변경 사항을 푸시한다.

이렇게 하면 커밋 메시지가 이전 메시지와 동일하지만 다음과 같이 revert라는 단어가 앞에 표시됩니다
'Revert "commit README.md file"'

커밋 히스토리에는 두 커밋이 따로 표시된다는 점에 유의하세요:

```git
Revert "commit README.md file"
@john-doe
john-doe committed 9 minutes ago

commit README.md file
@john-doe
john-doe committed 16 minutes ago 
```

## 결론

이제 Git에서 변경 내용을 실행 취소하는 방법을 알았습니다.

Git에 대해 자세히 알아보려면 다음 무료 리소스를 확인하세요:

- [초보자를 위한 Git 및 GitHub - 단기 속성 과정](https://www.youtube.com/watch?v=RGOj5yH7evk)
- [전문가를 위한 Git 튜토리얼 - Git으로 버전 관리를 마스터하기 위한 도구 및 개념](https://www.youtube.com/watch?v=Uszj_k0DGsg)
- [고급 Git 튜토리얼 - 대화형 리베이스, 체리피킹, 리플로그, 서브모듈 등](https://www.youtube.com/watch?v=qsTthZi23VE)

읽어주셔서 감사드리며 행복한 코딩 되세요 :)

## 참조

- [Git Remove Last Commit – How to Undo a Commit in Git](https://www.freecodecamp.org/news/git-remove-last-commit-how-to-undo-a-commit-in-git/)
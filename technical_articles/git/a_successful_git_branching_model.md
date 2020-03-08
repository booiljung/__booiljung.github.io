# 성공적인 Git 브랜치 모델

원문: [A successful Git branching model](https://nvie.com/posts/a-successful-git-branching-model/?fbclid=IwAR08miCyiCTl_LmWBQ772vj83GqpwmbEZsCXMCDCkwPi2yZm6VxgEI8nE5o)

By [Vicent Dressen](https://nvie.com/about/)

---

역자의 말씀:

셸 이나 git 등의 명령이나 이름에 해당하는 단어는 한국어로 번역하지 않고 한글로 적었습니다. 예를 들어 merge는 git의 명령에 있으므로 "병합"으로 번역하지 않고 "머지"로 적었습니다.

---

2020년 3월 5일 성찰의 기록 (Note of reflection, March 5, 2020)

이 모델은 10년 전인 2010년에 창안되었으며 Git 자체가 등장한지 얼마되지 않은 시점입니다. 그 10년 동안, 많은 소프트웨어 팀에서 git-flow (이 글에서 제시한 브랜칭 모델)는 사람들이 표준으로 취급하기 시작한 시점까지 매우 인기를 얻었습니다. 불행하게도 교리 또는 만병 통치약으로도요.

그 10 년 동안 Git 자체는 전 세계를 뒤흔들었고 Git으로 개발중인 가장 인기있는 소프트웨어 유형은 적어도 내 필터 버블에서 웹 응용 프로그램으로 더 많이 옮겨 가고 있습니다. 웹 응용 프로그램은 일반적으로 지속적으로 제공되며 롤백되지 않으며 야생(wild)에서 실행되는 여러 버전의 소프트웨어를 지원할 필요가 없습니다.

이것은 10년 전 블로그 게시물을 작성할 때 염두에 두었던 소프트웨어 부류(class)가 아닙니다. 팀에서 소프트웨어를 지속적으로 제공하는 경우 팀에 git-flow를 추가하는 대신 GitHub flow과 같은 훨씬 간단한 워크플로를 사용하는 것이 좋습니다.

그러나, 지난 10년간 명시적으로 버전이 지정된 소프트웨어를 빌드하거나 여러 버전의 소프트웨어를 야생으로 지원해야하는 경우 git-flow는 여전히 사람들에게 적합했던 것처럼 팀에 적합 할 수 있습니다. 이 경우 계속 읽어주십시오.

 결론적으로 만병 통치약은 존재하지 않는다는 것을 항상 기억해야합니다. 자신의 상황을 고려해야 합니다. 미워하지 마세요. 스스로 결정하시기 바랍니다.

이 글에서는 약 1년 전에 내 (직장 및 개인 프로젝트) 프로젝트 중 일부에 대해 소개한 개발 모델을 제시하고 있으며 이는 매우 성공적인 것으로 판명되었습니다. 나는 한동안 그것에 대해 글을 쓰려고했지만 지금까지 그렇게 할 시간을 찾지 못했습니다. 프로젝트의 세부 사항에 대해서는 이야기하지 않고 분기 전략 및 릴리스 관리에 대해서만 이야기하겠습니다.

![img](a_successful_git_branching_model.assets/git-model@2x.png)

## 왜 Git을 사용하는가? (Why git?)

중앙 집중식 소스 코드 제어 시스템과 비교하여 Git의 장단점에 대한 자세한 내용은 이 [웹](http://git.or.cz/gitwiki/GitSvnComparsion)을 참조하세요. 거기에는 많은 불꽃 튀는 전쟁이 진행되고 있습니다. 개발자로서 저는 오늘날 다른 도구보다 Git을 선호합니다. Git은 개발자들이 머지와 브랜치에 대한 생각을 바꾸었습니다. 내가 온 고전적인 CVS/Subversion 세계에서, 머지/브랜치는 항상 약간 무섭고 (“머지 충돌을 조심해라. 너를 물을 테니까!”) 한 번에 한 번만하는 일로 간주되었습니다.

그러나 Git을 사용하면 이러한 작업이 매우 저렴하고 간단하며 일상적인 워크플로의 핵심 부분 중 하나로 간주됩니다. 예를 들어 CVS/Subversion 서적에서 브랜치 및 머지는 이후 장 (고급 사용자 용)에서 먼저 설명하지만 모든 Git 서적에서는 이미 3 장 (기본 사항)에서 다룹니다.

단순성과 반복적인 특성으로 인해 브랜치와 머지는 더 이상 두려워 할 것이 아닙니다. 버전 관리 도구는 다른 것보다 더 많은 브랜치/머지를 지원해야합니다.

도구에 대해 충분히 살펴보면 개발 모델을 살펴 보겠습니다. 여기서 제시 할 모델은 본질적으로 모든 팀 구성원이 관리되는 소프트웨어 개발 프로세스를 수행하기 위해 따라야 하는 일련의 절차에 지나지 않습니다.

## 분산되었지만 중앙에 집중됨 (Decentralized but centralized)

우리가 사용하며 이 브랜칭 모델과 잘 작동하는 리포지토리 설정은 중앙의 "진짜" 저장소를 사용하는 것입니다. 이 저장소는 중앙 저장소로 간주됩니다 (Git이 DVCS이므로 기술 수준의 중앙 저장소와 같은 것은 없습니다). 이 이름은 모든 Git 사용자에게 친숙하기 때문에 이 저장소를 `origin`이라고 합니다.

![img](a_successful_git_branching_model.assets/centr-decentr@2x.png)

각 개발자는 `origin` 에서 pull 과 push를 합니다. 그러나 중앙 집중식 push-pull 관계 외에도 각 개발자는 다른 동료로부터 변경 사항을 가져와 하위 팀을 구성 할 수도 있습니다. 예를 들어, 진행중인 작업을 조기에 시작하기 전에 두 가지 이상의 개발자와 함께 큰 새로운 기능을 사용하는 것이 유용 할 수 있습니다. 위의 그림에는 Alice와 Bob, Alice와 David, Clair와 David의 하위 팀이 있습니다.

기술적으로 이것은 Alice가 Bob이라는 저장소를 가리키는 `bob`이라는 Git remote를 정의한 것 이상을 의미합니다.

## 주요 브랜치들 (The main branches)

핵심적으로, 개발 모델은 기존 모델에 크게 영향을 받습니다. 중앙 저장소에는 수명이 무한한 두 개의 주요 브랜치가 있습니다.

- `master`
- `develop`

![img](a_successful_git_branching_model.assets/main-branches@2x.png)

`origin`의 `master` 브랜치는 모든 Git 사용자에게 친숙해야합니다. `master`브랜치와 병렬로 또 다른 브랜치 인 `develop`가 있습니다.

우리는 `origin/master`를 `HEAD`의 소스 코드가 항상 *production-ready* 상태를 반영하는 주요 지점이라고 생각합니다.

우리는 `origin/develop`를 `HEAD`의 소스 코드가 항상 다음 릴리스에 대한 최신 개발 개발 변경 사항을 반영하는 메인 브랜치라고 생각합니다. 어떤 사람들은 이것을 "integration branch"이라고 부릅니다. 자동 야간 빌드가 시작되는 곳입니다.

`develop` 브랜치의 소스 코드가 안정된 지점에 도달하고 릴리스 될 준비가 되면, 모든 변경 사항은 어떻게 든 `master`로 다시 머지 된 다음 릴리스 번호로 태그되어야 합니다. 이에 대한 자세한 내용은 추후 논의 될 것입니다.

따라서, 변경 사항이 `master`로 다시 머지 될 때마다 *definition*에 따른 새로운 프로덕션 릴리스 입니다. 우리는 이것에 대해 매우 엄격한 경향이 있으므로 이론적으로 Git hook 스크립트를 사용하여 `master`에 대한 커밋이 있을 때마다 소프트웨어를 프로덕션 서버에 자동으로 빌드하고 롤아웃 할 수 있습니다.

## 서포트 브랜치들 (Supporting branches)

`master`와 `develop`  메인 브랜치 외에도, 우리의 개발 모델은 다양한 서포트 브랜치를 사용하여 팀 구성원 간의 병렬 개발을 지원하고 피쳐를 쉽게 추적하며, 프러덕션 릴리스를 준비하고, 실시간 프러덕션 문제를 신속하게 해결하는 데 도움을줍니다. 메인 브랜치들과 달리, 이 브랜치들은 결국 제거되기 때문에 수명이 항상 제한되어 있습니다.

우리가 사용할 수 있는 여러 가지 유형은 다음과 같습니다.

- 피쳐 브랜치
- 릴리즈 브랜치
- 핫픽스 브랜치

이러한 각 브랜치는 특정 목적을 가지고 있으며, 어떤 브랜치가 원래 브랜치 일 수 있고, 어떤 브랜치가 머지 대상이어야하는지에 대한 엄격한 규칙에 구속됩니다. 우리는 그들을 잠시 후에 살펴 볼 것입니다.

이 브랜치들이 기술적 관점에서 “특별”한 것은 아닙니다. 브랜치 유형은 사용 방식에 따라 분류됩니다. 그들은 물론 평범한 오래된 Git 브랜치입니다.

### 피쳐 브랜치들 (Feature branches)

다음에서 브랜치 됨:

- `develop`

그리고 다음으로 다시 머지 됨:

- `develop`

브랜치 명명 규칙:

- `master`, `develop`, `release-*` 또는 `hotfix-*`를 제외한 모든 이름

피처 브랜치 (또는 토픽 브랜치라고도 함)는 향후 또는 먼 향후 릴리스를 위한 새로운 피쳐를 개발하는 데 사용됩니다. 피쳐 개발을 시작할 때 이 피쳐가 통합 될 대상 릴리스는 그 시점에서 잘 알지 못할 수 있습니다. 피쳐 브랜치의 핵심은 피쳐가 개발중인 동안 존재하지만 결국 (다가오는 릴리스에 새로운 기능을 추가하기 위해) `develop`로 다시 머지 되거나 (실험이 실망하는 경우) 폐기됩니다.

피처 브랜치는 일반적으로 `origin`이 아닌 개발자 저장소에만 존재합니다.

#### 피쳐 브랜치 생성하기 (Creating a feature branch)

새로운 피쳐에 대한 작업을 시작할 때 `develop` 브랜치에서 브랜치 하세요:

```
$ git checkout -b myfeature develop
Switched to a new branch "myfeature"
```

#### `develop`에 완성 된 피쳐를 통합 (Incorporating a finished feature on develop )

완성 된 피처는 `develop` 브랜치로 머지되어 다음 릴리스에 추가 할 수 있습니다:

```
$ git checkout develop
Switched to branch 'develop'
$ git merge --no-ff myfeature
Updating ea1b82a..05e9557
(Summary of changes)
$ git branch -d myfeature
Deleted branch myfeature (was 05e9557).
$ git push origin develop
```

`--no-ff` 플래그는 fast-forward를 통해 머지를 수행 할 수 있는 경우에도 머지가 항상 새 커밋 객체를 생성하도록 합니다. 이렇게하면 피쳐 브랜치의 히스토리 존재에 대한 정보가 유실되지 않으며 피쳐를 함께 추가 한 모든 커밋이 함께 그룹화됩니다. 비교하자면:

![img](a_successful_git_branching_model.assets/merge-without-ff@2x.png)

후자의 경우, Git 히스토리에서 어떤 커밋 객체가 함께 기능을 구현했는지 확인할 수 없습니다. 모든 로그 메시지를 수동으로 읽어야 합니다. 전체 피쳐 (즉, 커밋 그룹)를 되돌리는 것은 후자의 상황에서 진정한 두통이지만 `--no-ff` 플래그를 사용하면 쉽게 수행됩니다.

그렇습니다. 커밋 객체를 몇 개 더 만들지만 비용보다 이득이 훨씬 큽니다.

### 브랜치를 릴리즈 하기 (Release branches)

다음에서 분기:

- `develop`

다음으로 다시 머지:

- `develop` and `master`

브랜치 명명 규칙:

- `release-*`

릴리스 브랜치는 새로운 프로덕션 릴리스 준비를 서포트합니다. 그들은 마지막 순간의 i와 t의 교차를 허용합니다. 또한 사소한 버그 수정과 릴리스 (버전 번호, 빌드 날짜 등)를 위한 메타 데이터 준비를 허용합니다. 릴리스 브랜치에서 이 모든 작업을 수행하면 개발 브랜치가 다음 큰 릴리스의 기능을 받을 수 있습니다.

`develop`에서 새 릴리스 브랜치를 분리하는 주요 계기는 개발(거의)이 새 릴리스의 원하는 상태를 반영 할 때입니다. 이 시점에서 개발할 릴리스를 대상으로하는 모든 기능을 머지해야 합니다. 향후 릴리스를 대상으로하는 모든 기능은 릴리스 분기가 분기 될 때까지 기다려야 합니다.

릴리스 브랜치가 시작될 때 곧 출시 될 릴리스에는 이전 버전이 아닌 버전 번호가 할당됩니다. 그때까지 `develop` 브랜치는 "다음 릴리스"에 대한 변경 사항을 반영했지만 릴리스 브랜치가 시작될 때까지 "다음 릴리스"가 결국 0.3 또는 1.0이 될지 확실하지 않습니다. 이 결정은 릴리스 브랜치 시작시 이루어지며 버전 번호 충돌에 대한 프로젝트 규칙에 따라 수행됩니다.

#### 릴리즈 브랜치 생성하기 (Creating a release branch)

릴리스 브랜치는 `develop` 브랜치에서 생성됩니다. 예를 들어 버전 1.1.5가 현재 프로덕션 릴리스이고 큰 릴리스가 있다고 가정합니다. `develop`의 상태는 “다음 릴리스”에 대한 준비가되어 있으며 우리는 이것이 1.1.6 또는 2.0이 아닌 1.2가 될 것이라고 결정했습니다. 따라서 우리는 분기를 해제하고 릴리스 분기에 새로운 버전 번호를 반영하는 이름을 부여합니다.

```
$ git checkout -b release-1.2 develop
Switched to a new branch "release-1.2"
$ ./bump-version.sh 1.2
Files modified successfully, version bumped to 1.2.
$ git commit -a -m "Bumped version number to 1.2"
[release-1.2 74d9424] Bumped version number to 1.2
1 files changed, 1 insertions(+), 1 deletions(-)
```

새 브랜치를 생성하고 전환 한 후 버전 번호가 충돌합니다. 여기서 `bump-version.sh`는 작업 복사본의 일부 파일을 변경하여 새 버전을 반영하는 가상의 셸 스크립트입니다. 물론 이것은 수동 변경 일 수도 있습니다. *일부* 파일이 변경되는 시점입니다. 그런 다음 범프 버전 번호가 커밋됩니다.

이 새로운 브랜치는 릴리스가 확실히 출시 될 때까지 잠시 동안 존재할 수 있습니다. 이 기간 동안 버그 수정이 (개발) 분기가 아닌이 분기에 적용될 수 있습니다. 여기에 큰 새로운 기능을 추가하는 것은 엄격히 금지되어 있습니다. 그것들은 `develop`로 합쳐 져야 하며, 따라서 다음 큰 릴리즈를 기다립니다.

#### 릴리즈 브랜치를 끝내기 (Finishing a release branch)

릴리스 브랜치의 상태가 실제 릴리스가 될 준비가 되면 일부 조치를 수행해야 합니다. 먼저, 릴리스 브랜치는 `master`로 병합됩니다 ( `master`에 대한 모든 커밋은 *definition*에 따른 새로운 릴리즈 입니다. 기억해 두세요). 다음으로, `master`에 대한 커밋에 이 과거 버전을 쉽게 참조 할 수 있도록 태그를 지정해야 합니다. 마지막으로 릴리스 브랜치의 변경 사항은 다시 `develop`로 머지해야하므로 향후 릴리스에도 이러한 버그 수정이 포함됩니다.

Git 에서 첫 두 스텝은:

```
$ git checkout master
Switched to branch 'master'
$ git merge --no-ff release-1.2
Merge made by recursive.
(Summary of changes)
$ git tag -a 1.2
```

이제 릴리스가 완료되었으며 나중에 참조 할 수 있도록 태그가 지정되었습니다.

> 편집: `-s`  또는 `-u`플래그를 사용하여 태그에 암호화 서명을 할 수도 있습니다.

릴리스 브랜치에서 변경된 사항을 유지하려면 다시 `develop`로 머지 해야 합니다:

```
$ git checkout develop
Switched to branch 'develop'
$ git merge --no-ff release-1.2
Merge made by recursive.
(Summary of changes)
```

이 단계는 머지 충돌로 이어질 수 있습니다 (아마도, 버전 번호가 변경되었으므로). 그렇다면 수정하고 커밋하십시오.

이제 더 이상 필요하지 않으므로 릴리스 브랜치가 제거 될 수 있습니다:

```
$ git branch -d release-1.2
Deleted branch release-1.2 (was ff452fe).
```

### 핫픽스 브랜치들 (Hotfix branches) 

다음 브랜치에서 브랜치 됩니다:

- `master`

다음 브랜치로 병합 됩니다:

- `develop` 와 `master`

브랜치 이름 규칙:

- `hotfix-*`

![img](a_successful_git_branching_model.assets/hotfix-branches@2x.png)

핫픽스 브랜치는 계획되지 않았지만 새로운 프로덕션 릴리스를 준비해야 한다는 점에서 릴리스 브랜치와 매우 유사합니다. 라이브 프로덕션 버전의 원하지 않는 상태에서 즉시 행동해야 할 필요성에서 발생합니다. 프로덕션 버전의 중요 버그를 즉시 해결해야하는 경우 프로덕션 버전을 표시하는 마스터 브랜치의 해당 태그에서 핫픽스 브랜치가 브랜치 될 수 있습니다.

본질적으로 팀 구성원 (개발팀)의 작업은 계속할 수 있고, 다른 사람은 빠른 프러덕션 수정을 준비하고 있습니다.

#### 핫픽스 브랜치 만들기 (Creating the hotfix branch )

핫픽스 브랜치는 `master` 브랜치에서 생성 됩니다. 예를 들어, 버전 1.2는 현재 실행중인 현재 프로덕션 릴리스이며 심각한 버그로 인해 문제가 발생한다고 합니다. 그러나 `develop` 브랜치의 변경들은 아직 불안정합니다. 그러면 다음 핫픽스 브랜치를 브랜치하고 문제 해결을 시작할 수 있습니다.

```
$ git checkout -b hotfix-1.2.1 master
Switched to a new branch "hotfix-1.2.1"
$ ./bump-version.sh 1.2.1
Files modified successfully, version bumped to 1.2.1.
$ git commit -a -m "Bumped version number to 1.2.1"
[hotfix-1.2.1 41e61bb] Bumped version number to 1.2.1
1 files changed, 1 insertions(+), 1 deletions(-)
```

브랜치 후 버전 번호를 범핑하는 것을 잊지 마세요!

그런 다음 버그를 수정하고 하나 이상의 개별 커밋에서 수정 사항을 커밋하세요

```
$ git commit -m "Fixed severe production problem"
[hotfix-1.2.1 abbe5d6] Fixed severe production problem
5 files changed, 32 insertions(+), 17 deletions(-)
```

#### 핫픽스 브랜치 끝내기 (Finishing a hotfix branch)

핫픽스가 완료되면 버그 수정이 다음 릴리스에도 포함되도록 보호하기 위해 버그 수정을 다시 `master`로 머지해야 하지만 다시 `develop`으로 머지해야 합니다. 이것은 릴리스 브랜치가 완료되는 방법과 완전히 유사합니다.

먼저 `master`를 업데이트하고 릴리스에 태그를 지정하십시오.

```
$ git checkout master
Switched to branch 'master'
$ git merge --no-ff hotfix-1.2.1
Merge made by recursive.
(Summary of changes)
$ git tag -a 1.2.1
```

> 편집 : 태그에 암호로 서명하기 위해 `-s` 또는 `-u` 플래그를 사용할 수도 있습니다.

다음, `develop`에 버그 수정을 포함하게 합니다:

```
$ git checkout develop
Switched to branch 'develop'
$ git merge --no-ff hotfix-1.2.1
Merge made by recursive.
(Summary of changes)
```

**여기서 규칙에 대한 한 가지 예외는 릴리스 브랜치가 현재 존재하는 경우 핫픽스 변경 사항을 `develop` 대신 해당 릴리스 브랜치로 머지해야한다는 것입니다.** 버그 수정을 릴리스 브랜치로 다시 머지하면 릴리스 브랜치가 완료되면, 버그 수정이 `develop`에도 머지 됩니다. (개발중인 작업에 이 버그 수정이 즉시 필요하고 릴리스 브랜치가 완료 될 때까지 기다릴 수 없는 경우 버그 수정을 이미 `develop`에 통합 할 수 있습니다.)

마지막으로 임시 브랜치를 제거하세요:

```
$ git branch -d hotfix-1.2.1
Deleted branch hotfix-1.2.1 (was abbe5d6)
```

## 요약 (Summary)

이 브랜칭 모델에 새로운 충격은 없지만 이 게시물이 시작된 “큰 그림”은 우리 프로젝트에서 매우 유용한 것으로 판명되었습니다. 그것은 이해하기 쉬운 엘레강스한 멘탈 모델을 형성하고 팀원들이 브랜치 및 릴리스 프로세스에 대한 공유 된 이해에서 개발할 수 있게 합니다.

그림의 고품질 PDF 버전이 [여기](https://nvie.com/files/Git-branching-model.pdf)에 제공됩니다. 언제라도 빠르게 참조 할 수 있도록 벽에 걸어 놓으십시오.

업데이트 : 요청한 사람 : 기본 다이어그램 이미지 (Apple Keynote)의  [gitflow-model.src.key](http://github.com/downloads/nvie/gitflow/Git-branching-model-src.key.zip) 가 있습니다.

[Git-branching-model.pdf 다운로드](https://nvie.com/files/Git-branching-model.pdf)






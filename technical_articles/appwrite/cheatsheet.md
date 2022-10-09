# Appwrite CLI cheatsheet

https://appwrite.io/docs/command-line

CLI 설치:

```
curl -sL https://appwrite.io/cli/install.sh | bash
```

설치 체크:

```
appwrite -v
```

git sparse checkout 활성화

```
git init # 초기화 없으면
git config core.sparsecheckout true
```

Appwrite에 로그인:

```
appwrite login
```

프로젝트 생성 또는 연결:

```
appwrite init project
```

디렉토리에 `appwrite.json`파일이 생성 된다.

CLI에서 데이터베이스 생성을 지원하지 않는다. 콘솔에서 데이터베이스를 생성한다. 다음,

콜렉션 생성:

```
appwrite init collection
```

펑션 준비:

```
appwrite init function
```

여기서 오류가 발생한다.

https://github.com/appwrite/functions-starter 에서 사용하고자 하는 open-runtime 소스코드를 가져 온다.

```
./functions/first/data.dart
./functions/.gitignore
./functions/pubspec.yaml
./appwrite.json
```

펑션 추가 (버그인지 불편하므로 `appwrite.json`으로 하는 방법이 있다.):

```
appwrite functions create --functionId 'unique()' --name first --execute guests --runtime  dart-2.17
```

`--execute`에는 `any, guests, users, user, team, member`중 지정할 수 있다.

성공하면 다음이 출력된다.

```
$id : 632c826eae06d101bd2d
$createdAt : 2022-09-22T15:42:38.713+00:00
$updatedAt : 2022-09-22T15:42:38.713+00:00
execute
[
  "any"
]
name : first
enabled : true
runtime : dart-2.17
deployment : 
vars
[]
events
[]
schedule : 
scheduleNext : 
schedulePrevious : 
timeout : 15
```

배포 생성 (버그인지 잘 동작하지 않는다. 불편하므로 `appwrite.json`으로 하는 방법이 있다.)

```
appwrite functions createDeployment --functionId 632c8c31f2e61d76ebec --entrypoint second.dart --code ./functions/ --activate true
```

가장 마지막에 배포된 빌딩에 성공한 함수가 활성화 된다.

문서에 의하면 `appwrite.json`이 자동으로 작성되고 추적 된다고 하는데 그렇지 않다. 현재 수작업으로 작성하고 있다. 그래서 커맨드 보다 이 문서를 작성하는 방향이 더 낫다.

```
{
    "projectId": "6329d14ebcbafef0d528",
    "projectName": "ilhada",
    "functions": [
        {
            "$id": "first",
            "name": "first",
            "runtime": "dart-2.17",
            "path": "functions",
            "entrypoint": "first.dart",
            "execute": ["any"]
        },
        ...
    ]    ,
    "collections": [
        {
            "$id": "millenium-problems",
            "$permissions": [
                "read(\"any\")",
                "create(\"team:admin\")",
                "update(\"team:admin\")",
                "delete(\"team:admin\")"
            ],
            "databaseId": "ilhada",
            "name": "Millenium Problems",
            "enabled": true,
            "documentSecurity": false,
            "attributes": [],
            "indexes": []
        }
    ]
}
```




[PostGraphile](https://www.graphile.org/)

# 빠른 시작 안내서

이 빠른 시작 안내서에서는 Node 및 PostgreSQL과 같은 필수 구성 요소 설치를 포함하여 첫 번째 PostGraphile 서버의 회전 과정을 안내합니다.

## 목차

- Node 설치
- PostgreSQL 설치
- 데이터베이스 만들기
- PostGraphile 설치

## Node 설치

PostGraphile을 실행하려면 Node.js v8.6 이상을 실행하고 있어야 합니다. 

```
$ node --version
```

을 실행하여 현재 Node 버전을 확인할 수 있습니다. 최신 버전을 실행하고 있다면 이 섹션을 건너뛸 수 있습니다.

Node를 설치하는 데는 여러 가지 방법이 있습니다. 맥OS에 있는 경우 

```
$ brew install node
```

를 통해 homebrew로 설치하는 것을 선호할 수 있습니다. 유닉스 기반 시스템에 있는 경우 nvm 도구를 사용할 수 있습니다. 이러한 오류가 발생하면 OS X 또는 Windows를 사용하는 경우 Node.js 다운로드 페이지에서 설치 프로그램 중 하나를 사용하십시오. LTS라는 이름의 버전을 선택해야 합니다. 리눅스 사용자는 페이지를 스크롤하여 시스템과 함께 작동하는 버전을 찾을 수 있습니다.

설치되면 터미널에서 

```
$ node -v
```

를 실행하여 버전을 확인합니다. 8.6.0 이상이어야 합니다.

## PostgreSQL 설치

연결할 PostgreSQL 데이터베이스가 필요합니다. PostgreSQL 버전 9.6.0 이상이 이미 설치되어 있는 경우 이 섹션을 생략할 수 있습니다.

PostgreSQL을 동일한 컴퓨터에 설치할 필요는 없지만 설치할 경우 더 나은 개발 환경을 가질 수 있습니다. 데이터베이스 연결 지연 시간을 최소화하십시오! 로컬 PostgreSQL 서버를 사용하지 않으면 이를 고려하여 다음 명령을 수정해야 합니다.

macOS에서 실행 중인 경우 PostgreSQL.app을 설치하여 사용하는 것이 좋습니다. 다른 플랫폼에 있는 경우 PostgreSQL 다운로드 페이지로 이동하여 PostgreSQL의 복사본을 선택하십시오. 9.6.0보다 높은 PostgreSQL 버전을 사용하는 것이 좋습니다. 이 요구 사항에 대한 자세한 이유는 문서에서 확인할 수 있습니다.

그런 다음 터미널에서 

````
$ psql postgres:///
````

를 실행하여 로컬에서 PostgreSQL 복사본이 실행되고 있는지 확인합니다 (슬래시 세 개는 의도적인 것입니다. 호스트 이름을 지정하지 않으므로 호스트 이름의 기본값인 localhost, port: 5432).

이와 같은 결과가 반환되면 PostgreSQL이 성공적으로 설치된 것입니다:

```
$ psql "postgres:///"

psql: FATAL:  database "username" does not exist
```

그러나 "Connection refused" 오류가 발생하면 PostgreSQL 서버가 실행 중이 아니거나 연결할 수 없는 상태임을 나타냅니다:

```
$ psql "postgres:///"

psql: could not connect to server: Connection refused
```

PostgreSQL 내의 다른 데이터베이스에 연결하려면 연결 문자열 끝에 데이터베이스 이름을 추가하면 됩니다:

다음과 같이 연결 문자열 끝에 데이터베이스 이름을 추가하면 됩니다.

```
$ psql postgres:///testdb # Connects to the `testdb` database on your local machine
$ psql "postgres://user:password@somehost:2345/somedb"  # Connects to the `somedb` database at `postgres://somehost:2345` using login with `user` and `password`
```

대체 형식(비밀번호 사용 포함)에 대해 자세히 알아보려면 PostgreSQL 연결 문자열에 대한 설명서를 참조하세요.

## 데이터베이스 생성하기

그런 다음, 데이터베이스를 만듭니다. 터미널을 사용하여 이 작업을 수행할 수 있습니다:

```
$ createdb mydb
```

그러면 "mydb"라는 PostgreSQL 데이터베이스가 생성됩니다. PostgreSQL 설명서 사이트에서 이에 대한 자세한 내용을 읽을 수 있습니다. 이제 데이터베이스 URL로 psql을 실행하고 SQL 프롬프트를 얻을 수 있습니다:

```
$ psql "postgres:///mydb"

psql (9.6.*)
Type "help" for help.

=#
```

다음 쿼리를 실행하여 원활하게 작동하는지 확인합니다:

```
=# select 1 + 1 as two;
 two
-----
   2
(1 row)

=#
```

## PostGraphile 설치

npm을 사용하면 PostGraphile을 설치할 수 있습니다:

```
$ npm install -g postgraphile
```

>  참고: PostGraphile(여기서 사용하는 -g 플래그에서 npm까지의)을 글로벌로 설치하는 것은 권장하지 않습니다. 그러나 Node.js를 막 시작하는 경우 글로벌 설치 방법을 사용하는 것이 훨씬 간단합니다. PostGraphile과 함께 플러그인을 사용하기 시작하면 로컬 설치를 사용하는 것이 좋습니다.

PostGraphile을 실행하려면 데이터베이스 이름이 추가된 psql에 사용한 것과 동일한 URL을 사용합니다:

```
# Connect to the `mydb` database within the local PostgreSQL server
$ postgraphile -c "postgres:///mydb"

# Connect to a database that requires SSL/TLS
$ postgraphile -c "postgres://securehost:5432/db?ssl=true"

# Connect to the `somedb` database within the PostgreSQL at somehost port 2345
$ postgraphile -c "postgres://somehost:2345/somedb"
```

다음과 같은 시계 플래그를 사용하여 PostGraphile을 실행할 수도 있습니다:

```
$ postgraphile -c "postgres:///mydb" --watch
```

`--watch` 플래그를 사용하면 PostGraphile은 검사 중인 PostgreSQL 스키마가 변경될 때마다 GraphQL API를 자동으로 업데이트합니다.

PostGraphile을 실행하면 두 개의 엔드포인트가 제공됩니다:

```
‣ GraphQL endpoint served at http://localhost:5000/graphql
‣ GraphiQL endpoint served at http://localhost:5000/graphiql
```

첫 번째 엔드포인트는 응용프로그램과 대화하는 것입니다. 두 번째 엔드포인트는 웹 브라우저에서 열려 시각적 GraphQL 탐색기인 GraphiQL을 통해 데이터베이스에 액세스할 수 있습니다.

잘했어요. PostGraphile을 실행하고 있습니다!


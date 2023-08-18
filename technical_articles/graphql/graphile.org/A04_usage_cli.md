[Postgraphile](https://www.graphile.org/postgraphile/quick-start-guide/)

## Command Line Interface

PostGraphile을 시작하고 실행하는 가장 쉬운 방법은 명령줄 인터페이스를 사용하는 것입니다.

npm을 통해 PostGraphile을 전역에 설치합니다:

```bash
npm install -g postgraphile
```

이렇게 하면 `postgraphile` 명령을 사용할 수 있습니다. 그런 다음 실행할 수 있습니다:

```bash
postgraphile -c postgres:///mydb -s public -a -j
```

여기서 `-c`는 연결 문자열(기본값은 `postgres:///`), `-s`는 스키마 이름(기본값은 "public"), `-a`는 릴레이 지원을 활성화하고 `-j`는 동적 JSON을 활성화합니다.

**macOS 사용자**: 다른 포트에 바인딩하려면 `--port` 옵션을 사용해야 하며, 기본 포트 5000은 이제 macOS Monterey의 새로운 AirPlay 서비스와 충돌합니다.

또한 PostGraphine은 현재 작업 디렉토리에 있는 '.postgraphilerc.js' 파일에서 옵션을 읽습니다.

### Recommended options

PostGraphile이 발전함에 따라 더 많은 피처들이 추가되지만, 이러한 피처는 변경 사항을 위반할 수 있으므로 항상 기본으로 활성화할 수는 없습니다. 보안에 영향을 미칠 수 있는 옵션도 있습니다. 이러한 이유로 많은 기능이 플래그 뒤에 숨어 있습니다. 이 페이지가 매우 길다는 것을 알기 때문에 여러분이 사용할 수 있는 몇 가지 기본 옵션 세트를 포함했습니다:

'@graphile-contrib/pg-simplify-inflector` 플러그인을 설치하는 것이 좋습니다.

#### For Development

```bash
postgraphile \
  --subscriptions \
  --watch \
  --dynamic-json \
  --no-setof-functions-contain-nulls \
  --no-ignore-rbac \
  --show-error-stack=json \
  --extended-errors hint,detail,errcode \
  --append-plugins @graphile-contrib/pg-simplify-inflector \
  --export-schema-graphql schema.graphql \
  --graphiql "/" \
  --enhance-graphiql \
  --allow-explain \
  --enable-query-batching \
  --legacy-relations omit \
  --connection $DATABASE_URL \
  --schema app_public
```

#### For Production

```bash
postgraphile \
  --subscriptions \
  --retry-on-init-fail \
  --dynamic-json \
  --no-setof-functions-contain-nulls \
  --no-ignore-rbac \
  --extended-errors errcode \
  --append-plugins @graphile-contrib/pg-simplify-inflector \
  --disable-graphiql \
  --enable-query-batching \
  --disable-query-log \ # our default logging has performance issues, but do make sure you have a logging system in place!
  --legacy-relations omit \
  --connection $DATABASE_URL \
  --schema app_public
```

### CLI options

GraphQL 서버를 사용자 정의하는 데 사용할 수 있는 더 많은 CLI 옵션이 있습니다(`postgraphile@4.12.3`에서 제공):

- `-V`, `--version`
  버전 번호 출력

- `--plugins <string>`
  로드할 PostGraphile 서버 플러그인 목록(Graphile 엔진 스키마 플러그인이 아님), 있는 경우 *첫 번째* 옵션이어야 합니다.

- `-c`, `--connection <string>`

  PostgreSQL 데이터베이스 이름 또는 커넥션 스트링입니다. 생략된 경우 환경 변수에서 추론됩니다(https://www.postgresql.org/docs/current/static/libpq-envars.html 참조). 예시 'db', 'postgres:///db', 'postgres://user:password@domain:port/db?ssl=true'

- `-C`, `--owner-connection <string>``

  `--connect`와 같지만, 권한이 있는 사용자(예: 시계 설정, 논리 디코딩 등)의 경우 기본값은 `--connection`의 값으로 설정됩니다.

- `-s`, `--schema <string>`

  인트로스펙(introspected) 검사할 Postgres 스키마를 지정합니다. 쉼표를 사용하여 여러 스키마를 정의하기

- `-S`, `--subscriptions`

  Subscription에 GraphQL 지원 활성화(현재 subscription 플러그인이 필요함)

- `--websockets <string>`

  사용할 웹소켓 전송 라이브러리를 선택합니다. 쉼표를 사용하여 여러 개를 정의합니다. `--subscripion` 또는 `-`-live`가 전달된 경우 기본값은 'v0,v1'이고, 그렇지 않으면 '[]'입니다.

- `--websocket-operations <operations>`

  지원되는 GraphQL 웹소켓 전송 작업을 전환합니다: 'subscriptions' 또는 'all'. 기본값은 'subscriptions'입니다.

- `-L`, `--live`

  [실험적] GraphQL `subscrition`을 통해 live-query 지원을 사용하도록 설정합니다 (중첩된 collections/records가 변경될 때마다 업데이트된 페이로드를 전송).  `--subscription`을 의미합니다.

- `-w`, `--watch`

  데이터베이스 스키마가 변경되면 GraphQL 스키마를 자동으로 업데이트합니다 (참고: DB 수퍼유저가 `postgraphile_watch` 스키마를 설치해야 함).

- `-n`, `--host <string>`

  사용할 호스트 이름입니다. 기본값은 `localhost`입니다.

- `-p`, `--port <number>`

  사용할 포트입니다. 기본값은 5000입니다.

- `-m`, `--max-pool-size <number>`

  Postgres 풀에 보관할 최대 클라이언트 수입니다. 기본값은 10입니다.

- `-r`, `--default-role <string>`

  요청이 있을 때 사용할 기본 Postgres 역할을 데이터베이스에 연결하는 데 사용되는 역할을 대체합니다.

- `--retry-on-init-fail`

  초기 스키마를 빌드하는 동안 오류가 발생하면 이 플래그를 사용하면 PostGraphile이 종료하지 않고 exponential backoff 를 사용하여 스키마 빌드를 계속 시도합니다.

- `-j`, `--dynamic-json`

  [권장] GraphQL 입력 및 출력에서 dynamic JSON을 활성화합니다. PostGraphile은 기본적으로 문자열화된 JSON을 사용합니다.

- `-N`, `--no-setof-functions-contain-nulls`

  [권장] `RETURNS SETOF compound_type` 함수 중 결과와 NULL을 혼합하는 함수가 없는 경우 이 기능을 활성화하여 GraphQL 스키마에서 Null 가능 항목을 줄일 수 있습니다.

- `-a`, `--classic-ids`

  클래식 글로벌 ID 필드 이름 사용. 릴레이 1을 지원하려면 필수입니다.

- `-M`, `--disable-default-mutations`

  기본 돌연변이를 비활성화하면 Postgres 함수를 통해서만 돌연변이가 가능합니다.

- `--simple-collections <omit|both|only>`
  "omit" (default) - relay connections only, "only" - simple collections only (no Relay connections), "both" - both

- `--no-ignore-rbac`

  [권장] 가능한 사용자가 사용할 수 없는 필드, 쿼리 및 뮤테이션을 제외하도록 설정합니다 (연결 문자열의 사용자 및 해당 사용자가 될 수 있는 역할에 따라 결정됨).

- `--no-ignore-indexes`

  누락된 인덱스로 인해 액세스 비용이 많이 드는 필터, orderBy 및 관계를 제외하도록 설정합니다.

- `--include-extension-resources`

  기본적으로 확장 프로그램에서 제공되는 테이블과 함수는 제외되며, 이 플래그를 사용하여 포함할 수 있습니다(권장하지 않음).

- `--show-error-stack [json|string]`

  GraphQL 결과 오류에 JavaScript 오류 스택 표시(개발 시 권장)

- `--extended-errors <string>`

  GraphQL 결과에 표시할 쉼표로 구분된 확장 Postgres 오류 필드 목록입니다. 개발 시 권장: 'hint,detail,errorcode.  기본값: none

- `--append-plugins <string>`

  Graphile 엔진 스키마 플러그인 목록에 추가할 쉼표로 구분된 플러그인 목록입니다.

- `--prepend-plugins <string>`
   a comma-separated list of plugins to prepend to the list of Graphile Engine schema plugins

- `--skip-plugins <string>`
   a comma-separated list of Graphile Engine schema plugins to skip

- `--read-cache <path>`
   [experimental] reads cached values from local cache file to improve startup time (you may want to do this in production)

- `--write-cache <path>`
   [experimental] writes computed values to local cache file so startup can be faster (do this during the build phase)

- `--export-schema-json <path>`
   enables exporting the detected schema, in JSON format, to the given  location. The directories must exist already, if the file exists it will be overwritten.

- `--export-schema-graphql <path>`
   enables exporting the detected schema, in GraphQL schema format, to the  given location. The directories must exist already, if the file exists  it will be overwritten.

- `--sort-export`
   lexicographically (alphabetically) sort exported schema for more stable diffing.

- `-X`, `--no-server`
   [experimental] for when you just want to use --write-cache or --export-schema-* and not actually run a server (e.g. CI)

- `-q`, `--graphql <path>`
   the route to mount the GraphQL server on. defaults to `/graphql`

- `-i`, `--graphiql <path>`
   the route to mount the GraphiQL interface on. defaults to `/graphiql`

- `--enhance-graphiql`
   [DEVELOPMENT] opt in to additional GraphiQL functionality (this may  change over time - only intended for use in development; automatically  enables with `subscriptions` and `live`)

- `-b`, `--disable-graphiql`
   disables the GraphiQL interface. overrides the GraphiQL route option

- `-o`, `--cors`
   enable generous CORS settings; disabled by default, if possible use a proxy instead

- `-l`, `--body-size-limit <string>`
   set the maximum size of the HTTP request body that can be parsed  (default 100kB). The size can be given as a human-readable string, such  as '200kB' or '5MB' (case insensitive).

- `--timeout <number>`
   set the timeout value in milliseconds for sockets

- `--cluster-workers <count>`
   [experimental] spawn  workers to increase throughput

- `--enable-query-batching`
   [experimental] enable the server to process multiple GraphQL queries in one request

- `--disable-query-log`
   disable logging queries to console (recommended in production)

- `--allow-explain`
   [EXPERIMENTAL] allows users to use the Explain button in GraphiQL to  view the plan for the SQL that is executed (DO NOT USE IN PRODUCTION)

- `-e`, `--jwt-secret <string>`
   the secret to be used when creating and verifying JWTs. if none is provided auth will be disabled

- `--jwt-verify-algorithms <string>`
   a comma separated list of the names of the allowed jwt token algorithms

- `-A`, `--jwt-verify-audience <string>`
   a comma separated list of JWT audiences that will be accepted; defaults  to 'postgraphile'. To disable audience verification, set to ''.

- `--jwt-verify-clock-tolerance <number>`
   number of seconds to tolerate when checking the nbf and exp claims, to deal with small clock differences among different servers

- `--jwt-verify-id <string>`
   the name of the allowed jwt token id

- `--jwt-verify-ignore-expiration`
   if `true` do not validate the expiration of the token defaults to `false`

- `--jwt-verify-ignore-not-before`
   if `true` do not validate the notBefore of the token defaults to `false`

- `--jwt-verify-issuer <string>`
   a comma separated list of the names of the allowed jwt token issuer

- `--jwt-verify-subject <string>`
   the name of the allowed jwt token subject

- `--jwt-role <string>`
   a comma seperated list of strings that create a path in the jwt from  which to extract the postgres role. if none is provided it will use the  key `role` on the root of the jwt.

- `-t`, `--jwt-token-identifier <identifier>`
   the Postgres identifier for a composite type that will be used to create JWT tokens

- `--token <identifier>`
   [DEPRECATED] Use --jwt-token-identifier instead. This option will be removed in v5.

- `--secret <string>`
   [DEPRECATED] Use --jwt-secret instead. This option will be removed in v5.

- `--jwt-audiences <string>`
   [DEPRECATED] Use --jwt-verify-audience instead. This option will be removed in v5.

- `--legacy-functions-only`
   [DEPRECATED] PostGraphile 4.1.0 introduced support for PostgreSQL  functions than declare parameters with IN/OUT/INOUT or declare RETURNS  TABLE(...); enable this flag to ignore these types of functions. This  option will be removed in v5.

- `--legacy-relations <omit|deprecated|only>`
   some one-to-one relations were previously detected as one-to-many -  should we export 'only' the old relation shapes, both new and old but  mark the old ones as 'deprecated', or 'omit' the old relation shapes  entirely

- `--legacy-json-uuid`
   ONLY use this option if you require the v3 typenames 'Json' and 'Uuid' over 'JSON' and 'UUID'

- `-h`, `--help`
   output usage information

The following options are not part of PostGraphile core, but are available from the `@graphile/pg-pubsub` [subscriptions plugin](https://www.graphile.org/postgraphile/subscriptions/) (formerly the paid "supporter" plugin, but now fully free and open source - please consider [sponsoring us](https://www.graphile.org/sponsor/)!):

- `-S`, `--simple-subscriptions` add simple subscription support
- `--subscription-authorization-function [fn]` PG function to call to check user is allowed to subscribe

The following features and not part of PostGraphile core, but are available from the Pro plugin - see [Go Pro!](https://www.graphile.org/postgraphile/pricing/) for more information.

- `--read-only-connection <string>` [pro](https://www.graphile.org/postgraphile/pricing/) ⚡️[experimental] a PostgreSQL connection string to use for read-only queries (i.e. not mutations)
- `--default-pagination-cap [int]` [pro](https://www.graphile.org/postgraphile/pricing/) ⚡️[experimental] Ensures all connections have first/last specified and are no large than this value (default: 50), set to -1 to disable; override via smart comment `@paginationCap 50`
- `--graphql-depth-limit [int]` [pro](https://www.graphile.org/postgraphile/pricing/) ⚡️[experimental] Validates GraphQL queries cannot be deeper than the specified int (default: 16), set to -1 to disable
- `--graphql-cost-limit [int]` [pro](https://www.graphile.org/postgraphile/pricing/) ⚡️[experimental] Only allows queries with a computed cost below the specified int (default: 1000), set to -1 to disable

### 

### RC file options

The CLI options can also be specified in a `.postgraphilerc.js` file in the current working directory. Any command line comma separated options (eg. `schema`) must be entered as Javascript arrays.

```javascript
module.exports = {
  options: {
    connection: "postgres://api_user:api_password@localhost/api_development",
    schema: ["myApp", "myAppPrivate"],
    jwtSecret: "myJwtSecret",
    defaultRole: "myapp_anonymous",
    jwtTokenIdentifier: "myApp.jwt_token",
    watch: true,
  },
};
```

Here is the list of keys and their default values, or types, supported in the `options` object returned by `.postgraphilerc.js`.

```text
  appendPlugins: <string>
  bodySizeLimit: <string>
  classicIds = false
  clusterWorkers: <integer>
  connection: <string>
  cors= false
  defaultRole: <string>
  disableDefaultMutations = false
  disableGraphiql = false
  disableQueryLog: true/false
  dynamicJson = false
  enableQueryBatching: true/false
  exportSchemaGraphql: <path string>
  exportSchemaJson: <path string>
  extendedErrors = []
  graphiql = '/graphiql'
  graphql = '/graphql'
  host = 'localhost'
  includeExtensionResources = false
  jwtAudiences: <string>
  jwtRole = ['role']
  jwtSecret: <string>
  jwtSignOptions: {}
  jwtTokenIdentifier
  jwtVerifyAlgorithms: <string>
  jwtVerifyAudience: <string>
  jwtVerifyClockTolerance: <number>
  jwtVerifyId: <string>
  jwtVerifyIgnoreExpiration: true/false
  jwtVerifyIgnoreNotBefore: true/false
  jwtVerifyIssuer: <string>
  jwtVerifySubject: <string>
  legacyJsonUuid: true/false
  maxPoolSize: <number>
  plugins: <string>
  port = 5000
  prependPlugins: <string>
  readCache: <path string>
  schema: <string>
  secret: <string>
  showErrorStack: true/false
  simpleCollections: [omit|both|only]
  skipPlugins: <string>
  timeout: <number>
  token: : <string>
  watch: true/false
  writeCache: <path string>
```

Please note that this interface is deprecated and will be removed in v5 (but its replacement hasn't been built yet...). You're encouraged to use PostGraphile as a library rather than using a `.postgraphilerc.js`.

[PostGraphile](https://www.graphile.org/)

# PostgreSQL Tables

PostGraphile은 검사된 스키마에 있는 테이블과 열을 기반으로 생성된 GraphQL 스키마에 여러 요소를 자동으로 추가합니다.

PostgreSQL 테이블의 예는 다음과 같습니다:

```postgresql
CREATE TABLE app_public.users (
  id serial PRIMARY KEY,
  username citext NOT NULL unique,
  name text NOT NULL,
  about text,
  organization_id int NOT NULL
    REFERENCES app_public.organizations ON DELETE CASCADE,
  is_admin boolean NOT NULL DEFAULT false,
  created_at timestamptz NOT NULL DEFAULT now(),
  updated_at timestamptz NOT NULL DEFAULT now()
);
```

이와 같은 테이블의 경우 PostGraphile은 다음과 같이 합니다:

- 테이블에 대해 UpperCamelCase 및 단수로 명명된 GraphQL 유형 `User`를 생성합니다(인플렉터: `tableType`).
  - 이 유형에 열에 대한 필드(예: `id`, `username`, `about`, `organizationId`, `isAdmin`, `createdAt`, `updatedAt`)를 추가하며, 이름은 camelCase(변형자: `tableType`)로 지정합니다.
  - 테이블에 기본 키가 있는 경우 `nodeId` [globally unique identifier](https://www.graphile.org/postgraphile/node-id/) 필드를 추가합니다.
  - [field for the relevant relations를 필드를 추가합니다(예: `organizationByOrganizationId`*).
- 관련 테이블 유형에 추가합니다:
- 역방향 [relations for each forward relation](https://www.graphile.org/postgraphile/relations/) 관계를 추가합니다 (예: `Organization.usersByOrganizationId`*).
- 루트 `Query` 유형에 추가합니다:
  - 페이지 매김, 필터링 및 순서가 있는 `allUsers` [conection](https://www.graphile.org/postgraphile/connections/) 필드(인플렉터: allRows)
  - `userByKey(key: ...)` 필드(예: `userById`, `userByUsername`), 테이블의 고유 제약 조건 각각에 대한 필드 수(인플렉터: `rowByUniqueKeys`)
  - `nodeId`로 행을 가져오는 `foo(nodeId:ID!)` 필드
  - 루트 뮤테이션 유형에 CRUD 뮤테이션 추가

이러한 필드는 `@graphile-contrib/pg-simplify-inflector` 플러그인을 로드하여 단순화할 수 있습니다.

[관계](https://www.graphile.org/postgraphile/relations/), [연결](https://www.graphile.org/postgraphile/connections/), [필터링](https://www.graphile.org/postgraphile/filtering/) 및 [CRUD 뮤테이션]([relations](), [connections](), [filtering]() and [CRUD Mutations]().)에 대해 자세히 알아보십시오.

## Permissions (권한)

`--no-ignore-rbac` 또는 `ignoreRBAC: false`(매우 권장됨)를 사용하는 경우 PostGraphile은 액세스할 수 있는 테이블/열/필드만 표시합니다. 예를 들어 

```postgresql
GRANT UPDATE (username, name) ON users TO graphql_visitor;
```

를 수행하는 경우 `updateUser` 뮤테이션은 `username` 및 `name` 필드만 허용합니다. 다른 열은 표시되지 않습니다.

`--no-ignore-rbac`(또는 라이브러리에서 `ignoreRBAC: false`)은 데이터베이스의 RBAC(GRANT/REVECK) 권한을 검사하고 이러한 권한을 GraphQL 스키마에 반영합니다. GraphQL 모범 사례와 마찬가지로 여전히 하나의 GraphQL 스키마만 생성되므로 (사용자당 하나가 아닌) PostgreSQL에 연결한 사용자 계정을 사용하여 (연결 문자열에서) 이 사용자가 될 수 있는 모든 역할을 사용하고 이러한 모든 권한을 결합하여 사용합니다. 실제로 사용할 수 없는 기능을 포함하지 않는 훨씬 희박한 스키마가 되므로 이 플래그를 사용하는 것이 좋습니다.

* 참고: PostGraphile과 함께 컬럼 기반 `SELECT` grants을 사용하지 말 것을 강력히 권고합니다. 대신 권한 우려를 별도의 테이블로 나누고 일대일 관계로 참여하십시오.
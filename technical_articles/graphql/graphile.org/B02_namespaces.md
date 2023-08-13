[PostGraphile](https://www.graphile.org/)

# Namespaces (PostgreSQL "schemas")

PostgreSQL에서 각 데이터베이스는 여러 개의 "스키마"로 구성됩니다. 기본 스키마는 "public"으로 명명되며 많은 사용자가 이 하나의 스키마만 처리합니다.

PostGraphile에서는 응용 프로그램을 구성하는 데 도움이 되는 스키마를 사용하는 것이 좋습니다. GraphQL에 노출될 테이블에 대해 하나의 스키마를 사용할 수 있고, 완전히 개인적이어야 하는 테이블에 대해 다른 스키마를 사용할 수 있습니다(예: Bcrypted 사용자 암호 또는 다른 비밀이 절대 노출되지 않도록 저장하는 경우). 응용 프로그램에 적합한 모든 스키마를 사용할 수 있습니다.

PostgreSQL에서 스키마를 만드는 방법:

```postgresql
CREATE SCHEMA app_public;
```

해당 스키마에서 무언가를 만들거나 참조하려면 스키마 이름으로 사물의 이름을 붙이면 됩니다. 예:

```postgresql
CREATE TABLE app_public.users ( ... );
CREATE FUNCTION app_public.best_user() ...;

SELECT * FROM app_public.users;
SELECT * FROM app_public.best_user();
```

### Advice

저(벤지)는 지금 PostGraphile에서 꽤 많은 애플리케이션을 구축한 후 다음과 같이 결정했습니다:

- `app_public` - GraphQL(또는 다른 시스템)에 노출될 테이블과 기능 - 공개 인터페이스입니다. 이것은 데이터베이스의 주요 부분입니다.
- `app_hidden` - `app_public`과 동일한 권한이지만 공개적으로 노출하려는 것은 아닙니다. `app_public` 스키마의 "구현 세부 정보"와 같습니다. 자주 필요하지 않을 수도 있습니다.
- `app_private` - `SUPER SECRET STUFF` 🕵누구도 선택적으로 일을 할 수 있도록 하는 `SECURITY DEFINER` 기능 없이는 이것을 읽을 수 없습니다. 이것은 비밀번호(bcrypted), 액세스 토큰(즉, 암호화됨) 등을 저장하는 곳입니다. 웹 사용자가 액세스하는 것은 (RBAC(`GRANT`/`REBOKE) 덕분에) 불가능해야 합니다.

저는 개인적으로 PostgreSQL 확장이 설치되는 기본 위치 이외에는 `public` 스키마를 사용하지 않습니다.

이 패턴을 사용하는 것은 **필요하지 않으며** 실제로 선택한 경우 기본 공개 스키마를 사용할 수 있습니다. 기본적으로 PostGraphile은 확장자에 의해 설치된 리소스를 자동으로 무시하므로 수동으로 생략할 필요가 없습니다. [smart comments](https://www.graphile.org/postgraphile/smart-comments/) 기능을 사용하여 다른 테이블과 기능을 생략할 수 있습니다.

### Other schemas

다음은 볼 수 있는 다른 스키마입니다 (수정해서는 안 됩니다):

- `graphile_worker` - https://github.com/graphile/worker 에서 사용하고 관리하는 스키마입니다.
- `graphile_schema` - https://github.com/graphile/migrate 에서 사용하고 관리하는 스키마입니다.
- `postgraphile_watch` - 감시 모드(--watch` or `watchPg: true)에서 PostGraphile을 사용하는 경우, PostGraphile은 이 스키마를 설치하여 감시 모드를 활성화하기 위해 PostgreSQL 이벤트 트리거를 만듭니다.
- `information_schema` - 데이터베이스에 대한 반사를 위한 SQL 표준 스키마: https://www.postgresql.org/docs/current/information-schema.html
- `pg_catalog` - PostgreSQL 시스템 카탈로그는 데이터베이스 내부의 모든 정보(보다 PG 네이티브 형태의 `information_schema`)를 포함합니다
- `pg_*` - 기타 다양한 PostgreSQL 예약 스키마


[PostGraphile](https://www.graphile.org/)

# 관계 (Relations)

데이터베이스 테이블의 외부 키(그리고 `--no-ignore-indexes` 또는 `ignoreIndex: false`가 설정된 경우 인덱스)를 검사하여 데이터베이스 테이블 간의 관계를 자동으로 검색하고 이를 사용하여 생성된 GraphQL 스키마에 관계를 구축합니다.

테이블을 정의할 때 외부 키 제약의 예로는 다음과 같은 `REFERENCE` 키워드가 있습니다:

```postgresql
CREATE TABLE app_public.users (
  -- ...
  organization_id int NOT NULL
    REFERENCES app_public.organizations ON DELETE CASCADE,  -- ...
);
CREATE INDEX ON app_public.users (organization_id);
```

또는 테이블을 만든 후에 외부 키 제약 조건을 추가할 수 있습니다:

```postgresql
ALTER TABLE users
  ADD CONSTRAINT users_organization_id_fkey
  FOREIGN KEY (organization_id)
  REFERENCES organizations
  ON DELETE CASCADE;
CREATE INDEX ON users (organization_id);
```

PostgreSQL 설명서에서 여러 열을 사용하는 제약 조건을 포함하여 외부 키 제약 조건을 정의하는 방법에 대해 자세히 볼 수 있습니다.

PostGraphile은 일대일, 일대다 및 다대일 관계를 자동으로 탐지하고 노출합니다. 다대다 관계는 [다대다 관계 플러그인](https://github.com/graphile-contrib/pg-many-to-many)으로 처리할 수 있습니다.

기본적으로 관계는 대상 유형과 소스 필드(인플렉터: `singleRelationByKeys`, `singleRelationByKeysBackwards` 및 `manyRelationByKeys`)의 camelCase 조합을 사용하여 GraphQL 필드로 표시됩니다. 고유한 제약 조건은 GraphQL 테이블 유형을 직접 노출하고 고유하지 않은 제약 조건은 커넥션을 노출합니다. 이러한 관계가 노출하는 GraphQL 커넥션은 페이지 설정, 필터링 및 순서 지정을 지원합니다.

## 예
### 일대다 관계에 대한 데이터베이스 스키마 예제

```postgresql
create schema a;
create schema c;

create table c.person (
  id serial primary key,
  name varchar not null,
  about text,
  email varchar not null unique,
  created_at timestamp default current_timestamp
);

create table a.post (
  id serial primary key,
  headline text not null,
  body text,
  -- `references` 👇  sets up the foreign key relation
  author_id int4 references c.person(id)
);
create index on a.post (author_id);
```

### 위 스키마에 대한 쿼리 예제

```
{
  allPosts {
    nodes {
      headline
      body

      # this relation is automatically exposed
      personByAuthorId {
        id
        name
        about
      }
    }
  }
}
```

### 다대다 관계

다대다 관계는 다대다 관계 플러그인을 사용하거나 다음 집합을 반환하는 계산 열을 사용하여 처리할 수 있습니다:

```postgresql
create table post (
  id serial primary key,
  headline text,
  body text
);
create table author (
  id serial primary key,
  name text
);
create table post_author (
  post_id integer references post,
  author_id integer references author,
  primary key (post_id, author_id)
);

create function "post_authorsByPostId"(p post)
returns setof author as $$
  select author.*
  from author
  inner join post_author
  on (post_author.author_id = author.id)
  where post_author.post_id = p.id;
$$ language sql stable;
```


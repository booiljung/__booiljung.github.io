[Postgraphile](https://www.graphile.org/postgraphile/quick-start-guide/)

# Introduction

PostGraphile(이전의 PostGraphQL)은 PostgreSQL 스키마에서 강력하고 확장 가능하며 성능이 뛰어난 GraphQL API를 단 몇 초 만에 구축하므로 몇 달은 아니더라도 몇 주 정도의 개발 시간을 절약할 수 있습니다.

이미 PostgreSQL을 사용하고 있다면 강력하게 유형화되고 잘 정의된 스키마가 애플리케이션 개발에 가져다주는 가치를 이해하고 있을 것입니다. GraphQL은 프론트엔드 애플리케이션 개발자(또는 API 클라이언트)가 데이터 계층에 액세스할 수 있도록 하는 데 있어 이 기술과 완벽하게 일치합니다. [세계에서 가장 진보된 오픈 소스 데이터베이스](https://www.postgresql.org/)에 내장된 검증된 기능을 활용할 수 있는데 왜 사용자 지정 API에 권한 부여 및 비즈니스 로직을 복제해야 할까요?

*GraphQL을 처음 사용하시는 경우, [여기](https://graphql.org/learn/)에서 GraphQL에 대한 공식 소개를 읽어보신 후 PostGraphile 설명서를 계속 읽어보시기를 권장합니다.*

PostgreSQL의 [역할 기반 부여 시스템](https://www.postgresql.org/docs/current/static/user-manag.html) 및 [행 수준 보안 정책](https://www.postgresql.org/docs/current/static/ddl-rowsecurity.html)과 같은 강력한 기능을 Graphile Engine의 고급 [GraphQL 룩-어헤드](https://www.graphile.org/graphile-build/look-ahead/) 및 [플러그인 확장](https://www.graphile.org/graphile-build/plugins/) 기술과 결합함으로써 PostGraphile은 생성된 스키마의 보안, 성능 및 확장성을 보장합니다.

제공하는 기능 중 일부입니다:

- [놀라운 성능](https://www.graphile.org/postgraphile/performance/) - N+1 쿼리 문제 없음
- [스키마](https://www.graphile.org/postgraphile/extending/) 및 [서버](https://www.graphile.org/postgraphile/plugins/) 플러그인을 통한 확장성
- [자동 검색된 관계](https://www.graphile.org/postgraphile/relations/) 예: `userByAuthorId`
- [컴퓨트 컬럼](https://www.graphile.org/postgraphile/computed-columns/)을 통해 API를 쉽게 확장할 수 있습니다.
- [커스텀 쿼리 프러시저](https://www.graphile.org/postgraphile/custom-queries/) - 임의의 SQL 쿼리를 가능하게 합니다.
- [자동 CRUD 뮤테이션](https://www.graphile.org/postgraphile/crud-mutations/) 예: `updatePost`
- 복잡한 변경 사항을 간단하게 노출할 수 있는 [커스텀 뮤테이션 프로시저](https://www.graphile.org/postgraphile/custom-mutations/)
- [실시간](https://www.graphile.org/postgraphile/realtime/) 기능은 LISTEN/NOTIFY 및/또는 논리적 디코딩으로 구동됩니다.

가장 쉽게 시작할 수 있는 방법은 [CLI 인터페이스](https://www.graphile.org/postgraphile/usage-cli/)를 사용하는 것이며, `npx`가 설치되어 있다면 이를 사용해 볼 수 있습니다:

```text
npx postgraphile -c 'postgres://user:pass@localhost/mydb' --watch --enhance-graphiql --dynamic-json
```

(사용자, 패스, mydb를 PostgreSQL 사용자 이름, 비밀번호, 데이터베이스 이름으로 바꿉니다.)
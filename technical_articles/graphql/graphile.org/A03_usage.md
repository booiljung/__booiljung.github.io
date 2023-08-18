[Postgraphile](https://www.graphile.org/postgraphile/quick-start-guide/)

# Usage

포스트그래파일은 세 개의 레이어로 구성됩니다.

- 맨 위에는 [**PostGraphile CLI**](https://www.graphile.org/postgraphile/usage-cli/)가 있습니다. 이 계층은 가장 사용자 친화적이며 명령줄에서 일반적인 옵션을 수락하고, HTTP 서버(또는 사용 중인 옵션에 따라 서버 클러스터 또는 전혀 사용하지 않을 수도 있음)를 스핀업하며, PostGraphile 미들웨어를 마운트하는 세 가지 작업을 담당합니다 (다음 참조). **대부분의 사용자는 이 레이어부터 시작해야 하며, 많은 사용자가 프로덕션 환경에서 성공적으로 사용하고 있으므로 이 레이어부터 시작하는 것을 권장합니다.**
- PostGraphile CLI는 [**포스트그래파일 라이브러리의** 미들웨어](https://www.graphile.org/postgraphile/usage-library/)를 래핑합니다. 이 미들웨어는 Node.js HTTP, **Connect**, **Express** 또는 **Koa** 애플리케이션에 마운트하기에 적합합니다(라이브러리는 **Fastify** 및 **Restify**와 같은 다른 프레임워크에서 사용하기에 적합한 "라우트 핸들러"도 내보냅니다). 이 계층은 제공된 옵션에 따라 사용자로부터 GraphQL HTTP 요청을 수신, 해독 및 검증하고, 관련 설정으로 PG 클라이언트를 구성한 다음, 쿼리를 GraphQL 스키마(다음 참조)로 전송하여 해결하도록 하는 역할을 담당합니다. **약 70%의 PostGraphile 사용자가 애플리케이션에서 이 계층을 사용합니다**; CLI보다 이 계층을 사용하는 이유는 속도 제한, 세션, 사용자 지정 로깅, 사용자 지정 인증 및 기타 문제를 수행하기 위해 PostGraphile 전에 Express 미들웨어를 추가할 수 있고 PostGraphile 시스템을 더 잘 제어할 수 있다는 점입니다.

- 가장 깊은 곳에는 모든 타입, 필드, 리졸버를 포함하는 [**PostGraphile GraphQL Scheme**("scheme-only")](https://www.graphile.org/postgraphile/usage-schema/) 자체가 있습니다. (스키마는 동적으로 구성되므로 디스크에 기록할 수 없습니다.) **대부분의 사용자는 이 계층을 사용하지 않을 것입니다.**

스택의 깊이가 깊어질수록 설정 코드가 복잡해지지만 통합 기능은 더욱 강력해질 수 있습니다. 저희는 항상 CLI를 통해 가능한 한 많은 기능을 노출하려고 노력하지만, 가능한 모든 것을 CLI 옵션으로 만드는 것은 합리적이지 않습니다. 그 정도의 사용자 정의가 필요하다면 미들웨어를 선택해야 합니다.

PostGraphile CLI로 시작한 다음 Node.js와의 보다 심층적인 통합이 필요한 경우 PostGraphile 미들웨어로 이동하는 것이 좋습니다.

어떤 레이어에 대해 더 자세히 알고 싶으신가요?

- [**CLI**](A04_usage_cli.md)
- [**Middleware** ("library")](A05_usage_middleware.md)
- [**GraphQL schema** ("schema-only")](A06_usage_graphql_scheme.md)

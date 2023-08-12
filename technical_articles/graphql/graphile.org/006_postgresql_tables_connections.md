[PostGraphile](https://www.graphile.org/)

# 커넥션 (Connections)

GraphQL 필드가 데이터베이스 레코드의 많은 목록을 반환할 것으로 예상되는 경우, 우리는 일반적으로 [릴레이 커서 연결 사양](https://facebook.github.io/relay/graphql/connections.htm)을 준수하는 연결을 구현합니다(몇 가지 향상된 기능 포함). 이 연결을 통해 커서 기반 페이지를 쉽게 수행할 수 있으며, GraphQL 모범 사례로 간주됩니다.

릴레이의 연결 사양을 기반으로 적용된 개선 사항은 다음과 같습니다:

- `totalCount` - 쿼리와 일치하는 총 레코드 수(커서/제한/오프셋 제약 조건 제외)
- `nodes` - 모든 항목에 커서가 필요하지 않고 간단한 데이터 구조가 필요한 경우에 유용한 노드 (에지 래퍼 없음).
- `ageInfo.startCursor` 및 `PageInfo.endCursor` - `nodes { ... }` 대신 `edges { cursor, node { ... } }`가 유용합니다.

많은 연결(특히 테이블, 보기 및 관계에서 오는 연결)은 조건으로 반환 결과를 필터링하는 것을 지원합니다.

## 조언 (Advices)

GraphQL 연결보다 더 간단한 목록 인터페이스를 선호하는 경우, `--simple-collections [omit|both|only]` [옵션](https://www.graphile.org/postgraphile/usage-cli/)을 사용하여 연결과 함께(`both`) 또는 단독으로(`only`) 사용하도록 설정할 수 있습니다.

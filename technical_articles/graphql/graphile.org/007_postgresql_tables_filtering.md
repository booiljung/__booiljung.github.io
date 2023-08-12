[PostGraphile](https://www.graphile.org/)

# Filtering

PostGraphile은 기본적으로 `condition` 인수를 사용하여 [`connection`](https://www.graphile.org/postgraphile/connections/)에 대한 초보적인 필터링을 지원합니다. 이를 통해 특정 값 (예: `username: "Alice"` 또는 `category: ARTICLE`)에 대해 필터링할 수 있습니다.

[`connection`](https://www.graphile.org/postgraphile/connections/) 인수를 사용하는 예제를 참조하세요.

필터를 구현할 때 성능을 염두에 두는 것이 중요하므로 PostGraphile에서는 `@omit filter` [스마트 코멘트](https://www.graphile.org/postgraphile/smart-comments/)를 사용하여 필터 목록에서 특정 필드를 생략할 수 있는 기능을 제공합니다. `--no-ignore-indexes 를 사용하여 인덱싱되지 않는 것으로 보이는 필드를 자동으로 생략할 수도 있습니다.

## 고급 필터링

[사용자 정의 쿼리](https://www.graphile.org/postgraphile/custom-queries/), [계산된 열](https://www.graphile.org/postgraphile/computed-columns/)을 사용하여 필드를 추가하거나 [makeExtendSchemaPlugin](https://www.graphile.org/postgraphile/make-extend-schema-plugin/) 을 사용하여 고급 필터링 기능으로 PostGraphile의 스키마를 확장할 수 있습니다.

또한 다음과 같은 [커스텀 Graphile 엔진 플러그](https://www.graphile.org/postgraphile/extending-raw/)인을 사용하여 PostGraphile의 기존 커넥션을 보강할 수도 있습니다:

## 필터 플러그인

> 주의🚨: 강력한 일반 필터링 기능을 GraphQL API에 추가하는 것은 벤지(PostGraphile의 유지 관리자)뿐만 아니라 리 바이런(GraphQL의 발명가 중 한 명)과 GraphQL 생태계의 다른 여러 전문가들도 강력히 권장하지 않습니다. 이와 같은 일반적인 필터링 플러그인을 사용하는 것보다 위의 기술 중 하나를 사용하여 매우 특정한 필터만 추가하고 입력을 가능한 한 간단하게 만드는 것이 좋습니다. 이 조언에 유의하지 않으면 추후에 매우 심각한 성능 문제가 발생하여 해결하기가 매우 어려울 수 있습니다.

매우 인기 있는 플러그인은 [Matt Bretl의 커넥션 필터 플러그인](https://github.com/graphile-contrib/postgraphile-plugin-connection-filter)입니다. 이 플러그인은 커넥션에 필터 인수를 추가하여 다른 테이블의 관련 레코드에 대한 필터링, 필터링에 보다 큼, 보다 작음 및 범위 사용, 함수 출력에 대한 필터링 등 고급 필터를 사용할 수 있게 해줍니다. GraphQL API에서 고급 필터링이 필요한 경우(그리고 악의적인 사용자가 복잡한 요청을 하는 것을 방지하기 위해 [퍼시스트 쿼리](https://www.graphile.org/postgraphile/production/#simple-query-whitelist-persisted-queries)와 같은 것을 사용할 수 있는 경우) 이 기능을 확인해 보시기를 권장합니다!

커넥션 필터 플러그인에는 라이브 쿼리 관련 처리 기능이 없다는 점에 유의하세요. 즉, [라이브 쿼리](https://www.graphile.org/postgraphile/live-queries/)는 작동하지만 쿼리 결과가 변경되었는지 확인할 때 필터가 고려되지 않습니다. 즉, 필터를 적용한 결과가 여전히 동일하더라도 컬렉션의 항목이 변경될 때마다 구독 업데이트를 받을 수 있습니다.

포함된 `condition` 필터를 사용하면 `makeAddPgConditionPlugin`을 사용하여 고유한 조건을 추가하기 전까지는 라이브 쿼리가 예상대로 작동합니다.

## 기타 플러그인

필터링과 관련된 커뮤니티 플러그인은 [커뮤니티 플러그인 페이지](https://www.graphile.org/postgraphile/community-plugins/)에서 자세히 알아볼 수 있습니다.

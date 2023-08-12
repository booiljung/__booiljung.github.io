[PostGraphile](https://www.graphile.org/)

# Globally Unique Object Identification ("nodeId" / "id")

우리는 [GraphQL 글로벌 객체 식별 사양](https://facebook.github.io/relay/graphql/objectidentification.htm)을 구현하므로 기본 키가 있는 모든 테이블은 자동으로 쿼리 및 변경에 사용할 수 있는 고유한 `nodeId` 필드를 갖게 됩니다. 이 필드는 일반적으로 클라이언트 라이브러리의 캐시 키로 사용됩니다(예: Apollo 클라이언트의 `dataIdFromObject`):

```js
import ApolloClient from "apollo-client";
import { HttpLink } from "apollo-link-http";
import { InMemoryCache } from "apollo-cache-inmemory";

const cache = new InMemoryCache({
  dataIdFromObject: object => object.nodeId || null,});

export const client = new ApolloClient({
  link: new HttpLink(),
  cache,
});
```

**경고**: 기본적으로 데이터베이스 설계에서 일반적으로 사용되는 `id` 필드와의 충돌을 피하기 위해 전역 객체 식별자 `nodeId`를 호출합니다. 사양에 명시된 대로 전역 객체 식별자 필드 `id`를 대신 호출하려면 `--classic-ids` CLI 플래그를 사용하면 됩니다. 이렇게 하면 모든 `id` 열의 이름이 자동으로 `rowId`로 변경됩니다.

### 글로벌 오브젝트 식별자 비활성화하기 (Disabling the Global Object Identifier)

예를 들어 CLI에서 `require('graphile-build').NodePlugin` 플러그인을 건너뛰면 API 전체에서 글로벌 오브젝트 식별자를 비활성화할 수 있습니다:

```text
postgraphile --skip-plugins graphile-build:NodePlugin ...
```

하지만 GraphQL 클라이언트에 대한 캐시 식별자를 생성하는 좋은 방법이 있는지 확인하세요!

(참고: GraphQL 글로벌 객체 식별 사양은 이전에는 릴레이 글로벌 객체 식별 사양으로 알려졌지만 릴레이에만 국한된 것은 아닙니다).


[GraphQL](https://graphql.org/learn/)

# GraphQL 모범 사례(GraphQL Best Practices)

GraphQL 사양은 네트워크 처리, 권한 부여 및 페이지 매김과 같이 API가 직면한 몇 가지 중요한 문제에 대해 의도적으로 침묵하고 있습니다. 그렇다고 GraphQL을 사용할 때 이러한 문제에 대한 해결책이 없다는 뜻은 아니며, 단지 이러한 문제가 GraphQL이 무엇인지에 대한 설명에서 벗어난 일반적인 관행에 불과하다는 뜻입니다.

이 섹션의 글은 복음(gaspel)으로 받아들여서는 안 되며, 경우에 따라서는 다른 접근 방식을 선호하여 무시할 수도 있습니다. 일부 글은 GraphQL 서비스 설계 및 배포와 관련하여 Facebook 내에서 개발된 철학을 소개하는 반면, 다른 글은 HTTP를 통한 서비스 제공 및 인증 수행과 같은 일반적인 문제를 해결하기 위한 보다 전술적인 제안입니다.

다음은 GraphQL 서비스에 대한 일반적인 모범 사례와 의견에 따른 몇 가지 입장을 간략하게 설명한 것이지만, 이 섹션의 각 문서에서는 이러한 주제와 기타 주제에 대해 더 자세히 다룹니다.

### HTTP

GraphQL은 일반적으로 서비스의 전체 기능 집합을 표현하는 단일 엔드포인트를 통해 HTTP를 통해 제공됩니다. 이는 각각 단일 리소스를 노출하는 일련의 URL을 노출하는 REST API와는 대조적입니다. GraphQL은 리소스 URL 모음과 함께 사용할 수 있지만, [GraphiQL](https://github.com/graphql/graphiql)과 같은 도구와 함께 사용하기가 더 어려울 수 있습니다.

이에 대한 자세한 내용은 [HTTP를 통한 서비스](https://graphql.org/learn/serving-over-http/)를 참조하세요.

### JSON (with GZIP)

GraphQL 서비스는 일반적으로 JSON을 사용하여 응답하지만, GraphQL 사양은 [이를 요구하지 않습니다](http://spec.graphql.org/draft/#sec-Serialization-Format). JSON은 더 나은 네트워크 성능을 약속하는 API 계층으로서는 이상한 선택처럼 보일 수 있지만, 대부분 텍스트로 구성되어 있기 때문에 GZIP을 사용하면 매우 잘 압축됩니다.

모든 프로덕션 GraphQL 서비스는 GZIP을 사용하도록 설정하고 클라이언트가 헤더를 보내도록 권장하는 것이 좋습니다:

```
Accept-Encoding: gzip
```

JSON은 클라이언트 및 API 개발자에게도 매우 친숙하며 읽기 및 디버깅이 쉽습니다. 실제로 GraphQL 구문은 부분적으로 JSON 구문에서 영감을 받았습니다.

### Versioning

GraphQL 서비스도 다른 REST API와 마찬가지로 버전이 생성되는 것을 막을 수는 없지만, GraphQL은 GraphQL 스키마의 지속적인 진화를 위한 도구를 제공함으로써 버전 생성을 피하는 것을 강력히 권장합니다.

대부분의 API가 버전을 생성하는 이유는 무엇인가요? API 엔드포인트에서 반환되는 데이터에 대한 제어가 제한되어 있는 경우, *모든 변경 사항*은 변경 사항으로 간주될 수 있으며, 변경 사항을 변경하려면 새 버전이 필요합니다. API에 새로운 기능을 추가하려면 새 버전이 필요한 경우, 자주 릴리스하고 많은 증분 버전을 보유하는 것과 API의 이해 가능성 및 유지 관리 가능성 간에 상충되는 문제가 발생합니다.

이와는 대조적으로 GraphQL은 명시적으로 요청된 데이터만 반환하므로, 새로운 기능을 새로운 유형과 해당 유형에 대한 새로운 필드를 통해 변경 없이 추가할 수 있습니다. 이로 인해 항상 변경을 피하고 버전이 없는 API를 제공하는 것이 일반적인 관행으로 이어졌습니다.

### Nullability

"null"을 인식하는 대부분의 타입 시스템은 일반 타입과 해당 타입의 *nullable* 버전을 모두 제공하며, 명시적으로 선언하지 않는 한 기본 타입에는 "null"이 포함되지 않습니다. 그러나 GraphQL 유형 시스템에서는 모든 필드가 기본적으로 *null 가능*입니다. 이는 데이터베이스 및 기타 서비스에 의해 지원되는 네트워크 서비스에서 문제가 발생할 수 있는 일이 많기 때문입니다. 데이터베이스가 다운되거나, 비동기 작업이 실패하거나, 예외가 발생할 수 있습니다. 단순한 시스템 장애 외에도 권한 부여는 종종 요청 내의 개별 필드에 서로 다른 권한 부여 규칙이 적용될 수 있는 세분화된 문제일 수 있습니다.

모든 필드를 *nullable*으로 기본 설정하면 이러한 이유로 인해 요청이 완전히 실패하는 것이 아니라 해당 필드만 "null"로 반환될 수 있습니다. 대신 GraphQL은 요청이 있을 경우 해당 필드가 절대로 "null"을 반환하지 않는다는 것을 클라이언트에 보장하는 [non-null](https://graphql.org/learn/schema/#lists-and-non-null) 유형의 변형을 제공합니다. 대신 오류가 발생하면 이전 상위 필드가 대신 "null"이 됩니다.

GraphQL 스키마를 설계할 때는 오류가 발생할 수 있는 모든 문제를 염두에 두고 실패한 필드에 "null"이 적절한 값인지 여부를 고려하는 것이 중요합니다. 일반적으로는 그렇지만 때로는 그렇지 않은 경우도 있습니다. 이러한 경우에는 널이 아닌 유형을 사용하여 보장하세요.

### Pagination

GraphQL 유형 시스템을 사용하면 일부 필드에서 [밸류 리스트](https://graphql.org/learn/schema/#lists-and-non-null)을 반환할 수 있지만, 긴 밸류 리스트의 페이지 매김은 API 디자이너에게 맡깁니다. 페이지 매김을 위한 다양한 API 설계가 가능하며, 각 설계에는 장단점이 있습니다.

일반적으로 긴 리스트를 반환할 수 있는 필드는 리스트의 특정 영역을 지정할 수 있도록 "first" 및 "after" 인수를 허용하며, 여기서 "after"는 리스트에 있는 각 값의 고유 식별자입니다.

궁극적으로 풍부한 페이지 매김 기능을 갖춘 API를 설계하면서 "연결(Connection)"이라는 모범 사례 패턴이 탄생했습니다. [릴레이](https://facebook.github.io/relay/)와 같은 일부 GraphQL용 클라이언트 도구는 연결 패턴에 대해 알고 있으며 GraphQL API가 이 패턴을 사용할 때 클라이언트 측 페이지 매김을 자동으로 지원할 수 있습니다.

이에 대한 자세한 내용은 [페이지 매김](https://graphql.org/learn/pagination/) 문서를 참조하세요.

### Server-side Batching & Caching

GraphQL은 서버에서 깔끔한 코드를 작성할 수 있도록 설계되었으며, 모든 유형의 모든 필드에는 해당 값을 확인하는 단일 목적 함수가 집중되어 있습니다. 그러나 추가적인 고려가 없다면 나이브한 GraphQL 서비스는 매우 '수다스럽거나' 데이터베이스에서 데이터를 반복적으로 로드할 수 있습니다.

이는 일반적으로 일괄 처리 기법을 통해 해결되는데, 이 기법은 백엔드에서 데이터에 대한 여러 요청을 단기간에 수집한 다음 Facebook의 [DataLoader](https://github.com/facebook/dataloader)와 같은 도구를 사용하여 기본 데이터베이스 또는 마이크로서비스에 단일 요청으로 발송하는 것입니다.

---

[Thinking in Graphs](007_thinking_in_graphs.md)

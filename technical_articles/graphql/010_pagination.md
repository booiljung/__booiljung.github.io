[GraphQL](https://graphql.org/learn/)

# 패이지 매김(Pagination)

> 다양한 페이지 매김 모델을 통해 다양한 클라이언트 기능 지원

GraphQL의 일반적인 사용 사례는 개체 집합 간의 관계를 횡단(traverse)하는 것입니다. GraphQL에서 이러한 관계를 노출(expose)할 수 있는 여러 가지 방법이 있으며, 클라이언트 개발자에게 다양한 기능을 제공합니다.

## Plurals

개체 간의 연결을 노출하는 가장 간단한 방법은 복수형을 반환하는 필드를 사용하는 것입니다. 예를 들어 "R2-D2"의 친구 목록을 가져오려면 모든 친구를 요청하면 됩니다:

```
{
  hero {
    name
    friends {
      name
    }
  }
}
```

```
{
  "data": {
    "hero": {
      "name": "R2-D2",
      "friends": [
        {
          "name": "Luke Skywalker"
        },
        {
          "name": "Han Solo"
        },
        {
          "name": "Leia Organa"
        }
      ]
    }
  }
}
```

## Slicing

하지만 곧 클라이언트가 원하는 추가 동작이 있다는 것을 알게 되었습니다. 클라이언트는 가져올 친구의 수를 지정할 수 있기를 원할 수도 있고, 처음 두 명만 원할 수도 있습니다. 따라서 다음과 같이 노출하고 싶을 것입니다:

```
{
  hero {
    name
    friends(first: 2) {
      name
    }
  }
}
```

하지만 처음 두 친구를 가져온 경우, 클라이언트가 처음 두 친구를 가져온 후 다음 두 친구를 요청하기 위해 두 번째 요청을 보낼 수 있습니다. 이 동작을 어떻게 활성화할 수 있을까요?

## Pagination and Edges

페이지 매김을 할 수 있는 방법은 여러 가지가 있습니다:

- `friends(first:2 offset:2)`와 같이 목록에서 다음 두 친구를 요청할 수 있습니다.
- `friends(first:2 after:$friendId)`와 같이 마지막으로 가져온 친구의 다음 두 친구를 요청할 수 있습니다.
- 마지막 항목에서 커서를 가져와 페이지 매김에 사용하는 `friends(first:2 after:$friendCursor)`와 같은 식으로 할 수 있습니다.

일반적으로 **커서 기반 페이지 매김**이 가장 강력하게 설계된 것으로 나타났습니다. 특히 커서가 불투명(opaque)한 경우 커서를 오프셋 또는 ID로 만들어 커서 기반 페이지 매김을 구현할 수 있으며, 커서를 사용하면 향후 페이지 매김 모델이 변경될 경우 추가적인 유연성을 확보할 수 있습니다. 커서가 불투명하고 커서 형식에 의존해서는 안 된다는 점을 상기시켜드리기 위해 커서를 base64로 인코딩하는 것이 좋습니다.

그런데 문제는 객체에서 커서를 어떻게 가져올 수 있을까요? 커서는 객체가 아닌 연결의 속성이기 때문에 `User` 유형에 커서가 있는 것은 원하지 않습니다. 따라서 새로운 방향성 계층을 도입할 수 있습니다. `friends` 필드는 에지 목록(list of edges)을 제공해야 하며, 에지(edige)에는 커서와 기본 노드가 모두 있습니다:

```
{
  hero {
    name
    friends(first: 2) {
      edges {
        node {
          name
        }
        cursor
      }
    }
  }
}
```

엣지라는 개념은 객체 중 하나가 아닌 엣지에 특정한 정보가 있을 때도 유용합니다. 예를 들어, API에서 'friendship time'을 노출하고 싶을 때, 이를 엣지에 저장하는 것이 자연스럽습니다.

## End-of-list, counts, and Connections

이제 커서를 사용하여 연결을 페이지 매김할 수 있게 되었지만 연결의 끝에 도달했을 때를 어떻게 알 수 있을까요? 빈 목록을 반환할 때까지 계속 쿼리해야 하지만, 연결이 언제 끝에 도달했는지 알려주면 추가 요청이 필요하지 않을 것입니다. 마찬가지로 연결 자체에 대한 추가 정보(예: R2-D2의 총 친구 수)를 알고 싶으면 어떻게 해야 할까요?

이 두 가지 문제를 모두 해결하기 위해 `friends` 필드는 연결 객체를 반환할 수 있습니다. 그러면 연결 객체에는 엣지에 대한 필드와 기타 정보(예: 총 개수 및 다음 페이지가 존재하는지 여부에 대한 정보)가 포함됩니다. 따라서 최종 쿼리는 다음과 같이 보일 수 있습니다:

```
{
  hero {
    name
    friends(first: 2) {
      totalCount
      edges {
        node {
          name
        }
        cursor
      }
      pageInfo {
        endCursor
        hasNextPage
      }
    }
  }
}
```

이 `PageInfo` 객체에는 `endCursor`와 `startCursor`도 포함될 수 있습니다. 이렇게 하면 페이지 매김에 필요한 커서를 `pageInfo`에서 가져왔기 때문에 엣지에 포함된 추가 정보가 필요하지 않은 경우 가장자리에 대해 쿼리할 필요가 전혀 없습니다. 이는 연결에 대한 잠재적인 사용성 개선으로 이어집니다. '엣지' 목록을 노출하는 대신 노드만 전용 목록으로 노출하여 방향성 계층을 피할 수도 있습니다.

## Complete Connection Model

복수로만 구성하려던 원래 디자인보다 더 복잡해진 것은 분명합니다! 하지만 이 디자인을 채택함으로써 클라이언트를 위한 여러 가지 기능을 사용할 수 있게 되었습니다:

- 목록을 페이지 매김하는 기능.
- `totalCount` 또는 `pageInfo`와 같은 연결 자체에 대한 정보를 요청할 수 있는 기능.
- `cursor` 또는 `friendshipTime`과 같이 엣지 자체에 대한 정보를 요청하는 기능.
- 사용자가 불투명한 커서를 사용하기 때문에 백엔드에서 페이지 매김을 수행하는 방식을 변경할 수 있는 기능입니다.

이 기능을 실제로 확인하기 위해 예제 스키마에 `friendsConnection`이라는 추가 필드가 있으며, 이 필드에는 이러한 모든 개념이 노출되어 있습니다. 예제 쿼리에서 확인할 수 있습니다. `friendsConnection`에서 `after` 매개변수를 제거하여 페이지 매김에 어떤 영향이 있는지 확인해 보세요. 또한 클라이언트에 적합한 경우 연결의 에지 필드를 헬퍼 친구 필드로 대체하여 추가 에지 레이어 없이 친구 목록으로 바로 이동할 수 있도록 해 보세요.

```
{
  hero {
    name
    friendsConnection(first: 2, after: "Y3Vyc29yMQ==") {
      totalCount
      edges {
        node {
          name
        }
        cursor
      }
      pageInfo {
        endCursor
        hasNextPage
      }
    }
  }
}
```

```
{
  "data": {
    "hero": {
      "name": "R2-D2",
      "friendsConnection": {
        "totalCount": 3,
        "edges": [
          {
            "node": {
              "name": "Han Solo"
            },
            "cursor": "Y3Vyc29yMg=="
          },
          {
            "node": {
              "name": "Leia Organa"
            },
            "cursor": "Y3Vyc29yMw=="
          }
        ],
        "pageInfo": {
          "endCursor": "Y3Vyc29yMw==",
          "hasNextPage": false
        }
      }
    }
  }
}
```

## Connection Specification

이 패턴의 일관된 구현을 보장하기 위해 릴레이 프로젝트에는 커서 기반 연결 패턴을 사용하는 GraphQL API를 구축하기 위해 따를 수 있는 공식 [사양](https://facebook.github.io/relay/graphql/connections.htm)이 있습니다.

---

[Global Object Identification](011_global_object_identification.md)
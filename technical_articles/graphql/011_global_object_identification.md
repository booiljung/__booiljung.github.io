[GraphQL](https://graphql.org/learn/)

# Global Object Identification

> 일관된 오브젝트 액세스를 통해 간단한 캐싱 및 오브젝트 조회 가능

GraphQL 클라이언트가 캐싱 및 데이터 리페칭을 원활하게 처리할 수 있는 옵션을 제공하려면 GraphQL 서버가 표준화된 방식으로 객체 식별자를 노출해야 합니다.

이를 위해서는 클라이언트가 표준 메커니즘을 통해 쿼리하여 ID로 객체를 요청해야 합니다. 그런 다음 스키마는 응답에서 이러한 ID를 제공하는 표준 방법을 제공해야 합니다.

ID 외에는 개체에 대해 알려진 것이 거의 없기 때문에 이러한 개체를 "노드(nodes)"라고 부릅니다. 다음은 노드에 대한 쿼리 예제입니다:

```
{
  node(id: "4") {
    id
    ... on User {
      name
    }
  }
}
```

- GraphQL 스키마는 루트 쿼리 객체의 `node` 필드를 통해 모든 객체를 가져올 수 있도록 형식이 지정되어 있습니다. 이것은 "노드" [인터페이스](https://graphql.org/learn/schema/#interfaces)를 준수하는 객체를 반환합니다.
- `id` 필드는 응답에서 안전하게 추출할 수 있으며, 캐싱 및 리페칭을 통해 재사용할 수 있도록 저장할 수 있습니다.
- 클라이언트는 인터페이스 조각을 사용하여 노드 인터페이스에 부합하는 유형에 특정한 추가 정보를 추출할 수 있습니다. 이 경우 "User"입니다.

노드 인터페이스는 다음과 같습니다:

```
# An object with a Globally Unique ID
interface Node {
  # The ID of the object.
  id: ID!
}
```

사용자가 다음을 통해 준수합니다:

```
type User implements Node {
  id: ID!
  # Full name
  name: String!
}
```

# Specification

아래의 모든 내용은 서버 구현 전반에 걸쳐 일관성을 보장하기 위해 준수해야 하는 객체 식별에 관한 사양을 보다 공식적인 요구 사항과 함께 설명합니다. 이러한 사양은 서버가 [Relay](https://facebook.github.io/relay/) API 클라이언트와 호환되는 방법을 기반으로 하지만 모든 클라이언트에 유용할 수 있습니다.

# Reserved Types

이 사양과 호환되는 GraphQL 서버는 일관된 객체 식별 모델을 지원하기 위해 특정 유형과 유형 이름을 예약해야 합니다. 특히, 이 스펙은 다음 유형에 대한 가이드라인을 생성합니다:

- `Node`라는 인터페이스.
- 루트 쿼리 유형의 `node` 필드.

# Node Interface

서버는 `Node`라는 인터페이스를 제공해야 합니다. 이 인터페이스에는 `null`이 아닌 `ID`를 반환하는 `id`라는 필드가 정확히 하나 포함되어야 합니다.

이 `id`는 이 객체에 대한 전역적으로 고유한 식별자이어야 하며, 이 `id`만 주어지면 서버가 객체를 리페치할 수 있어야 합니다.

## Introspection

위의 인터페이스를 올바르게 구현한 서버는 다음 인트로스펙션 쿼리를 수락하고 제공된 응답을 반환합니다:

```
{
  __type(name: "Node") {
    name
    kind
    fields {
      name
      type {
        kind
        ofType {
          name
          kind
        }
      }
    }
  }
}
```

반환:

```
{
  "__type": {
    "name": "Node",
    "kind": "INTERFACE",
    "fields": [
      {
        "name": "id",
        "type": {
          "kind": "NON_NULL",
          "ofType": {
            "name": "ID",
            "kind": "SCALAR"
          }
        }
      }
    ]
  }
}
```

# Node root field

서버는 `Node` 인터페이스를 반환하는 `node`라는 루트 필드를 제공해야 합니다. 이 루트 필드는 정확히 하나의 인자, 즉 `id`라는 non-null인 ID를 취해야 합니다.

쿼리가 `Node`를 구현하는 객체를 반환하는 경우 서버가 `Node`의 `id` 필드에 반환한 값을 `id` 매개변수로 `node` 루트 필드에 전달하면 이 루트 필드는 동일한 객체를 리페치해야 합니다.

서버는 이 데이터를 가져오기 위해 최선을 다해야 하지만 항상 가능한 것은 아닙니다. 예를 들어, 서버가 유효한 `id`를 가진 `User`를 반환할 수 있지만 `node` 루트 필드로 해당 User를 다시 가져오도록 요청할 때 User의 데이터베이스를 사용할 수 없거나 사용자가 계정을 삭제했을 수 있습니다. 이 경우 이 필드를 쿼리한 결과는 `null`이어야 합니다.

## Introspection

위의 요구 사항을 올바르게 구현한 서버는 다음 인트로스펙션 쿼리를 수락하고 제공된 응답이 포함된 응답을 반환합니다.

```
{
  __schema {
    queryType {
      fields {
        name
        type {
          name
          kind
        }
        args {
          name
          type {
            kind
            ofType {
              name
              kind
            }
          }
        }
      }
    }
  }
}
```

반환:

```
{
  "__schema": {
    "queryType": {
      "fields": [
        // This array may have other entries
        {
          "name": "node",
          "type": {
            "name": "Node",
            "kind": "INTERFACE"
          },
          "args": [
            {
              "name": "id",
              "type": {
                "kind": "NON_NULL",
                "ofType": {
                  "name": "ID",
                  "kind": "SCALAR"
                }
              }
            }
          ]
        }
      ]
    }
  }
}
```

# Field stability

쿼리에 두 개의 객체가 나타나고 둘 다 동일한 ID를 가진 `Node`를 구현하는 경우, 두 객체는 동일해야 합니다.

이 정의의 목적상 객체 동등성은 다음과 같이 정의됩니다:

- 두 개체 모두에서 필드가 쿼리되는 경우 첫 번째 개체에서 해당 필드를 쿼리한 결과는 두 번째 개체에서 해당 필드를 쿼리한 결과와 같아야 합니다.
  - 필드가 스칼라를 반환하는 경우 해당 스칼라에 적합한 동등성이 정의됩니다.
  - 필드가 열거형을 반환하는 경우 동등성은 두 필드가 동일한 열거형 값을 반환하는 것으로 정의됩니다.
  - 필드가 객체를 반환하는 경우, 동등성은 위와 같이 재귀적으로 정의됩니다.

예를 들어

```
{
  fourNode: node(id: "4") {
    id
    ... on User {
      name
      userWithIdOneGreater {
        id
        name
      }
    }
  }
  fiveNode: node(id: "5") {
    id
    ... on User {
      name
      userWithIdOneLess {
        id
        name
      }
    }
  }
}
```

반환:

```
{
  "fourNode": {
    "id": "4",
    "name": "Mark Zuckerberg",
    "userWithIdOneGreater": {
      "id": "5",
      "name": "Chris Hughes"
    }
  },
  "fiveNode": {
    "id": "5",
    "name": "Chris Hughes",
    "userWithIdOneLess": {
      "id": "4",
      "name": "Mark Zuckerberg"
    }
  }
}
```

`fourNode.id`와 `fiveNode.userWithIdOneLess.id`가 동일하기 때문에 위의 조건에 따라 `fourNode.name`이 `fiveNode.userWithIdOneLess.name`과 동일해야 하며 실제로도 동일합니다.

# Plural identifying root fields

사용자의 사용자 이름을 가져와 해당 사용자를 반환하는 `username`이라는 이름의 루트 필드가 있다고 가정해 보겠습니다:

```
{
  username(username: "zuck") {
    id
  }
}
```

다음이 반환될 수 있습니다:

```
{
  "username": {
    "id": "4"
  }
}
```

분명히 응답의 객체, 즉 ID가 4인 사용자를 요청과 연결하여 사용자 이름 "zuck"을 가진 객체를 식별할 수 있습니다. 이제 사용자 이름 목록을 받아 객체 목록을 반환하는 `usernames`라는 이름의 루트 필드를 상상해 보겠습니다:

```
{
  usernames(usernames: ["zuck", "moskov"]) {
    id
  }
}
```

다음이 반환될 수 있습니다:

```
{
  "usernames": [
    {
      "id": "4"
    },
    {
      "id": "6"
    }
  ]
}
```

클라이언트가 사용자 이름을 응답에 연결할 수 있으려면 응답의 배열이 인수로 전달된 배열과 크기가 같고 응답의 순서가 인수의 순서와 일치하는지 알아야 합니다. 이를 *복수 식별 루트 필드(plural identifying root field)*라고 하며, 그 요구 사항은 아래에 설명되어 있습니다.

## Fields

이 스펙을 준수하는 서버는 입력 인수 목록을 허용하고 응답 목록을 반환하는 루트 필드를 노출할 수 있습니다. 사양을 준수하는 클라이언트가 이러한 필드를 사용하려면 해당 필드가 *복수 식별 루트 필드*여야 하며 다음 요구 사항을 준수해야 합니다.

참고 사양 준수 서버는 *복수 식별 루트 필드*가 아닌 루트 필드를 노출할 수 있으며, 사양 준수 클라이언트는 해당 필드를 쿼리에서 루트 필드로 사용할 수 없습니다.

*복수 식별 루트 필드*에는 단일 인수가 있어야 합니다. 해당 인수의 유형은 널이 아닌 non-nullable 목록이어야 합니다. `username` 예제에서 필드는 `username`이라는 단일 인수를 받으며, 이 인수의 유형(유형 시스템 약어를 사용)은 `[String!]!`이 됩니다.

*복수 식별 루트 필드*의 반환 유형은 목록이거나 목록을 둘러싼 non-nullable 래퍼여야 합니다. 목록은 `Node` 인터페이스, `Node` 인터페이스를 구현하는 객체 또는 이러한 유형을 둘러싼 non-nullable 래퍼를 감싸야 합니다.

*복수 식별 루트 필드*를 사용할 때마다 응답의 목록 길이는 인수의 목록 길이와 같아야 합니다. 응답의 각 항목은 입력의 해당 항목과 일치해야 합니다. 좀 더 공식적으로, 루트 필드에 입력 목록 `Lin`을 전달하면 출력 값 `Lout`이 나온다면, 임의의 순열 `P`의 경우 루트 필드 `P(Lin)`를 전달하면 출력 값 `P(Lout)`가 나와야 합니다.

따라서 서버는 입력의 특정 항목에 대한 객체를 가져올 수 없는 경우에도 해당 입력 항목에 대한 값을 출력에 제공해야 하므로 응답 유형에 non-nullable 래퍼를 래핑하지 않는 것이 좋으며, `null`은 이를 위해 유용한 값입니다.

---

[Caching](012_caching.md)
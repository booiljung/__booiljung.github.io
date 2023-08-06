[GraphQL](https://graphql.org/learn/)

# Execution

유효성을 검사한 후 GraphQL 쿼리는 요청된 쿼리의 형태와 동일한 결과를 반환하는 GraphQL 서버에 의해 실행되며, 일반적으로 JSON으로 반환됩니다.

GraphQL은 타입 시스템 없이는 쿼리를 실행할 수 없으므로 예제 타입 시스템을 사용하여 쿼리 실행을 설명해 보겠습니다. 이 예제는 이 글의 예제 전체에서 사용된 것과 동일한 타입 시스템의 일부입니다:

```
type Query {
  human(id: ID!): Human
}

type Human {
  name: String
  appearsIn: [Episode]
  starships: [Starship]
}

enum Episode {
  NEWHOPE
  EMPIRE
  JEDI
}

type Starship {
  name: String
}
```

쿼리가 실행될 때 어떤 일이 발생하는지 설명하기 위해 예제를 통해 살펴보겠습니다.

```
{
  human(id: 1002) {
    name
    appearsIn
    starships {
      name
    }
  }
}
```

```json
{
  "data": {
    "human": {
      "name": "Han Solo",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ],
      "starships": [
        {
          "name": "Millenium Falcon"
        },
        {
          "name": "Imperial shuttle"
        }
      ]
    }
  }
}
```

GraphQL 쿼리의 각 필드는 다음 유형을 반환하는 이전 유형의 함수 또는 메서드라고 생각할 수 있습니다. 실제로 이것이 바로 GraphQL이 작동하는 방식입니다. 각 유형의 각 필드는 GraphQL 서버 개발자가 제공하는 *resolver*라는 함수에 의해 뒷받침됩니다. 필드가 실행되면 해당 *resolver*가 호출되어 다음 값을 생성합니다.

필드가 문자열이나 숫자와 같은 스칼라 값을 생성하면 실행이 완료됩니다. 그러나 필드가 객체 값을 생성하는 경우 쿼리에는 해당 객체에 적용되는 다른 필드 선택 항목이 포함됩니다. 이 과정은 스칼라 값에 도달할 때까지 계속됩니다. GraphQL 쿼리는 항상 스칼라 값에서 끝납니다.

## 루트 필드 및 리졸버(Root field & resolvers)

모든 GraphQL 서버의 최상위 수준에는 GraphQL API에 가능한 모든 진입점을 나타내는 유형이 있으며, 이를 흔히 *Root* 유형 또는 *Query* 유형이라고 부릅니다.

이 예제에서 쿼리 유형은 `id`라는 인수를 허용하는 `human`이라는 필드를 제공합니다. 이 필드의 resolver 함수는 데이터베이스에 액세스한 다음 `Human` 객체를 구성하여 반환할 것입니다.

```js
Query: {
  human(obj, args, context, info) {
    return context.db.loadHumanByID(args.id).then(
      userData => new Human(userData)
    )
  }
}
```

이 예제는 자바스크립트로 작성되었지만 GraphQL 서버는 다양한 언어로 구축할 수 있습니다. 리졸버 함수는 네 개의 인수를 받습니다:

- `obj`: 루트 쿼리 유형에 있는 필드의 경우 종종 사용되지 않는 이전 객체입니다.
- `args`: GraphQL 쿼리에서 필드에 제공되는 인수입니다.
- `context`: 모든 리졸버에 제공되며 현재 로그인한 사용자 또는 데이터베이스에 대한 액세스와 같은 중요한 컨텍스트 정보를 보유하는 값입니다.
- `info`: 현재 쿼리와 관련된 필드별 정보와 스키마 세부 정보를 보유하는 값으로, [자세한 내용은 `GraphQLResolveInfo` 유형](https://graphql.org/graphql-js/type/#graphqlobjecttype)을 참조하세요.

## 비동기 리졸버(Asynchronous resolvers)

이 리졸버 함수에서 어떤 일이 일어나는지 자세히 살펴보겠습니다.

```js
human(obj, args, context, info) {
  return context.db.loadHumanByID(args.id).then(
    userData => new Human(userData)
  )
}
```

`context`는 GraphQL 쿼리에서 인수로 제공된 `id`로 사용자의 데이터를 로드하는 데 사용되는 데이터베이스에 대한 액세스를 제공하는 데 사용됩니다. 데이터베이스에서 로드하는 것은 비동기 작업이므로, 이것은 [`Promise`](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Promise)를 반환합니다. JavaScript에서 `Promise`는 비동기 값으로 작업하는 데 사용되지만, 동일한 개념이 많은 언어에 존재하며, 종종 `Future`, `Task` 또는 `Deferred`라고 불립니다. 데이터베이스가 반환되면 새로운 `Human` 객체를 생성하여 반환할 수 있습니다.

리졸버 함수는 `Promise`를 인식해야 하지만 GraphQL 쿼리는 그렇지 않다는 점에 주목하세요. 단순히 `Human` 필드가 `name`을 물어볼 수 있는 무언가를 반환할 것으로 기대합니다. 실행 중에 GraphQL은 계속하기 전에 `Promise`, `Future` 및 `Task`가 완료될 때까지 기다리며 최적의 동시성을 유지합니다.

## Trivial resolvers

이제 `Human` 개체를 사용할 수 있으므로 요청된 필드를 사용하여 GraphQL 실행을 계속할 수 있습니다.

```
Human: {
  name(obj, args, context, info) {
    return obj.name
  }
}
```

GraphQL 서버는 다음에 수행할 작업을 결정하는 데 사용되는 유형 시스템에 의해 구동됩니다. `Human` 필드가 아무 것도 반환하기 전에도 GraphQL은 유형 시스템이 `Human` 필드가 인간을 반환할 것이라고 알려주기 때문에 다음 단계가 `Human` 유형에 대한 필드 리졸브라는 것을 알고 있습니다.

이 경우 이름 Resolving 은 매우 간단합니다. 이름 리졸버 함수가 호출되고 객체 인수는 이전 필드에서 반환된 `new Human` 객체입니다. 이 경우 `Human` 객체에는 직접 읽고 반환할 수 있는 이름 프로퍼티가 있을 것으로 예상합니다.

실제로 많은 GraphQL 라이브러리에서는 이렇게 간단하게 리졸버를 생략할 수 있으며, 필드에 리졸버가 제공되지 않는 경우 동일한 이름의 프로퍼티를 읽고 반환해야 한다고 가정합니다.

## Scalar coercion

`name` 필드가 리졸브되는 동안 `appearsIn` 및 `starships` 필드는 동시에 리졸브 될 수 있습니다. 나타나는 필드에도 사소한 리졸버가 있을 수 있지만 자세히 살펴보겠습니다:

```
Human: {
  appearsIn(obj) {
    return obj.appearsIn // returns [ 4, 5, 6 ]
  }
}
```

시스템 시스템 클레임이 나타나는 유형은 알려진 값을 가진 Enum 값을 반환하지만, 이 함수는 숫자를 반환합니다! 실제로 결과를 조회해보면 적절한 Enum 값이 반환되고 있음을 알 수 있습니다. 무슨 일이 벌어지고 있는 걸까요?

이것은 스칼라 강제의 예입니다. 타입 시스템은 무엇을 예상해야 하는지 알고 있으며 리졸버 함수가 반환한 값을 API 계약을 준수하는 값으로 변환합니다. 이 경우, 내부적으로는 4, 5, 6과 같은 숫자를 사용하지만 GraphQL 유형 시스템에서는 열거형 값으로 표시하는 Enum이 서버에 정의되어 있을 수 있습니다.

## List resolvers

이미 위에서 `appearsIn` 필드가 사물(things)의 리스트를 반환할 때 어떤 일이 발생하는지 살펴봤습니다. 이 필드는 열거형 값의 리스트를 반환했고, 유형 시스템이 예상한 대로 리스트의 각 항목이 적절한 열거형 값으로 강제로 변환되었습니다. `starships` 필드가 리졸브되면 어떻게 될까요?

```
Human: {
  starships(obj, args, context, info) {
    return obj.starshipIDs.map(
      id => context.db.loadStarshipByID(id).then(
        shipData => new Starship(shipData)
      )
    )
  }
}
```

이 필드의 리졸버는 단순히 `Promise`를 반환하는 것이 아니라 `Promise list`를 반환합니다. `Human` 객체에는 그들이 조종한 `startshipIDs` 리스트가 있지만, 실제 `Startships` 객체를 얻으려면 모든 ID를 로드해야 합니다.

GraphQL은 계속하기 전에 이러한 모든 `Promise`를 동시에 기다린 다음, 객체 리스타가 남으면 각 항목의 `name` 필드를 로드하기 위해 다시 한 번 계속합니다.

## Producing the result

각 필드가 리졸브되면 결과 값은 필드 이름(또는 별칭)을 키로, 리졸브된 값을 값으로 하는 key-value map에 배치됩니다. 이는 쿼리의 맨 아래 leaves 필드에서 Root 쿼리 유형의 original 필드까지 계속 이어집니다. 이 모든 것을 종합하면 original 쿼리를 미러링하는 구조가 생성되며, 이 구조는 쿼리를 요청한 클라이언트에 (일반적으로 JSON으로) 전송될 수 있습니다.

마지막으로 original 쿼리를 다시 한 번 살펴보고 이러한 모든 리졸브 함수가 어떻게 결과를 생성하는지 살펴보겠습니다:

```
{
  human(id: 1002) {
    name
    appearsIn
    starships {
      name
    }
  }
}
```

```json
{
  "data": {
    "human": {
      "name": "Han Solo",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ],
      "starships": [
        {
          "name": "Millenium Falcon"
        },
        {
          "name": "Imperial shuttle"
        }
      ]
    }
  }
}
```

---

- [Introspection](005_introspection.md)

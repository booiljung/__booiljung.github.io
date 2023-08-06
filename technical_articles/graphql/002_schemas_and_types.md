[GraphQL](https://graphql.org/learn/)

# 스키마 및 유형

이 페이지에서는 GraphQL 유형 체계(Type system)와 쿼리할 수 있는 데이터를 설명하는 방식에 대해 알아야 할 모든 것을 배웁니다. GraphQL은 모든 백엔드 프레임워크 또는 프로그래밍 언어와 함께 사용할 수 있으므로 구현과 관련된 세부 사항은 피하고 개념에 대해서만 설명하겠습니다.
## 유형 체계(Type system)

이전에 GraphQL 쿼리를 본 적이 있다면 GraphQL 쿼리 언어가 기본적으로 **객체에서 필드를 선택하는 언어**라는 것을 알고 계실 것입니다. 예를 들어 다음 쿼리에서:

```
{
  hero {
    name
    appearsIn
  }
}
```

```
{
  "data": {
    "hero": {
      "name": "R2-D2",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ]
    }
  }
}
```

1. 특별한 "root" 오브젝트로 시작합니다.
2. 그 객체에서 `hero` 필드를 선택합니다.
3. `hero`가 반환하는 객체에 대해 `name`과 `appearsIn` 필드를 선택합니다.

GraphQL 쿼리의 형태는 결과와 거의 일치하기 때문에 서버에 대해 잘 몰라도 쿼리가 무엇을 반환할지 예측할 수 있습니다. 하지만 요청할 수 있는 데이터에 대한 정확한 설명이 있으면 유용합니다. 어떤 필드를 선택할 수 있나요? 어떤 종류의 객체가 반환될 수 있나요? 해당 하위 객체에서 어떤 필드를 사용할 수 있나요? 바로 여기에 스키마가 필요합니다.

모든 GraphQL 서비스는 해당 서비스에서 쿼리할 수 있는 가능한 데이터 집합을 완전히 설명하는 유형 집합을 정의합니다. 그런 다음 쿼리가 들어올 때 해당 스키마에 대해 유효성을 검사하고 실행합니다.

#### 유형 언어 (Type Language)

GraphQL 서비스는 어떤 언어로든 작성할 수 있습니다. GraphQL 스키마에 대해 이야기하기 위해 JavaScript와 같은 특정 프로그래밍 언어 구문에 의존할 수 없으므로 자체적인 간단한 언어를 정의하겠습니다. 쿼리 언어와 유사하며 언어에 구애 받지 않고 GraphQL 스키마에 대해 이야기할 수 있는 "GraphQL 스키마 언어"를 사용하겠습니다.

#### 객체 유형 및 필드(Object types and fields)

GraphQL 스키마의 가장 기본적인 구성 요소는 서비스에서 가져올 수 있는 객체의 종류를 나타내는 객체 유형과 그 객체에 어떤 필드가 있는지를 나타내는 필드입니다. GraphQL 스키마 언어에서는 다음과 같이 표현할 수 있습니다:

```
type Character {
  name: String!
  appearsIn: [Episode!]!
}
```

이 언어는 꽤 읽기 쉽지만 어휘를 공유할 수 있도록 살펴 보겠습니다:

- `Character`는 GraphQL 객체 유형으로, 일부 필드가 있는 유형입니다. 스키마에 있는 대부분의 유형은 객체 유형입니다.
- `name` 및 `appearsIn`은 `Character` 유형의 필드입니다. 즉, 이름과 appearsIn은 Character 유형에서 작동하는 GraphQL 쿼리의 모든 부분에 나타날 수 있는 유일한 필드입니다.
- **문자열**은 **기본 제공 스칼라 유형** 중 하나로, **단일 스칼라 개체로 해석되는 유형**이며 쿼리에 하위 선택 항목을 가질 수 없습니다. 나중에 스칼라 유형에 대해 자세히 살펴보겠습니다.
- `String!` 은 필드가 `non-nullable`을 의미하며, 이 필드를 쿼리할 때 GraphQL 서비스가 항상 값을 제공하기로 약속한다는 의미입니다. 타입 언어에서는 느낌표로 표시하겠습니다.
- `[Episode!]!`는 에피소드 객체의 배열을 나타냅니다. 이 필드도 `non-nullable`이므로 나타나는 필드를 쿼리할 때 항상 (0개 이상의 항목이 포함된) 배열을 기대할 수 있습니다. 또한 `Episode!` 역시 널이 아니므로 배열의 모든 항목이 항상 `Episode` 객체일 것으로 예상할 수 있습니다.

이제 GraphQL 객체 유형이 어떻게 생겼는지, 그리고 GraphQL 유형 언어의 기본 사항을 읽는 방법을 알았습니다.

### 인수(Arguments)

GraphQL 객체 유형의 모든 필드에는 0개 이상의 인수가 있을 수 있습니다(예: 아래 `length` 필드):

```
type Starship {
  id: ID!
  name: String!
  length(unit: LengthUnit = METER): Float
}
```

모든 인수는 이름이 지정됩니다. 함수가 정렬된 인수의 목록을 취하는 JavaScript 및 Python과 같은 언어와 달리 GraphQL의 모든 인수는 구체적으로 이름으로 전달됩니다. 이 경우 `length` 필드에는 하나의 정의된 인자, `unit`이 있습니다.

인수는 필수 또는 선택 사항일 수 있습니다. 인수가 선택 사항인 경우 기본값을 정의할 수 있으며, 단위 인수가 전달되지 않으면 기본값은 `METER`로 설정됩니다.

#### 쿼리 및 뮤테이션 유형

스키마에 있는 대부분의 유형은 일반적인 객체 유형이지만 스키마 내에서 특별한 두 가지 유형이 있습니다:

```
query {
  hero {
    name
  }
  droid(id: "2000") {
    name
  }
}
```

```
{
  "data": {
    "hero": {
      "name": "R2-D2"
    },
    "droid": {
      "name": "C-3PO"
    }
  }
}
```

즉, GraphQL 서비스에는 `hero` 및 `droid` 필드가 있는 쿼리 유형이 있어야 합니다:

```
type Query {
  hero(episode: Episode): Character
  droid(id: ID!): Droid
}
```

뮤테이션도 비슷한 방식으로 작동합니다. 뮤테이션 유형에서 필드를 정의하면 쿼리에서 호출할 수 있는 루트 뮤테이션이 필드로 사용할 수 있습니다.

스키마로의 "entry point"이라는 특수한 상태를 제외하면 쿼리 및 뮤테이션 유형은 다른 GraphQL 객체 유형과 동일하며 해당 필드도 정확히 동일한 방식으로 작동한다는 점을 기억하는 것이 중요합니다.

#### 스칼라 유형

GraphQL 객체 유형에는 이름과 필드가 있지만, 어느 시점에서는 이러한 필드가 구체적인 데이터로 해석되어야 합니다. 바로 이때 스칼라 유형이 등장합니다. 스칼라 유형은 쿼리의 리프(leaves)를 나타냅니다.

다음 쿼리에서 이름과 appearsIn 필드는 스칼라 타입으로 해석됩니다:

```
  hero {
    name
    appearsIn
  }
}
```

```
{
  "data": {
    "hero": {
      "name": "R2-D2",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ]
    }
  }
}
```

이러한 필드에는 하위 필드가 없으며 쿼리의 리프(leaves)이기 때문에 이를 알 수 있습니다.

GraphQL에는 기본 스칼라 유형 세트가 기본적으로 제공됩니다:

- `Int`: 부호 있는 32비트 정수입니다.
- `Float`: 부호 있는 배정밀도 부동 소수점 값입니다.
- `String` UTF-8 문자 시퀀스입니다.
- `Boolean`: 참 또는 거짓입니다.
- `ID`: ID 스칼라 유형은 객체를 리페치하거나 캐시의 키로 자주 사용되는 고유 식별자를 나타냅니다. ID 유형은 문자열과 같은 방식으로 직렬화되지만, ID로 정의하면 사람이 읽을 수 없도록 되어 있음을 의미합니다.

대부분의 GraphQL 서비스 구현에는 사용자 정의 스칼라 유형을 지정하는 방법도 있습니다. 예를 들어 Date 유형을 정의할 수 있습니다:

```
scalar Date
```

그런 다음 해당 유형을 직렬화, 역직렬화 및 유효성 검사하는 방법을 정의하는 것은 구현에 달려 있습니다. 예를 들어, 날짜 유형이 항상 정수 타임스탬프로 직렬화되도록 지정할 수 있으며, 클라이언트는 모든 날짜 필드에 대해 해당 형식을 예상할 수 있어야 합니다.

#### 열거형 유형(Enumeration types)

열거형이라고도 하는 열거형 유형은 허용되는 특정 값 집합으로 제한되는 특별한 종류의 스칼라입니다. 이를 통해 다음을 수행할 수 있습니다:

- 이 유형의 인수가 허용된 값 중 하나인지 확인합니다.
- 유형 시스템을 통해 필드가 항상 유한한 값 집합 중 하나라는 것을 전달합니다.

열거형 정의는 GraphQL 스키마 언어에서 다음과 같이 보일 수 있습니다:

```
enum Episode {
  NEWHOPE
  EMPIRE
  JEDI
}
```

즉, 스키마에서 Episode 유형을 사용하는 곳이면 어디에서든 NEWHOPE, EMPIRE 또는 JEDI 중 하나가 될 것으로 예상합니다.

다양한 언어로 구현된 GraphQL 서비스에는 열거형을 처리하는 고유한 언어별 방법이 있다는 점에 유의하세요. 열거형을 기본으로 지원하는 언어에서는 구현이 이를 활용할 수 있지만, 열거형을 지원하지 않는 JavaScript와 같은 언어에서는 이러한 값이 내부적으로 정수 집합에 매핑될 수 있습니다. 그러나 이러한 세부 정보는 클라이언트로 유출되지 않으며, 클라이언트는 열거형 값의 문자열 이름으로만 작동할 수 있습니다.

#### Lists 및 Non-Null

객체 유형, 스칼라 및 열거형은 GraphQL에서 정의할 수 있는 유일한 유형입니다. 그러나 스키마의 다른 부분이나 쿼리 변수 선언에서 유형을 사용하는 경우 해당 값의 유효성 검사에 영향을 주는 추가 유형 수정자를 적용할 수 있습니다. 예를 살펴보겠습니다:

```
type Character {
  name: String!
  appearsIn: [Episode]!
}
```

여기서는 문자열 유형을 사용하고 유형 이름 뒤에 느낌표(!)를 추가하여 Non-Null로 표시하고 있습니다. 즉, 서버는 항상 이 필드에 대해 널이 아닌 값을 반환할 것으로 예상하고 있으며, 실제로 널 값을 반환하는 경우 GraphQL 실행 오류가 발생하여 클라이언트에게 문제가 발생했음을 알립니다.

필드에 대한 인수를 정의할 때도 Non-Null 유형 수정자를 사용할 수 있으며, 이 경우 GraphQL 문자열이나 변수에 null 값이 해당 인수로 전달되면 GraphQL 서버가 유효성 검사 오류를 반환하게 됩니다.

```
query DroidById($id: ID!) {
  droid(id: $id) {
    name
  }
}
```

```
{
  "id": null
}
```

```json
{
  "errors": [
    {
      "message": "Variable \"$id\" of non-null type \"ID!\" must not be null.",
      "locations": [
        {
          "line": 1,
          "column": 17
        }
      ]
    }
  ]
}
```

목록도 비슷한 방식으로 작동합니다: 유형 수정자(modifier)를 사용하여 유형을 목록으로 표시할 수 있으며, 이는 이 필드가 해당 유형의 배열을 반환할 것임을 나타냅니다. 스키마 언어에서는 대괄호인 `[` 및 `]`로 유형을 감싸서 표시합니다. 유효성 검사 단계에서 해당 값에 대한 배열을 기대하는 인수의 경우에도 동일하게 작동합니다.

`Non-Null` 및 `List` 수정자는 결합할 수 있습니다. 예를 들어, `Null`이 아닌 문자열 목록을 가질 수 있습니다:

```
myField: [String!]
```

즉, 목록 자체는 `Null`일 수 있지만 `Null` 멤버는 가질 수 없습니다. 예를 들어, JSON에서:

```
myField: null // valid
myField: [] // valid
myField: ["a", "b"] // valid
myField: ["a", null, "b"] // error
```

이제 문자열의 `Null`이 아닌 목록을 정의했다고 가정해 보겠습니다:

```
myField: [String]!
```

즉, 목록 자체는 `Null`이 될 수 없지만 `Null` 값을 포함할 수 있습니다:

```
myField: null // 오류
myField: [] // 유효
myField: ["a", "b"] // 유효
myField: ["a", null, "b"] // 유효
```

필요에 따라 `Nun-Null` 및 목록 수정자를 원하는 개수만큼 임의로 중첩할 수 있습니다.

### 인터페이스(Interfaces)

많은 유형 시스템과 마찬가지로 GraphQL은 인터페이스를 지원합니다. 인터페이스는 유형이 인터페이스를 구현하기 위해 포함해야 하는 특정 필드 집합을 포함하는 추상 유형입니다.

예를 들어, 스타워즈 3부작의 모든 캐릭터를 나타내는 `Character` 인터페이스를 가질 수 있습니다:

```
interface Character {
  id: ID!
  name: String!
  friends: [Character]
  appearsIn: [Episode]!
}
```

즉, `Character`를 구현하는 모든 유형에는 이러한 인자와 반환 유형(return type)을 가진 정확한 필드가 있어야 합니다.

예를 들어 `Character`를 구현할 수 있는 몇 가지 유형은 다음과 같습니다:

```
type Human implements Character {
  id: ID!
  name: String!
  friends: [Character]
  appearsIn: [Episode]!
  starships: [Starship]
  totalCredits: Int
}

type Droid implements Character {
  id: ID!
  name: String!
  friends: [Character]
  appearsIn: [Episode]!
  primaryFunction: String
}
```

이 두 가지 유형 모두 Character 인터페이스의 모든 필드를 가지고 있지만, 특정 캐릭터 유형에 특정한 추가 필드인 `totalCredits`, `starships` 및 `primaryFunction`도 가져오는 것을 볼 수 있습니다.

인터페이스는 객체 또는 객체 집합을 반환하려는 경우 유용하지만 여러 가지 유형이 있을 수 있습니다.

예를 들어 다음 쿼리는 오류를 생성합니다:

```
query HeroForEpisode($ep: Episode!) {
  hero(episode: $ep) {
    name
    primaryFunction
  }
}
```

```
{
  "ep": "JEDI"
}
```

```json
{
  "errors": [
    {
      "message": "Cannot query field \"primaryFunction\" on type \"Character\". Did you mean to use an inline fragment on \"Droid\"?",
      "locations": [
        {
          "line": 4,
          "column": 5
        }
      ]
    }
  ]
}
```

`hero` 필드는 `Character` 유형을 반환하므로 `Epsoide` 인수에 따라 `Human` 또는 `Droid가 될 수 있습니다. 위의 쿼리에서는 `primaryFunction`이 포함되지 않은 `Character` 인터페이스에 존재하는 필드만 요청할 수 있습니다.

특정 객체 유형에 대한 필드를 요청하려면 인라인 프래그먼트를 사용해야 합니다:

```
query HeroForEpisode($ep: Episode!) {
  hero(episode: $ep) {
    name
    ... on Droid {
      primaryFunction
    }
  }
}
```

```
{
  "ep": "JEDI"
}
```

```json
{
  "data": {
    "hero": {
      "name": "R2-D2",
      "primaryFunction": "Astromech"
    }
  }
}
```

쿼리 가이드의 [인라인 프래그먼트 섹션](https://graphql.org/learn/queries/#inline-fragments)에서 이에 대해 자세히 알아보세요.

#### 유니온 유형(Union types)

유니온 유형은 인터페이스와 매우 유사하지만 유형 간에 공통 필드를 지정할 수 없습니다.

```
union SearchResult = Human | Droid | Starship
```

스키마에서 `SearchResult` 유형을 반환할 때마다 `Human`, `Droid` 또는 `Startship`이 반환될 수 있습니다. 유니온 유형의 멤버는 구체적인 객체 유형이어야 하며, 인터페이스나 다른 유니온으로 유니온 유형을 만들 수 없다는 점에 유의하세요.

이 경우 `SearchResult` 유니온 유형을 반환하는 필드를 쿼리하는 경우 인라인 프래그먼트를 사용하여 모든 필드를 쿼리 할 수 있어야 합니다:

```
{
  search(text: "an") {
    __typename
    ... on Human {
      name
      height
    }
    ... on Droid {
      name
      primaryFunction
    }
    ... on Starship {
      name
      length
    }
  }
}
```

```json
{
  "data": {
    "search": [
      {
        "__typename": "Human",
        "name": "Han Solo",
        "height": 1.8
      },
      {
        "__typename": "Human",
        "name": "Leia Organa",
        "height": 1.5
      },
      {
        "__typename": "Starship",
        "name": "TIE Advanced x1",
        "length": 9.2
      }
    ]
  }
}
```

`__typename` 필드는 클라이언트에서 서로 다른 데이터 유형을 구분할 수 있는 문자열로 해석됩니다.

또한 이 경우 `Human`과 `Droid`는 공통 인터페이스(문자)를 공유하므로 여러 유형에서 동일한 필드를 반복하지 않고도 한 곳에서 공통 필드(common field)를 쿼리할 수 있습니다:

```
{
  search(text: "an") {
    __typename
    ... on Character {
      name
    }
    ... on Human {
      height
    }
    ... on Droid {
      primaryFunction
    }
    ... on Starship {
      name
      length
    }
  }
}
```

`Starship`이 `Character`가 아니기 때문에 `Starship`에 이름을 지정하지 않으면 결과에 표시되지 않으므로 주의하세요!

#### 입력 유형(Input type)

지금까지 열거형이나 문자열과 같은 스칼라 값을 필드에 인수로 전달하는 방법에 대해서만 설명했습니다. 하지만 복잡한 객체도 쉽게 전달할 수 있습니다. 이는 생성할 전체 객체를 전달해야 하는 뮤테이션의 경우에 특히 유용합니다. GraphQL 스키마 언어에서 입력 유형은 일반 객체 유형과 완전히 동일하게 보이지만 유형 대신 키워드 입력이 사용됩니다:

```
input ReviewInput {
  stars: Int!
  commentary: String
}
```

다음은 뮤테이션에서 입력 객체 유형을 사용하는 방법입니다:

```
mutation CreateReviewForEpisode($ep: Episode!, $review: ReviewInput!) {
  createReview(episode: $ep, review: $review) {
    stars
    commentary
  }
}
```

```
{
  "ep": "JEDI",
  "review": {
    "stars": 5,
    "commentary": "This is a great movie!"
  }
}
```

```json
{
  "data": {
    "createReview": {
      "stars": 5,
      "commentary": "This is a great movie!"
    }
  }
}
```

입력 객체 유형의 필드는 그 자체로 입력 객체 유형을 참조할 수 있지만 스키마에서 입력 및 출력 유형을 혼합할 수는 없습니다. 또한 입력 객체 유형은 필드에 인수를 가질 수 없습니다.

---

[Validation](003_validation.md)


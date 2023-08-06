[GraphQL](https://graphql.org/learn/)

## [Queries and Mutations](https://graphql.org/learn/queries/)

이 페이지에서는 GraphQL 서버를 쿼리하는 방법에 대해 자세히 설명합니다.

### 필드 (Fields)

가장 간단하게 GraphQL은 객체의 특정 필드를 요청하는 것입니다. 아주 간단한 쿼리와 이를 실행했을 때 얻을 수 있는 결과를 살펴보겠습니다:

```
{
  hero {
    name
  }
}
```

```json
{
  "data": {
    "hero": {
      "name": "R2-D2"
    }
  }
}
```

쿼리의 모양이 결과와 정확히 동일한 것을 즉시 확인할 수 있습니다. 이는 항상 예상한 것을 반환하고 서버가 클라이언트가 요청하는 필드를 정확히 알고 있기 때문에 GraphQL에 필수적입니다.

필드 이름은 문자열 유형, 이 경우 스타워즈의 주인공 `name`인 "R2-D2"를 반환합니다.

> 아, 한 가지 더 - 위의 쿼리는 대화형입니다. 즉, 원하는 대로 변경하고 새로운 결과를 확인할 수 있습니다. 쿼리의 `hero` 개체에 appearsIn 필드를 추가하고 새 결과를 확인해 보세요.

이전 예제에서는 `hero`의 `name`을 요청하여 문자열을 반환했지만 필드는 객체를 참조할 수도 있습니다. 이 경우 해당 객체에 대한 필드의 하위 선택을 만들 수 있습니다. GraphQL 쿼리는 관련 객체와 해당 필드를 순회할 수 있으므로 클라이언트는 기존 REST 아키텍처에서와 같이 여러 번 왕복하는 대신 한 번의 요청으로 많은 관련 데이터를 가져올 수 있습니다.

```
{
  hero {
    name
    # Queries can have comments!
    friends {
      name
    }
  }
}
```

```json
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

이 예제에서 `friends` 필드는 항목 배열을 반환합니다. GraphQL 쿼리는 단일 항목이나 항목 목록 모두 동일하게 보이지만 스키마에 표시된 내용을 기반으로 어떤 쿼리를 예상해야 하는지 알 수 있습니다.

### 인수 (Arguments)

객체와 그 필드를 트래버스하는 것만 가능했다면 GraphQL은 이미 데이터 가져오기에 매우 유용한 언어가 되었을 것입니다. 하지만 필드에 인수를 전달할 수 있는 기능을 추가하면 상황이 훨씬 더 흥미로워집니다.

```
{
  human(id: "1000") {
    name
    height
  }
}
```

```json
{
  "data": {
    "human": {
      "name": "Luke Skywalker",
      "height": 1.72
    }
  }
}
```

REST와 같은 시스템에서는 요청에 쿼리 매개변수와 URL 세그먼트라는 단일 인수 세트만 전달할 수 있습니다. 하지만 GraphQL에서는 모든 필드와 중첩된 객체가 고유한 인수 집합을 가져올 수 있으므로 GraphQL은 여러 API 가져오기를 완전히 대체할 수 있습니다. 심지어 인수를 스칼라 필드로 전달하여 모든 클라이언트에서 개별적으로 데이터 변환을 구현하는 대신 서버에서 한 번만 구현할 수도 있습니다.

```
{
  human(id: "1000") {
    name
    height(unit: FOOT)
  }
}
```

```json
{
  "data": {
    "human": {
      "name": "Luke Skywalker",
      "height": 5.6430448
    }
  }
}
```

인수는 다양한 유형이 될 수 있습니다. 위의 예에서는 한정된 옵션 집합(이 경우 길이 단위, 미터 또는 피트) 중 하나를 나타내는 열거 유형을 사용했습니다. GraphQL은 기본 유형 세트와 함께 제공되지만, 전송 형식으로 직렬화할 수 있는 경우 GraphQL 서버에서 자체 사용자 정의 유형을 선언할 수도 있습니다.

### 별칭 (Aliases)

눈썰미가 좋다면 결과 개체 필드가 쿼리의 필드 이름과 일치하지만 인수를 포함하지 않기 때문에 다른 인수를 사용하여 동일한 필드를 직접 쿼리할 수 없다는 것을 눈치채셨을 것입니다. 따라서 별칭이 필요한데, 별칭을 사용하면 필드 결과의 이름을 원하는 대로 바꿀 수 있습니다.

```
{
  empireHero: hero(episode: EMPIRE) {
    name
  }
  jediHero: hero(episode: JEDI) {
    name
  }
}
```

```
{
  "data": {
    "empireHero": {
      "name": "Luke Skywalker"
    },
    "jediHero": {
      "name": "R2-D2"
    }
  }
}
```

위의 예제에서는 두 `hero` 필드가 충돌할 수 있지만, 다른 이름으로 별칭을 지정할 수 있으므로 한 번의 요청으로 두 결과를 모두 얻을 수 있습니다.

### 프래그먼트 (Fragments)

앱에 두 명의 `hero`을 친구와 함께 나란히 볼 수 있는 비교적 복잡한 페이지가 있다고 가정해 봅시다. 이러한 쿼리는 비교의 각 측면에 대해 필드를 적어도 한 번씩 반복해야 하므로 금방 복잡해질 수 있습니다.

이것이 바로 GraphQL이 프래그먼트라는 재사용 가능한 단위를 포함하는 이유입니다. 프래그먼트를 사용하면 필드 집합을 구성한 다음 필요한 곳에 쿼리에 포함할 수 있습니다. 다음은 프래그먼트를 사용하여 위의 상황을 해결하는 방법의 예입니다:

```
{
  leftComparison: hero(episode: EMPIRE) {
    ...comparisonFields
  }
  rightComparison: hero(episode: JEDI) {
    ...comparisonFields
  }
}

fragment comparisonFields on Character {
  name
  appearsIn
  friends {
    name
  }
}
```

```
{
  "data": {
    "leftComparison": {
      "name": "Luke Skywalker",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ],
      "friends": [
        {
          "name": "Han Solo"
        },
        {
          "name": "Leia Organa"
        },
        {
          "name": "C-3PO"
        },
        {
          "name": "R2-D2"
        }
      ]
    },
    "rightComparison": {
      "name": "R2-D2",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ],
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

필드가 반복되는 경우 위의 쿼리가 얼마나 반복적인지 알 수 있습니다. 프래그먼트라는 개념은 복잡한 애플리케이션 데이터 요구 사항을 더 작은 덩어리로 분할하는 데 자주 사용되며, 특히 서로 다른 프래그먼트를 가진 많은 UI 구성 요소를 하나의 초기 데이터 가져오기로 결합해야 할 때 더욱 그렇습니다.

#### 프래그먼트 내에서 변수 사용

프래그먼트가 쿼리 또는 뮤테이션(mutation)에서 선언된 변수에 액세스할 수 있습니다. [변수](https://graphql.org/learn/queries/#variables)를 참조하십시오.

### 작업 이름(Operation name)

위의 몇 가지 예제에서는 `query` 키워드와 쿼리 이름을 모두 생략하는 약식 구문을 사용했지만, 프로덕션 앱에서는 코드를 덜 모호하게 만들기 위해 이러한 구문을 사용하는 것이 유용합니다.

다음은 `query` 키워드를 작업 유형으로, `HeroNameAndFriends`를 작업 이름으로 포함하는 예제입니다:

```
query HeroNameAndFriends {
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

작업 유형은 쿼리, 뮤테이션 또는 구독(subscription) 중 하나이며 수행하려는 작업 유형을 설명합니다. 쿼리 속기(shorthand) 구문을 사용하지 않는 한 작업 유형은 필수이며, 이 경우 작업의 이름이나 변수 정의를 제공할 수 없습니다.

작업 이름은 의미 있고 명시적인 작업 이름입니다. 다중 작업 문서에서만 필요하지만 디버깅 및 서버 측 로깅에 매우 유용하므로 사용을 권장합니다. 문제가 발생했을 때(네트워크 로그나 GraphQL 서버의 로그에 오류가 표시되는 경우) 내용을 해독하는 대신 코드베이스에서 쿼리를 이름으로 식별하는 것이 더 쉽습니다. 즐겨 사용하는 프로그래밍 언어의 함수 이름처럼 생각하면 됩니다. 예를 들어, JavaScript에서는 익명 함수로만 쉽게 작업할 수 있지만, 함수에 이름을 지정하면 함수를 추적하고 코드를 디버깅하고 호출 시기를 기록하기가 더 쉽습니다. 같은 방식으로, GraphQL 쿼리 및 뮤테이션 이름과 프래그먼트 이름은 서버 측에서 다양한 GraphQL 요청을 식별하는 데 유용한 디버깅 도구가 될 수 있습니다.

### 변수(Variables)

지금까지는 쿼리 문자열 안에 모든 인수를 작성했습니다. 하지만 대부분의 애플리케이션에서는 필드에 대한 인수가 동적입니다: 예를 들어, 관심 있는 스타워즈 에피소드를 선택할 수 있는 드롭다운이나 검색 필드, 필터 세트가 있을 수 있습니다.

이러한 동적 인수를 쿼리 문자열에 직접 전달하면 클라이언트 측 코드가 런타임에 쿼리 문자열을 동적으로 조작하고 GraphQL 전용 형식으로 직렬화해야 하므로 좋은 생각이 아닙니다. 대신 GraphQL은 쿼리에서 동적 값을 조작(manipulate)하여 별도의 딕셔너리로 전달할 수 있는 최상의 방법을 제공합니다. 이러한 값을 변수라고 합니다.

변수로 작업을 시작하면 세 가지 작업을 수행해야 합니다:

> 쿼리의 정적 값을 `$veriableName`으로 바꿉니다.
> 쿼리에서 허용되는 변수 중 하나로 `$variableName`을 선언합니다.
> 별도의 전송별(보통 JSON) 변수 사전에서 `variableName:` 값을 전달합니다.

전체적으로 살펴보면 다음과 같습니다:

```
query HeroNameAndFriends($episode: Episode) {
  hero(episode: $episode) {
    name
    friends {
      name
    }
  }
}
```

```
{
  "episode": "JEDI"
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

이제 클라이언트 코드에서 완전히 새로운 쿼리를 구성할 필요 없이 다른 변수를 전달하기만 하면 됩니다. 이는 일반적으로 쿼리에서 어떤 인수가 동적일 것으로 예상되는지를 나타내는 데에도 좋은 방법이며, 사용자가 제공한 값으로 쿼리를 구성하기 위해 문자열 보간(interpolation)을 수행해서는 안 됩니다.

#### 변수 정의 (Variable definitions)

변수 정의는 위 쿼리에서 `($episode: Episode)`와 같이 보이는 부분입니다. 이는 타입 언어의 함수에 대한 인수 정의와 동일하게 작동합니다. 여기에는 모든 변수가 나열되며, 그 앞에 `$`가 붙고 그 뒤에 해당 유형(이 경우 `Episode`)이 붙습니다.

선언된 모든 변수는 스칼라, 열거형 또는 입력 객체 유형이어야 합니다. 따라서 복잡한 객체를 필드에 전달하려면 서버에서 일치하는 입력 유형을 알아야 합니다. 스키마 페이지에서 입력 개체 유형에 대해 자세히 알아보세요.

변수 정의는 선택 사항일 수도 있고 필수 사항일 수도 있습니다. 위의 경우 에피소드 유형 옆에 `!` 표시가 없으므로 선택 사항입니다. 그러나 변수를 전달하는 필드에 널이 아닌 인수가 필요한 경우에는 변수도 필수여야 합니다.

이러한 변수 정의의 구문에 대해 자세히 알아보려면 GraphQL 스키마 언어를 학습하는 것이 유용합니다. 스키마 언어는 스키마 페이지에 자세히 설명되어 있습니다.

#### 변수 기본값 (Default variables)

유형 선언 뒤에 기본값을 추가하여 쿼리의 변수에 기본값을 할당할 수도 있습니다.

```
query HeroNameAndFriends($episode: Episode = JEDI) {
  hero(episode: $episode) {
    name
    friends {
      name
    }
  }
}
```

모든 변수에 기본값이 제공되면 변수를 전달하지 않고 쿼리를 호출할 수 있습니다. 변수 사전의 일부로 변수를 전달하면 해당 변수가 기본값을 재정의합니다.

### 지시어 (Directives)

위에서 변수를 사용하면 동적 쿼리를 구성하기 위해 수동 문자열 보간을 수행하지 않아도 되는 방법에 대해 설명했습니다. 인자로 변수를 전달하면 이러한 문제 중 상당 부분을 해결할 수 있지만, 변수를 사용하여 쿼리의 구조와 형태를 동적으로 변경하는 방법도 필요할 수 있습니다. 예를 들어 요약 보기와 상세 보기가 있고 한 보기가 다른 보기보다 더 많은 필드를 포함하는 UI 컴포넌트를 상상해 볼 수 있습니다.

이러한 컴포넌트에 대한 쿼리를 작성해 보겠습니다:

```
query Hero($episode: Episode, $withFriends: Boolean!) {
  hero(episode: $episode) {
    name
    friends @include(if: $withFriends) {
      name
    }
  }
}
```

```
{
  "episode": "JEDI",
  "withFriends": false
}
```

```json
{
  "data": {
    "hero": {
      "name": "R2-D2"
    }
  }
}
```

위의 변수를 편집하여 `withFriends`에 참을 전달하도록 변경하고 결과가 어떻게 달라지는지 확인해 보세요.

우리는 지시어라는 GraphQL의 새로운 기능을 사용해야 했습니다. 지시어는 필드 또는 프래그먼트 포함에 첨부할 수 있으며, 서버가 원하는 방식으로 쿼리 실행에 영향을 줄 수 있습니다. 핵심 GraphQL 사양에는 정확히 두 개의 지시어가 포함되어 있으며, 이 지시어는 사양을 준수하는 모든 GraphQL 서버 구현에서 지원되어야 합니다:

- include (if: Boolean) 인수가 참인 경우에만 이 필드를 결과에 포함합니다.
- skip (if: Boolean) 인자가 참이면 이 필드를 건너뜁니다.

지시어는 쿼리에서 필드를 추가하거나 제거하기 위해 문자열 조작을 수행해야 하는 상황에서 벗어나는 데 유용할 수 있습니다. 서버 구현에서는 완전히 새로운 지시문을 정의하여 실험적인 기능을 추가할 수도 있습니다.

### 뮤테이션 (Mutations)

GraphQL에 대한 대부분의 논의는 데이터 가져오기에 초점을 맞추고 있지만, 완전한 데이터 플랫폼에는 서버 측 데이터를 수정할 수 있는 방법도 필요합니다.

REST에서는 모든 요청이 서버에 부작용을 일으킬 수 있지만, 관례에 따라 데이터를 수정할 때 GET 요청을 사용하지 않는 것이 좋습니다. GraphQL도 비슷합니다. 기술적으로는 모든 쿼리가 데이터 쓰기를 유발하도록 구현될 수 있습니다. 그러나 쓰기를 유발하는 모든 연산은 변형을 통해 명시적으로 전송되어야 한다는 규칙을 설정하는 것이 유용합니다.

쿼리에서와 마찬가지로, 뮤테이션 필드가 객체 유형을 반환하는 경우 중첩된 필드를 요청할 수 있습니다. 이는 업데이트 후 객체의 새 상태를 가져올 때 유용할 수 있습니다. 간단한 뮤테이션 예제를 살펴보겠습니다:

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

`createReview` 필드가 새로 생성된 `review`의 `star` 및 `commentary` 필드를 반환하는 방식에 주목하세요. 한 번의 요청으로 필드의 새 값을 변경하고 쿼리할 수 있으므로 필드를 증분할 때와 같이 기존 데이터를 변경할 때 특히 유용합니다.

또한 이 예제에서 전달한 `review` 변수가 스칼라가 아니라는 것을 알 수 있습니다. 이 변수는 인자로 전달할 수 있는 특수한 종류의 객체 유형인 입력 객체 유형입니다. 스키마 페이지에서 입력 유형에 대해 자세히 알아보세요.

#### 뮤테이션의 여러 필드

뮤테이션은 쿼리와 마찬가지로 여러 필드를 포함할 수 있습니다. 쿼리와 뮤테이션 사이에는 이름 외에 한 가지 중요한 차이점이 있습니다:

**쿼리 필드는 병렬로 실행되는 반면, 뮤테이션 필드는 차례로 직렬로 실행됩니다.**

즉, 하나의 요청으로 두 개의 증분 크레딧 뮤테이션을 보내면 첫 번째 요청은 두 번째 요청이 시작되기 전에 완료되도록 보장되므로 서로 경합 조건이 발생하지 않습니다.

### 인라인 프래그먼트

다른 많은 유형 시스템과 마찬가지로 GraphQL 스키마에는 인터페이스 및 유니온 유형을 정의하는 기능이 포함되어 있습니다. [스키마 가이드에서 이에 대해 자세히 알아보세요.](https://graphql.org/learn/schema/#interfaces)

인터페이스 또는 유니온 유형을 반환하는 필드를 쿼리하는 경우 인라인 프래그먼트를 사용하여 기본 콘크리트 유형의 데이터에 액세스해야 합니다. 예제를 통해 가장 쉽게 이해할 수 있습니다:

```
query HeroForEpisode($ep: Episode!) {
  hero(episode: $ep) {
    name
    ... on Droid {
      primaryFunction
    }
    ... on Human {
      height
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

이 쿼리에서 `hero` 필드는 에피소드 인수에 따라 `Human` 또는 `Droid`가 될 수 있는 `Character` 유형을 반환합니다. 직접 선택에서는 `name`과 같이 `Character` 인터페이스에 존재하는 필드만 요청할 수 있습니다.

구체적인 유형에 대한 필드를 요청하려면 유형 조건이 있는 인라인 프래그먼트를 사용해야 합니다. 첫 번째 프래그먼트는 `Droid`에서 ...로 레이블이 지정되어 있으므로 `hero`에서 반환된 `Character`가 `Droid` 유형인 경우에만 `primaryFunction` 필드가 실행됩니다. `Human` 타입의 키 필드도 마찬가지입니다.

명명된 프래그먼트에는 항상 유형이 첨부되므로 명명된 프래그먼트도 같은 방식으로 사용할 수 있습니다.

#### 메타 필드

GraphQL 서비스에서 어떤 유형을 반환할지 모르는 경우가 있으므로 클라이언트에서 해당 데이터를 어떻게 처리할지 결정할 방법이 필요합니다. GraphQL을 사용하면 쿼리의 어느 지점에서든 메타 필드인 `__typename`을 요청하여 해당 지점의 개체 유형 이름을 가져올 수 있습니다.

```
{
  search(text: "an") {
    __typename
    ... on Human {
      name
    }
    ... on Droid {
      name
    }
    ... on Starship {
      name
    }
  }
}
```

```
{
  "data": {
    "search": [
      {
        "__typename": "Human",
        "name": "Han Solo"
      },
      {
        "__typename": "Human",
        "name": "Leia Organa"
      },
      {
        "__typename": "Starship",
        "name": "TIE Advanced x1"
      }
    ]
  }
}
```

위의 쿼리에서 검색은 세 가지 옵션 중 하나가 될 수 있는 유니온 유형을 반환합니다. 유형 이름 필드가 없으면 클라이언트와 다른 유형을 구분할 수 없습니다.

GraphQL 서비스는 몇 가지 메타 필드를 제공하며, 나머지는 인트로스펙션 시스템을 노출하는 데 사용됩니다.

---

[Schemas and Types](002_schemas_and_types.md)


[GraphQL](https://graphql.org/learn/)

# Validation

유형 시스템을 사용하면 GraphQL 쿼리가 유효한지 여부를 미리 결정할 수 있습니다. 이를 통해 서버와 클라이언트는 런타임 검사에 의존하지 않고도 유효하지 않은 쿼리가 생성되었을 때 개발자에게 효과적으로 알릴 수 있습니다.

스타워즈 예제의 경우 [`starWarsValidation-test.ts`](https://github.com/graphql/graphql-js/blob/main/src/__tests__/starWarsValidation-test.ts) 파일에는 다양한 유효성 오류를 보여주는 여러 쿼리가 포함되어 있으며, 참조 구현의 유효성 검사기를 실행하여 실행할 수 있는 테스트 파일입니다.

먼저 복잡한 유효성 검사 쿼리를 살펴보겠습니다. 이 쿼리는 이전 섹션의 예제와 유사한 중첩 쿼리이지만 중복된 필드가 프래그먼트로 분리되어 있습니다:

```
{
  hero {
    ...NameAndAppearances
    friends {
      ...NameAndAppearances
      friends {
        ...NameAndAppearances
      }
    }
  }
}

fragment NameAndAppearances on Character {
  name
  appearsIn
}
```

```json
{
  "data": {
    "hero": {
      "name": "R2-D2",
      "appearsIn": [
        "NEWHOPE",
        "EMPIRE",
        "JEDI"
      ],
      "friends": [
        {
          "name": "Luke Skywalker",
          "appearsIn": [
            "NEWHOPE",
            "EMPIRE",
            "JEDI"
          ],
          "friends": [
            {
              "name": "Han Solo",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "Leia Organa",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "C-3PO",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "R2-D2",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            }
          ]
        },
        {
          "name": "Han Solo",
          "appearsIn": [
            "NEWHOPE",
            "EMPIRE",
            "JEDI"
          ],
          "friends": [
            {
              "name": "Luke Skywalker",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "Leia Organa",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "R2-D2",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            }
          ]
        },
        {
          "name": "Leia Organa",
          "appearsIn": [
            "NEWHOPE",
            "EMPIRE",
            "JEDI"
          ],
          "friends": [
            {
              "name": "Luke Skywalker",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "Han Solo",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "C-3PO",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            },
            {
              "name": "R2-D2",
              "appearsIn": [
                "NEWHOPE",
                "EMPIRE",
                "JEDI"
              ]
            }
          ]
        }
      ]
    }
  }
}
```

그리고 이 쿼리는 유효합니다. 몇 가지 잘못된 쿼리를 살펴봅시다...

프래그먼트는 자신을 참조하거나 순환을 생성할 수 없으며, 이는 무한한 결과를 초래할 수 있습니다! 다음은 위와 동일한 쿼리이지만 명시적인 세 가지 수준의 중첩이 없는 쿼리입니다:

```
{
  hero {
    ...NameAndAppearancesAndFriends
  }
}

fragment NameAndAppearancesAndFriends on Character {
  name
  appearsIn
  friends {
    ...NameAndAppearancesAndFriends
  }
}
```

```json
{
  "errors": [
    {
      "message": "Cannot spread fragment \"NameAndAppearancesAndFriends\" within itself.",
      "locations": [
        {
          "line": 11,
          "column": 5
        }
      ]
    }
  ]
}
```

필드를 쿼리할 때는 주어진 유형에 존재하는 필드를 쿼리해야 합니다. 따라서 `hero`가 `Character`를 반환하면 `Character`의 필드를 쿼리해야 합니다. 해당 유형에는 `favoriteSpaceship` 필드가 없으므로 이 쿼리는 유효하지 않습니다: 

```
# INVALID: favoriteSpaceship does not exist on Character
{
  hero {
    favoriteSpaceship
  }
}
```

```json
{
  "errors": [
    {
      "message": "Cannot query field \"favoriteSpaceship\" on type \"Character\".",
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

필드를 쿼리할 때마다 스칼라나 열거형이 아닌 다른 것을 반환하는 경우, 필드에서 어떤 데이터를 반환할지 지정해야 합니다. `Hero`는 `Character`를 반환하고, 여기에 `name` 및 `appearsIn`과 같은 필드를 요청하고 있는데, 이를 생략하면 쿼리가 유효하지 않습니다:

```
# INVALID: hero is not a scalar, so fields are needed
{
  hero
}
```

```json
{
  "errors": [
    {
      "message": "Field \"hero\" of type \"Character\" must have a selection of subfields. Did you mean \"hero { ... }\"?",
      "locations": [
        {
          "line": 3,
          "column": 3
        }
      ]
    }
  ]
}
```

마찬가지로 필드가 스칼라인 경우 해당 필드에 대한 추가 필드를 쿼리하는 것은 의미가 없으며, 그렇게 하면 쿼리가 무효화됩니다:

```
# INVALID: name is a scalar, so fields are not permitted
{
  hero {
    name {
      firstCharacterOfName
    }
  }
}
```

```json
{
  "errors": [
    {
      "message": "Field \"name\" must not have a selection since type \"String!\" has no subfields.",
      "locations": [
        {
          "line": 4,
          "column": 10
        }
      ]
    }
  ]
}
```

앞서 쿼리는 해당 유형의 필드에 대해서만 쿼리할 수 있으며, `Character`를 반환하는 `hero`에 대해 쿼리할 때는 `Character`에 존재하는 필드에 대해서만 쿼리할 수 있다고 언급했습니다. 하지만 `R2-D2`의 `primaryFunction`에 대해 쿼리하려면 어떻게 해야 할까요?

```
# INVALID: primaryFunction does not exist on Character
{
  hero {
    name
    primaryFunction
  }
}
```

```json
{
  "errors": [
    {
      "message": "Cannot query field \"primaryFunction\" on type \"Character\". Did you mean to use an inline fragment on \"Droid\"?",
      "locations": [
        {
          "line": 5,
          "column": 5
        }
      ]
    }
  ]
}
```

`primaryFunction`이 `Character`의 필드가 아니므로 해당 쿼리는 유효하지 않습니다. `Character`가 드로이드인 경우 `primaryFunction`을 가져오고, 그렇지 않은 경우 해당 필드를 무시한다는 것을 나타내는 방법이 필요합니다. 이를 위해 앞서 소개한 프래그먼트를 사용할 수 있습니다. `Droid`에 정의된 조각을 설정하고 이를 포함시킴으로써 `primaryFunction`이 정의된 곳에서만 쿼리하도록 할 수 있습니다.

```
{
  hero {
    name
    ...DroidFields
  }
}

fragment DroidFields on Droid {
  primaryFunction
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

이 쿼리는 유효하지만 약간 장황합니다. 명명된 프래그먼트는 위에서 여러 번 사용할 때 유용했지만, 여기서는 한 번만 사용합니다. 명명된 프래그먼트를 사용하는 대신 인라인 프래그먼트를 사용하면 쿼리하는 유형을 표시할 수 있지만 별도의 프래그먼트에 이름을 지정하지 않아도 됩니다:

```
{
  hero {
    name
    ... on Droid {
      primaryFunction
    }
  }
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

이것은 유효성 검사 시스템의 표면적인 부분일 뿐이며, GraphQL 쿼리가 의미적으로 의미가 있는지 확인하기 위해 여러 가지 유효성 검사 규칙이 마련되어 있습니다. 사양에서는 ['유효성 검사' 섹션](https://github.com/graphql/graphql-js/tree/main/src/validation)에서 이 주제에 대해 자세히 설명하며, GraphQL.js의 유효성 검사 디렉터리에는 사양을 준수하는 GraphQL 유효성 검사기를 구현하는 코드가 포함되어 있습니다.

---

[Execution](004_execution.md)
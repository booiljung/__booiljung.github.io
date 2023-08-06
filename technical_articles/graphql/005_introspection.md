[GraphQL](https://graphql.org/learn/)

# 인트로스펙션(Introspection)

GraphQL 스키마가 지원하는 쿼리에 대한 정보를 요청하는 것은 종종 유용합니다. GraphQL을 사용하면 인트로스펙션 시스템을 사용하여 그렇게 할 수 있습니다!

스타워즈 예제의 경우, [starWarsIntrospection-test.ts](https://github.com/graphql/graphql-js/blob/main/src/__tests__/starWarsIntrospection-test.ts) 파일에는 인트로스펙션 시스템을 보여주는 여러 쿼리가 포함되어 있으며, 참조 구현의 인트로스펙션 시스템을 실행하여 실행할 수 있는 테스트 파일입니다.

타입 시스템을 설계했기 때문에 어떤 타입을 사용할 수 있는지 알 수 있지만, 모르는 경우 쿼리의 루트 타입에서 항상 사용할 수 있는 `__schema` 필드를 쿼리하여 GraphQL에 요청할 수 있습니다. 이제 그렇게 해서 어떤 타입을 사용할 수 있는지 물어보겠습니다.

```
{
  __schema {
    types {
      name
    }
  }
}

```

```json
{
  "data": {
    "__schema": {
      "types": [
        {
          "name": "Query"
        },
        {
          "name": "String"
        },
        {
          "name": "ID"
        },
        {
          "name": "Mutation"
        },
        {
          "name": "Episode"
        },
        {
          "name": "Character"
        },
        {
          "name": "Int"
        },
        {
          "name": "LengthUnit"
        },
        {
          "name": "Human"
        },
        {
          "name": "Float"
        },
        {
          "name": "Droid"
        },
        {
          "name": "FriendsConnection"
        },
        {
          "name": "FriendsEdge"
        },
        {
          "name": "PageInfo"
        },
        {
          "name": "Boolean"
        },
        {
          "name": "Review"
        },
        {
          "name": "ReviewInput"
        },
        {
          "name": "Starship"
        },
        {
          "name": "SearchResult"
        },
        {
          "name": "__Schema"
        },
        {
          "name": "__Type"
        },
        {
          "name": "__TypeKind"
        },
        {
          "name": "__Field"
        },
        {
          "name": "__InputValue"
        },
        {
          "name": "__EnumValue"
        },
        {
          "name": "__Directive"
        },
        {
          "name": "__DirectiveLocation"
        }
      ]
    }
  }
}
```

와, 종류가 정말 많네요! 어떤 것들이 있을까요? 그룹화해 봅시다:

- **`Query`, `Character`, `Human`, `Episod`, `Droid`** - 유형 시스템에서 정의한 유형입니다.
- **`String`, `Boolean`** - 유형 시스템에서 제공한 내장 스칼라입니다.
- **`__Schema`, `__Type`, `__TypeKind`, `__Field`, `__InputValue`, `__EnumValue`, `__Directive`** - 이들 모두 앞에 이중 밑줄이 표시되어 있어 인트로스펙션 시스템의 일부임을 알 수 있습니다.

이제 어떤 쿼리를 사용할 수 있는지 탐색을 시작하기에 좋은 곳을 찾아보겠습니다. 유형 시스템을 설계할 때 모든 쿼리가 어떤 유형에서 시작될지 지정했으니, 이제 인트로스펙션 시스템에 물어보겠습니다!

```js
{
  __schema {
    queryType {
      name
    }
  }
}
```

```json

{
  "data": {
    "__schema": {
      "queryType": {
        "name": "Query"
      }
    }
  }
}
```

이는 타입 시스템 섹션에서 `Query` 타입이 우리가 시작할 곳이라고 말한 것과 일치합니다! 여기서 이름을 지정한 것은 관례에 따른 것입니다. `Query` 유형에 다른 이름을 지정할 수 있으며, 쿼리의 시작 유형이라고 지정했더라도 여전히 여기에 반환되었을 것입니다. 하지만 `Query`로 명명하는 것은 유용한 규칙입니다.

하나의 특정 유형을 조사하는 것이 유용할 때가 많습니다. Droid` 유형을 살펴보겠습니다:

```js
{
  __type(name: "Droid") {
    name
  }
}
```

```json

{
  "data": {
    "__type": {
      "name": "Droid"
    }
  }
}
```

하지만 Droid에 대해 더 자세히 알고 싶다면 어떻게 해야 할까요? 예를 들어 인터페이스인가요, 아니면 객체인가요?

```
{
  __type(name: "Droid") {
    name
    kind
  }
}
```

```json

{
  "data": {
    "__type": {
      "name": "Droid",
      "kind": "OBJECT"
    }
  }
}
```

`kind`는 `__TypeKind` 열거형을 반환하며, 그 값 중 하나는 `OBJECT`입니다. 대신 `Character`에 대해 물어보면 인터페이스임을 알 수 있습니다:

```
{
  __type(name: "Character") {
    name
    kind
  }
}
```

```json

{
  "data": {
    "__type": {
      "name": "Character",
      "kind": "INTERFACE"
    }
  }
}
```

객체가 어떤 필드를 사용할 수 있는지 아는 것은 유용하므로 인트로스펙션 시스템에 '드로이드'에 대해 물어보겠습니다:

```
{
  __type(name: "Droid") {
    name
    fields {
      name
      type {
        name
        kind
      }
    }
  }
}
```

```json

{
  "data": {
    "__type": {
      "name": "Droid",
      "fields": [
        {
          "name": "id",
          "type": {
            "name": null,
            "kind": "NON_NULL"
          }
        },
        {
          "name": "name",
          "type": {
            "name": null,
            "kind": "NON_NULL"
          }
        },
        {
          "name": "friends",
          "type": {
            "name": null,
            "kind": "LIST"
          }
        },
        {
          "name": "friendsConnection",
          "type": {
            "name": null,
            "kind": "NON_NULL"
          }
        },
        {
          "name": "appearsIn",
          "type": {
            "name": null,
            "kind": "NON_NULL"
          }
        },
        {
          "name": "primaryFunction",
          "type": {
            "name": "String",
            "kind": "SCALAR"
          }
        }
      ]
    }
  }
}
```

이것들은 우리가 `Droid`에 정의한 필드입니다!

`id`가 조금 이상하게 보이는데, 유형에 대한 이름이 없습니다. 이는 `NON_NULL` 타입의 "래퍼(wrapper)" 타입이기 때문입니다. 해당 필드의 유형에 대해 `ofType`을 쿼리하면 `ID` 유형을 찾을 수 있으며, 이는 이것이 널이 아닌 `ID` 유형이라는 것을 알려줍니다.

마찬가지로, `friends`와 `appearsIn`은 모두 `LIST` 래퍼 유형이므로 이름이 없습니다. 이러한 유형에 대해 `ofType`을 쿼리하면 이것이 어떤 목록인지 알 수 있습니다.

```
{
  __type(name: "Droid") {
    name
    fields {
      name
      type {
        name
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

```json
{
  "data": {
    "__type": {
      "name": "Droid",
      "fields": [
        {
          "name": "id",
          "type": {
            "name": null,
            "kind": "NON_NULL",
            "ofType": {
              "name": "ID",
              "kind": "SCALAR"
            }
          }
        },
        {
          "name": "name",
          "type": {
            "name": null,
            "kind": "NON_NULL",
            "ofType": {
              "name": "String",
              "kind": "SCALAR"
            }
          }
        },
        {
          "name": "friends",
          "type": {
            "name": null,
            "kind": "LIST",
            "ofType": {
              "name": "Character",
              "kind": "INTERFACE"
            }
          }
        },
        {
          "name": "friendsConnection",
          "type": {
            "name": null,
            "kind": "NON_NULL",
            "ofType": {
              "name": "FriendsConnection",
              "kind": "OBJECT"
            }
          }
        },
        {
          "name": "appearsIn",
          "type": {
            "name": null,
            "kind": "NON_NULL",
            "ofType": {
              "name": null,
              "kind": "LIST"
            }
          }
        },
        {
          "name": "primaryFunction",
          "type": {
            "name": "String",
            "kind": "SCALAR",
            "ofType": null
          }
        }
      ]
    }
  }
}
```

툴링(Tooling)에 특히 유용한 인트로스펙션 시스템의 기능으로 끝을 맺으면서 시스템에 문서화를 요청해 보겠습니다!

```
{
  __type(name: "Droid") {
    name
    description
  }
}
```

```

{
  "data": {
    "__type": {
      "name": "Droid",
      "description": null
    }
  }
}
```

따라서 인트로스펙션을 사용하여 타입 시스템에 대한 문서에 액세스하고 문서 브라우저 또는 풍부한 IDE 환경을 만들 수 있습니다.

이것은 인트로스펙션 시스템의 표면적인 부분일 뿐이며, 열거형 값, 유형이 구현하는 인터페이스 등을 쿼리할 수 있습니다. 심지어 인트로스펙션 시스템 자체에 대한 인트로스펙션도 가능합니다. 사양은 ["인트로스펙션"](https://github.com/graphql/graphql-js/blob/main/src/type/introspection.ts) 섹션에서 이 주제에 대해 자세히 설명하며, GraphQL.js의 [introspection](https://github.com/graphql/graphql-js/blob/main/src/type/introspection.ts) 파일에는 사양을 준수하는 GraphQL 쿼리 인트로스펙션 시스템을 구현하는 코드가 포함되어 있습니다.

---

[GraphQL Best Practices](006_graphql_best_practices.md)


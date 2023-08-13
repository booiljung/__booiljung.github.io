[PostGraphile](https://www.graphile.org/)

## Inflection (인플렉션)

PostGraphile에서는 생성된 GraphQL 스키마에서 PostgreSQL의 사물이 어떻게 명명되는지 자세히 설명하는 "인플렉션" 개념이 있습니다.

PostGraphile의 기본 인플렉션은 명명 충돌을 피하려고 시도하는 동안 GraphQL의 자연 이름에 매핑을 시도합니다.  예:

- 테이블 이름이 단수화되어 UpperCamelCase로 변경됩니다: `pending_users` → `PendingUser`
- 열 이름이 camelCase 로 변경됩니다: `created_at` → `createdAt`
- 관계는 대상 유형과 참조 열을 참조합니다. `postsByAuthorId`(단축에 대해서는 아래의 "조언" 참조)

## Overriding Naming - One-off (이름 오버라이딩 - 일회성)

하나의 필드 또는 유형의 이름만 변경하려면 다음과 같이 표에 대해 [스마트 코멘트](https://www.graphile.org/postgraphile/smart-comments/)를 사용하는 것이 가장 좋습니다:

```postgresql
COMMENT ON TABLE post IS E'@name message';
```

참고: 이 경우에도 여전히 인플렉션을 사용하지만 테이블 이름이 다른 것처럼 위장하므로 인플렉션에 대한 입력이 다릅니다.

## Overriding Inflection - General (인플렉션 오버라이딩 - 일반)

플러그인으로 개별 인버터를 오버라이드 할 수 있습니다. 이렇게 하는 방법은 [`makeAddInflectorsPlugin` 글](https://www.graphile.org/postgraphile/make-add-inflectors-plugin/)에 설명되어 있습니다.

예제 플러그인은 다음과 같습니다:

```
module.exports = makeAddInflectorsPlugin(
  {
    patchType(typeName: string) {
      return this.upperCamelCase(`${typeName}-change-set`);
    },
  },
  true
);
```

[인플렉터 오버라이딩](https://www.graphile.org/postgraphile/make-add-inflectors-plugin/#where-are-the-default-inflectors-defined)도 함께 보시기 바랍니다.

## Advice (조언)

관계 필드 이름은 우발적 충돌을 방지하기 위해 매우 명확하며, 스키마를 매우 상세하게 만들 수 있습니다. 예를 들어 `userByAuthorId`, `userByEditorId`, `userByPublisherId` 등입니다.

일부 사람들은 이 장황함을 좋아하지만, 더 짧은 이름을 선호한다면 `@graphile-contrib/pg-simplify-inflector` 플러그인을 사용하는 것이 좋습니다. 이렇게 하면 해당 필드가 자동으로 저자, 편집자 및 게시자로 각각 지정됩니다.

```
postgraphile --append-plugins @graphile-contrib/pg-simplify-inflector
```

Benjie, 저는 모든 프로젝트에서 `pg-simplify-inflector`를 사용하는 것을 선호합니다.
[PostGraphile](https://www.graphile.org/)

# CRUD Mutations

CRUD, 즉 "Create, Read, Update, Delete"는 데이터 조작 API의 일반적인 패러다임이며, "CRUD 뮤테이션"은 "R"을 제외한 모든 것을 의미합니다. PostGraphile은 각 테이블의 스키마에 CRUD 뮤테이션을 자동으로 추가합니다. 모든 뮤테이션을 직접 정의하고 싶은 경우(예: [커스텀 뮤테이션](https://www.graphile.org/postgraphile/custom-mutations/)을 사용하는 경우) `--disable-default-mutations` CLI 설정(또는 `disableDefaultMutations: true` 라이브러리 설정)을 통해 이 동작을 비활성화할 수 있습니다.

[상위 문서](https://www.graphile.org/postgraphile/tables/)의 `users` 테이블을 사용하면 사용하는 PostGraphile 설정(및 부여한 권한)에 따라 다음과 같은 뮤테이션이 발생할 수 있습니다:

- `createUser` - 단일 `User`를 만듭니다. [예시를 참조하세요](https://www.graphile.org/postgraphile/examples/#Mutations__Create).
- `updateUser` - 전역적으로 고유한 ID와 패치를 사용하여 단일 `User`를 업데이트합니다.
- `updateUserById` - 고유 키와 패치를 사용하여 단일 `User`를 업데이트합니다. [예시를 참조하세요](https://www.graphile.org/postgraphile/examples/#Mutations__Update).
- `updateUserByUsername` - 고유 키와 패치를 사용하여 단일 `User`를 업데이트합니다.
- `deleteUser` - 전역적으로 고유한 ID를 사용하여 단일 `User`를 삭제합니다.
- `deleteUserById` - 고유 키를 사용하여 단일 `User`를 삭제합니다. [예시를 참조하세요](https://www.graphile.org/postgraphile/examples/#Mutations__Delete).
- `deleteUserByUsername` - 고유 키를 사용하여 단일 `User`를 삭제합니다.

테이블에 기본 키 열이 포함된 경우에만 `update` 및 `delete` 뮤테이션이 생성됩니다.

또한 다음과 같은 쿼리 필드("Read")가 제공됩니다:

- `user` - 전역적으로 고유한 `ID`를 사용하여 단일 `User`를 반환합니다.
- `userById` - 전역적으로 고유한 `ID`를 사용하여 단일 `User`를 읽습니다.
- `userByUsername` - 고유한 `username`을 사용하여 단일 `User`를 읽습니다.
- `allUsers` - (visible) `User` 집합을 통해 페이지 매김을 가능하게 하는 [커넥션](https://www.graphile.org/postgraphile/connections/)을 반환합니다.

## 예제

```postgresql
# Create a User and get back details of the record we created
mutation {
  createUser(
    input: { user: { id: 1, name: "Bilbo Baggins", username: "bilbo" } }
  ) {
    user {
      id
      name
      username
      createdAt
    }
  }
}

# Update Bilbo using the user.id primary key
mutation {
  updateUserById(
    input: { id: 1, userPatch: { about: "An adventurous hobbit" } }
  ) {
    user {
      id
      name
      username
      about
      createdAt
    }
  }
}

# Delete Bilbo using the unique user.username column and return the mutation ID
mutation {
  deleteUserByUsername(input: { username: "bilbo" }) {
    deletedUserId
  }
}
```

## 뮤테이션이 나타나지 않는다면...

먼저 PostGraphile 서버에서 오류가 출력되는지 확인합니다. 오류가 없다면, 생성된 스키마에 뮤테이션이 표시되지 않을 수 있는 몇 가지 이유가 있습니다:

- `--disable-default-mutations`(또는 `-M`) 지정 (또는 라이브러리에 지정)되는 경우
-  테이블에 `@omit create,update,delete` 스마트 코멘트가 있는 경우
- 테이블에 대한 권한이 충분하지 않고 `--no-ignore-rbac`이 지정되는 경우
- 노출된 스키마에 없는 테이블
- 테이블 대신 뷰
- 기본 키가 누락됨 (이 경우에도 `create` 뮤테이션은 계속 추가됨)
- 기본 키를 사용하는 뮤테이션만 표시되는 경우: `PrimaryKeyMutationsOnlyPlugin`을 사용하고 있는 경우 일 수 있습니다.

이러한 설정과 관련된 `.postgraphilerc`도 확인하는 것을 잊지 마세요!

GraphQL을 처음 사용하신다면 잘못된 곳을 찾고 계신 건 아닌가요? GraphiQL 인터페이스에서 오른쪽에 있는 문서를 열고 루트로 이동합니다. `Mutation` 유형을 선택하면 사용 가능한 뮤테이션을 확인할 수 있습니다. 자동 완성 사용 등 뮤테이션을 실행하려는 경우 요청을 작성할 때 `mutation` 작업 유형을 사용해야 합니다:

```graphql
mutation {
  createThing...
}
```

그렇지 않으면 GraphiQL이 요청을 쿼리로 해석합니다.


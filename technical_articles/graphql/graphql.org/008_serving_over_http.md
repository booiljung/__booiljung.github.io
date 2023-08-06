[GraphQL](https://graphql.org/learn/)

# Serving over HTTP

HTTP는 그 보편성 때문에 GraphQL을 사용할 때 클라이언트-서버 프로토콜로 가장 일반적으로 선택됩니다. 다음은 HTTP를 통해 작동하도록 GraphQL 서버를 설정하기 위한 몇 가지 지침입니다.

## Web Request Pipeline

대부분의 최신 웹 프레임워크는 요청이 미들웨어 스택(일명 필터/플러그인)을 통해 전달되는 파이프라인 모델을 사용합니다. 요청이 파이프라인을 통과할 때 요청을 검사, 변환, 수정하거나 응답을 통해 종료할 수 있습니다. GraphQL은 모든 인증 미들웨어 뒤에 배치해야 HTTP 엔드포인트 핸들러에서와 동일한 세션 및 사용자 정보에 액세스할 수 있습니다.

## URIs, Routes

HTTP는 일반적으로 "리소스"를 핵심 개념으로 사용하는 REST와 연관되어 있습니다. 이와 대조적으로 GraphQL의 개념 모델은 엔티티 그래프입니다. 따라서 GraphQL의 엔티티는 URL로 식별되지 않습니다. 대신 GraphQL 서버는 단일 URL/엔드포인트(일반적으로 `/graphql`)에서 작동하며, 특정 서비스에 대한 모든 GraphQL 요청은 이 엔드포인트로 전달되어야 합니다.

## HTTP Methods, Headers, and Body

GraphQL HTTP 서버는 HTTP GET 및 POST 메서드를 처리해야 합니다.

### GET request

HTTP GET 요청을 수신할 때 "쿼리" 쿼리 문자열에 GraphQL 쿼리를 지정해야 합니다. 예를 들어, 다음과 같은 GraphQL 쿼리를 실행하고자 한다고 가정해 보겠습니다:

```
{
  me {
    name
  }
}
```

이 요청은 다음과 같이 HTTP GET을 통해 전송할 수 있습니다:

```
http://myapi/graphql?query={me{name}}
```

쿼리 변수는 `변수`라는 추가 쿼리 매개변수를 통해 JSON 인코딩된 문자열로 전송할 수 있습니다. 쿼리에 여러 개의 명명된 연산이 포함된 경우, `operationName` 쿼리 매개변수를 사용하여 실행할 연산을 제어할 수 있습니다.

### POST request

표준 GraphQL POST 요청은 'application/json' 콘텐츠 유형을 사용해야 하며, 다음 형식의 JSON 인코딩된 본문을 포함해야 합니다:

```
{
  "query": "...",
  "operationName": "...",
  "variables": { "myVariable": "someValue", ... }
}
```

작업 이름` 및 `변수`는 선택적 필드입니다. 작업 이름`은 쿼리에 여러 작업이 있는 경우에만 필요합니다.

## Response

쿼리와 변수가 전송된 방식에 관계없이 응답은 요청 본문에 JSON 형식으로 반환되어야 합니다. 사양에서 언급했듯이 쿼리에서 일부 데이터와 일부 오류가 발생할 수 있으며, 이러한 오류는 해당 형식의 JSON 객체로 반환되어야 합니다:

```
{
  "data": { ... },
  "errors": [ ... ]
}
```

반환된 오류가 없는 경우, `"errors"` 필드는 응답에 없어야 합니다. [GraphQL 사양](https://spec.graphql.org/October2021/#sec-Data)에 따라 데이터가 반환되지 않는 경우, `"data"` 필드는 실행 중에 오류가 발생하지 않은 경우에만 포함되어야 합니다.

## GraphiQL

GraphiQL은 테스트 및 개발 중에 유용하지만 프로덕션 환경에서는 기본적으로 비활성화해야 합니다. `express-graphql`을 사용하는 경우 `NODE_ENV` 환경 변수를 기반으로 토글할 수 있습니다:

```
app.use('/graphql', graphqlHTTP({
  schema: MySessionAwareGraphQLSchema,
  graphiql: process.env.NODE_ENV === 'development',
}));
```

## Node

NodeJS를 사용하는 경우 [서버 구현 목록](https://graphql.org/code/#javascript-server)을 참조하는 것이 좋습니다.

## Draft Transport Specification

자세한 [HTTP 전송 사양](https://github.com/graphql/graphql-over-http)이 개발 중입니다. 아직 확정되지는 않았지만, 이 초안 사양은 GraphQL 클라이언트 및 라이브러리 유지 관리자를 위한 단일 소스 역할을 하며 HTTP 전송을 사용하여 GraphQL API를 노출하고 소비하는 방법을 자세히 설명합니다. 언어 사양과 달리 준수가 의무 사항은 아니지만, 대부분의 구현은 상호 운용성을 극대화하기 위해 이러한 표준을 따르고 있습니다.

---

[Authorization](009_authorization.md)


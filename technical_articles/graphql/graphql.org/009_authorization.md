[GraphQL](https://graphql.org/learn/)

# 권한부여(Authorization)

> 권한부여 로직을 비즈니스 로직 계층에 위임하기

권한부여는 특정 사용자/세션/컨텍스트가 작업을 수행하거나 데이터를 볼 수 있는 권한(permission)이 있는지 여부를 설명하는 비즈니스 로직의 한 유형입니다. 예를 들어

*"작성자만 초안을 볼 수 있음"*

이러한 종류의 동작을 적용하는 것은 [비즈니스 로직 계층](https://graphql.org/learn/thinking-in-graphs/#business-logic-layer)에서 이루어져야 합니다. 이렇게 GraphQL 계층에 권한부여 로직을 배치하고 싶은 유혹이 있습니다:

```
var postType = new GraphQLObjectType({
  name: ‘Post’,
  fields: {
    body: {
      type: GraphQLString,
      resolve: (post, args, context, { rootValue }) => {
        // return the post body only if the user is the post's author
        if (context.user && (context.user.id === post.authorId)) {
          return post.body;
        }
        return null;
      }
    }
  }
});
```

글의 `authorId` 필드가 현재 사용자의 `id`와 일치하는지 확인하여 '작성자가 글을 소유하고 있음(author owns a post)'을 정의합니다. 문제를 발견할 수 있나요? 서비스의 각 진입점에 대해 이 코드를 복제해야 합니다. 권한부여 로직이 완벽하게 동기화되지 않으면 사용자는 사용하는 API에 따라 다른 데이터를 볼 수 있습니다. 이런! 권한부여에 대한 [단일 소스](https://graphql.org/learn/thinking-in-graphs/#business-logic-layer)를 사용하면 이러한 문제를 방지할 수 있습니다.

리졸버 내부에서 권한부여 로직을 정의하는 것은 GraphQL을 학습하거나 프로토타이핑할 때 괜찮습니다. 그러나 프로덕션 코드베이스의 경우 권한부여 로직을 비즈니스 로직 계층에 위임하세요. 다음은 예시입니다:

```
//Authorization logic lives inside postRepository
var postRepository = require('postRepository');

var postType = new GraphQLObjectType({
  name: ‘Post’,
  fields: {
    body: {
      type: GraphQLString,
      resolve: (post, args, context, { rootValue }) => {
        return postRepository.getBody(context.user, post);
      }
    }
  }
});
```

위의 예에서는 비즈니스 로직 계층에서 호출자(caller)가 사용자 객체를 제공해야 한다는 것을 알 수 있습니다. GraphQL.js를 사용하는 경우, 사용자 객체는 리졸버의 네 번째 인수의 `context` 인자 또는 `rootValue`에 채워져야 합니다.

불투명한 토큰이나 API 키 대신 완전히 수화된(fully-hydrated) User 객체를 비즈니스 로직 계층에 전달하는 것이 좋습니다. 이렇게 하면 요청 처리 파이프라인의 여러 단계에서 [인증(authentication)](https://graphql.org/graphql-js/authentication-and-express-middleware/)과 권한 부여(authorization)라는 별개의 문제를 처리할 수 있습니다.

---

[Pagination](010_pagination.md)


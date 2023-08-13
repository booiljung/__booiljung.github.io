[Postgraphile](https://www.graphile.org/postgraphile/quick-start-guide/)

# Example queries and mutations

*GraphQL을 처음 사용하시는 분은 [여기](https://graphql.org/learn/)에서 GraphQL에 대한 공식 소개를 읽어보신 후 PostGraphile 문서를 계속 읽어보시길 권장합니다.*

아래에서는 [예제 리포지토리 스키마](https://github.com/graphile/examples/tree/master/db)에 대해 다양한 GraphQL 쿼리를 실행한 결과를 확인할 수 있습니다. 이는 소개 및 빠른 참조를 위한 것으로, 자세한 내용은 문서 링크를 참조하시기 바랍니다.

이 예제에서는 기본값보다 필드 이름을 간소화하기 위해 [@graphile-contrib/pg-simplify-inflector](https://github.com/graphile-contrib/pg-simplify-inflector) 플러그인을 사용한다는 점에 유의하시기 바랍니다. 다음 명령을 통해 사용할 수 있습니다:

```sh
yarn add postgraphile @graphile-contrib/pg-simplify-inflector
yarn postgraphile --append-plugins @graphile-contrib/pg-simplify-inflector
```

[마크다운으로 스크립트 표현을 하지 못하므로 예제는 원문 참조 하기시 바랍니다.](https://www.graphile.org/postgraphile/examples/#Basic__Forums)


[PostGraphile](https://www.graphile.org/)

## Database Functions

포스트그래파일 스키마에 더 많은 기능을 추가하는 가장 쉬운 방법 중 하나는 PostgreSQL 함수를 사용하는 것입니다. 세 가지 주요 방법은 다음과 같습니다:

- [컴퓨트 컬럼](https://www.graphile.org/postgraphile/computed-columns/)을 사용하면 테이블 유형에 계산된 필드를 추가할 수 있습니다.
- [커스텀 쿼리](https://www.graphile.org/postgraphile/custom-queries/)를 사용하면 스칼라, 목록, 사용자 정의 유형, 테이블 행 또는 테이블 커넥션을 반환할 수 있는 루트 수준 쿼리 필드를 추가할 수 있습니다.
- [사용자 지정 뮤테이션](https://www.graphile.org/postgraphile/custom-mutations/)를 사용하면 데이터베이스를 수정하고 아무것도 반환하지 않을 수 있는 루트 수준 뮤테이션 필드(`void`), 스칼라, 목록, 사용자 지정 유형, 테이블 행 또는 테이블 행 목록을 추가할 수 있습니다 (단, 뮤테이션에 대해 페이지 매김할 수 없으므로 커넥션은 반환하지 않음).

### 함수 이해하기 (Understanding Functions)

가장 강력한 PostGraphile 서버를 만들려면 PostgreSQL 함수를 이해하는 것이 매우 중요합니다. 함수를 사용하면 SQL 또는 다른 여러 스크립팅 언어 중 하나를 사용하여 데이터베이스에서 비즈니스 로직을 정의할 수 있습니다. 비즈니스 로직을 데이터베이스에 넣는 것이 데이터 집약적인 용도에 맞게 세밀하게 조정된 PostgreSQL을 애플리케이션 계층에 넣는 것보다 성능이 더 좋은 경우가 많습니다.

일부 함수 예제는 [포럼 예제 SQL 스키마](https://github.com/graphile/postgraphile/blob/v4/examples/forum/schema.sql)를 참조하세요.

### 추천 자료 (Recommended Reading)

- 실제로 함수를 생성하기 위한 PostgreSQL [`CREATE FUNCTION`](http://www.postgresql.org/docs/current/static/sql-createfunction.html) 설명서를 참조하세요.
- PostgreSQL [`CREATE TRIGGER`](http://www.postgresql.org/docs/current/static/sql-createtrigger.html) 문서.
- PostgreSQL의 [컴퓨트 열](http://stackoverflow.com/a/11166268/1568890)을 설명하는 Stackoverflow 답변.



### 절차적 언어 (Procedural Languages)

PostgreSQL의 함수를 사용하려면 SQL 또는 절차적 언어를 사용해야 합니다. PostgreSQL에서 가장 일반적인 절차적 언어는 [PL/pgSQL](http://www.postgresql.org/docs/current/static/plpgsql.html)입니다. SQL은 이미 익숙할 가능성이 높으므로 사용하기 가장 쉽습니다. PL/pgSQL은 PostgreSQL의 사용자 정의 절차적 언어로, 배우기가 매우 쉬우며 이 언어에 대한 StackOverflow 및 기타 리소스가 많이 있습니다. 트리거에는 SQL을 사용할 수 없으므로 트리거를 작성하려면 PL/pgSQL(또는 다른 절차적 언어 중 하나)을 배워야 합니다. 걱정하지 마세요. PL/pgSQL에 대한 깊은 지식이 없어도 멋진 애플리케이션을 만들 수 있습니다.

`LANGUAGE sql`로 작성된 간단한 함수는 다음과 같습니다:

```sql
CREATE FUNCTION add(a int, b int) RETURNS int AS $$
  select a + b;
$$ LANGUAGE sql IMMUTABLE STRICT;
```

`LANGUAGE plpgsql`을 사용하는 동일한 함수는 다음과 같이 보일 수 있습니다:

```sql
CREATE FUNCTION add(a int, b int) RETURNS int AS $$
BEGIN
  RETURN a + b;
END;
$$ LANGUAGE plpgsql IMMUTABLE STRICT;
```

PL/pgSQL 또는 SQL을 사용하고 싶지 않다면 PostgreSQL 내부에서 널리 사용되는 여러 스크립팅 언어를 사용하여 함수를 작성할 수 있습니다! 여기에서 이러한 프로젝트 중 몇 가지를 볼 수 있습니다:

- [JavaScript (plv8)](https://github.com/plv8/plv8)
- [Ruby(plruby)](https://github.com/knu/postgresql-plruby)

예를 들어 JavaScript를 사용하여 정의한 함수는 다음과 같습니다:

```sql
-- This does look the exact same as the PL/pgSQL example…
CREATE FUNCTION add(a int, b int) RETURNS int AS $$
  return a + b;
$$ LANGUAGE plv8 IMMUTABLE STRICT;

-- Here’s a better example from the plv8 repo…
CREATE FUNCTION plv8_test(keys text[], vals text[]) RETURNS text AS $$
  var object = {}
  for (var i = 0; i < keys.length; i++) {
    object[keys[i]] = vals[i]
  }
  return JSON.stringify(object)
$$ LANGUAGE plv8 IMMUTABLE STRICT;
```

### 명명된 인수 (Named Arguments)

PostgreSQL allows you to mix named and positional (unnamed) arguments in your functions. However, GraphQL will *only* allow named arguments. So if you don’t name an argument, PostGraphile will give it a name like `arg1`, `arg2`, `arg3`, and so on. An example of a function with unnamed arguments is as follows:

PostgreSQL을 사용하면 함수에서 이름 있는 인수와 위치(이름 없는) 인수를 혼합할 수 있습니다. 그러나 GraphQL은 이름 있는 인수만 허용합니다. 따라서 인수의 이름을 지정하지 않으면 PostGraphile은 'arg1', 'arg2', 'arg3' 등의 이름을 부여합니다. 이름 없는 인수를 가진 함수의 예는 다음과 같습니다:

```sql
CREATE FUNCTION add(int, int) RETURNS int AS $$
  SELECT $1 + $2;
$$ LANGUAGE sql IMMUTABLE STRICT;
```

반면 명명된 인수는 다음과 같습니다:

```sql
CREATE FUNCTION add(a int, b int) RETURNS int AS $$
  select a + b;
$$ LANGUAGE sql IMMUTABLE STRICT;
```

### Solving naming conflicts (이름 충돌 해결)

함수 매개변수에 대해 선택한 이름이 함수 내에서 액세스할 수 있는 열 이름이나 기타 식별자와 충돌하는 경우가 있습니다.

이러한 충돌을 피하려면 첫 번째 인수는 `$1`, 두 번째 인수는 `$2`와 같은 [숫자 인수](https://www.postgresql.org/docs/current/xfunc-sql.html#XFUNC-SQL-FUNCTION-ARGUMENTS)를 사용하고 테이블 이름을 사용하여 모호하지 않게 할 수 있습니다:

```sql
create function get_user(id int) returns users as $$
  select * from users where users.id = $1;
$$ language sql stable;
```

또는 숫자 `$n` 인수 대신 인수 이름을 사용하는 것을 선호하는 경우 함수 이름을 사용하여 명확하게 구분할 수 있습니다:

```sql
create function get_user(id int) returns users as $$
  select * from users where users.id = get_user.id;
$$ language sql stable;
```

이것은 일반적으로 잘 작동하지만 충분하지 않은 경우가 있습니다. 예를 들어 `plpgsql` 언어 함수에 업서트(`INSERT...ON CONFLICT`) 문이 있는 경우와 같은 경우입니다:

```sql
create function upsert_value(id int, value text) returns void as $$
begin
  insert into my_table (id, value)
    values(id, value)
    on conflict (id) -- This will error
    do update set value = excluded.value;
end;
$$ language plpgsql volatile;
```

In this case the `on conflict (id)` causes an issue because PL/pgSQL does not know if `id` refers to the table column or the function argument, and adding the table name inside the parenthesis is a syntax error.

To solve this, you can change language to `sql` which will treat columns preferentially. Alternatively you can tell the function to solve conflicts by using the column:

이 경우 `on conflict (id)`은 `id`가 테이블 열을 가리키는지 함수 인수를 가리키는지 PL/pgSQL이 알지 못하고 괄호 안에 테이블 이름을 추가하면 구문 오류가 발생하기 때문에 문제를 일으킵니다.

이 문제를 해결하려면 열을 우선적으로 처리하는 `sql`로 언어를 변경하면 됩니다. 또는 함수에 열을 사용하여 충돌을 해결하도록 지시할 수 있습니다:

```sql
create function upsert_value(id int, value text) returns void as $$
#variable_conflict use_column
begin
  insert into my_table (id, value)
    values(id, value)
    on conflict (id)
    do update set value = excluded.value;
end;
$$ language plpgsql volatile;
```

이러한 충돌과 해결 방법을 더 잘 이해하려면 [변수 치환](https://www.postgresql.org/docs/current/plpgsql-implementation.html#PLPGSQL-VAR-SUBST)에 대한 PostgreSQL 문서를 참조하세요.

### 휘발성(뮤테이션) 함수 (VOLATILE (Mutation) Functions)

기본적으로 함수는 "휘발성"입니다. 예를 들어 함수는 다음과 같이 정의됩니다:

```sql
CREATE FUNCTION my_function(a int, b int) RETURNS int AS $$ … $$ LANGUAGE sql;
```

로 정의된 함수와 동일합니다:

```sql
CREATE FUNCTION my_function(a int, b int) RETURNS int AS $$ … $$ LANGUAGE sql VOLATILE;
```

PostgreSQL 문서에서 가져옵니다:

> `VOLATILE` indicates that the function value can change even within a single table scan, so no optimizations can be made… But note that any function that has side-effects must be classified volatile, even if its result is quite predictable, to prevent calls from being optimized away; an example is `setval()`.
>
> `VOLATILE`은 단일 테이블 스캔 내에서도 함수 값이 변경될 수 있으므로 최적화를 수행할 수 없음을 나타냅니다... 그러나 결과가 예측 가능하더라도 부작용이 있는 함수는 반드시 volatile으로 분류하여 호출이 최적화되지 않도록 해야 합니다 (예를 들어 `setval()`이 있습니다).

In simpler terms `VOLATILE` basically means you are changing data or storing state.

Anyone familiar with HTTP could compare a `VOLATILE` function to “unsafe” HTTP methods like `POST`, `PUT`, `PATCH`, and `DELETE`.

Certain VOLATILE functions간단히 말해서 `VOLATILE`은 기본적으로 데이터를 변경하거나 상태를 저장하는 것을 의미합니다.

HTTP에 익숙한 사람이라면 누구나 `VOLATILE` 함수를 `POST`, `PUT`, `PATCH`, `DELETE`와 같은 "안전하지 않은" HTTP 메서드와 비교할 수 있습니다.

특정 VOLATILE 함수는 PostGraphine에 의해 [커스텀 뮤테이션](https://www.graphile.org/postgraphile/custom-mutations/)으로 노출됩니다.

### 안정/불안정(쿼리) 함수 (STABLE/IMMUTABLE (Query) Functions)

함수가 데이터나 상태를 수정하지 않는다면 `STABLE`로 선언해야 합니다. (함수가 인자에만 의존하고 테이블과 같은 다른 소스에서 데이터를 가져오지 않는 경우, `STABLE`의 더 엄격한 형태인 `IMMUTABLE`로 선언할 수 있습니다.)

함수를 `STABLE` 또는 `IMMUTABLE`로 표시하면 PostgreSQL은 동일한 문에서 동일한 입력에 대해 여러 번 호출하지 않도록 메모화(memoization)를 비롯한 여러 가지 최적화를 적용할 수 있다는 것을 알고 있습니다.

다음은 함수를 STABLE/IMMUTABLE로 정의하는 예제입니다:

```sql
CREATE FUNCTION my_function(a int, b int) RETURNS int AS $$ … $$ LANGUAGE sql STABLE;

-- or…

CREATE FUNCTION my_function(a int, b int) RETURNS int AS $$ … $$ LANGUAGE sql IMMUTABLE;

-- or if you wanted to return a row from a table…

CREATE FUNCTION my_function(a int, b int) RETURNS my_table AS $$ … $$ LANGUAGE sql STABLE;
```

PostgreSQL 문서에서 가져옵니다:

> `IMMUTABLE`은 함수가 데이터베이스를 수정할 수 없으며 동일한 인수 값이 주어질 때 항상 동일한 결과를 반환한다는 것을 나타냅니다. 즉, 데이터베이스 조회를 수행하거나 인수 목록에 직접 존재하지 않는 정보를 사용하지 않는다는 의미입니다. 이 옵션을 지정하면 모든 상수 인수가 있는 함수를 호출하면 즉시 함수 값으로 대체할 수 있습니다.

그리고…

> `STABLE`은 함수가 데이터베이스를 수정할 수 없으며, 단일 테이블 스캔 내에서 동일한 인수 값에 대해 일관되게 동일한 결과를 반환하지만, 그 결과는 SQL 문에 따라 변경될 수 있음을 나타냅니다. 이는 데이터베이스 조회, 매개변수 변수(예: 현재 시간대) 등에 따라 결과가 달라지는 함수에 적합한 선택입니다(현재 명령에 의해 수정된 행을 쿼리하려는 AFTER 트리거에는 부적절합니다).

HTTP 비유를 다시 사용하자면, `IMMUTABLE`과 `STABLE`은 `GET`과 `HEAD`와 같은 "safe" HTTP 메서드에 비유할 수 있습니다.

특정 STABLE/IMMUTABLE 함수는 포스트그래파일에서 [커스텀 쿼리](https://www.graphile.org/postgraphile/custom-queries/) 또는 [컴퓨트 컬럼](https://www.graphile.org/postgraphile/computed-columns/)로 노출됩니다.

### SETOF 함수 -  커넥션 (SETOF Functions - Connections)

스칼라, 복합 유형 및 이들의 배열뿐만 아니라 PostgreSQL 함수는 집합도 반환할 수 있습니다. 집합은 테이블을 에뮬레이트하므로, [커넥션](https://www.graphile.org/postgraphile/connections/)을 사용하여 집합을 GraphQL에 노출하는 것은 당연한 일입니다.

SETOF 함수는 사용자가 한 번에 처리하기에는 너무 많아 페이지 매김이 필요할 수 있는 데이터를 사용자에게 노출하는 강력한 방법이 될 수 있습니다.

커넥션을 반환하는 함수를 만들려면 다음과 같은 SQL을 사용할 수 있습니다:

```sql
-- Assuming we already have a table named `person`…
CREATE FUNCTION my_function(a int, b int) RETURNS SETOF person AS $$ … $$ LANGUAGE sql STABLE;
```

이 함수는 [사용자 지정 쿼리](https://www.graphile.org/postgraphile/custom-queries/)로 인식되며, 다음과 같이 쿼리할 수 있습니다:

```graphql
{
  myFunction(a: 1, b: 2, first: 2) {    pageInfo {
      hasNextPage
      hasPrevPage
    }
    edges {
      cursor
      node {
        id
      }
    }
  }
}
```

고급 쿼리 구성에 대한 자세한 내용은 [여기](https://www.graphile.org/postgraphile/custom-queries/)에서 확인할 수 있습니다.

------

*This article was originally written by [Caleb Meredith](https://twitter.com/calebmer) but has been heavily edited since.*
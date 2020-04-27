# SQL anywhere Quick Tutorial

### 예약어

식별자가 예약어와 동일하면 `""`로 식별자를 감싼다. 아래 예제는 테이블 이름이 `SELECT`인 경우 예약어와 동일하기 때문에 감싼다.

```
SELECT * FROM "SELECT"
```

예약어 목록을 얻으려면 아래와 같이 한다.

```
SELECT * FROM sa_reserved_words() ORDER BY reserved_word;
```

### 식별자

식별자 최대 길이는 128 바이트이다.

알파벳, 숫자, _, @, #, $ 문자로 구성된다.

큰따옴표, 제어문자, 백슬래시, 대괄호, 역따옴표는 사용할 수 없다.

선행 또는 후행 공백, 작은 따옴표, 세미콜론은 사용자 이름이나 역할 이름으로 사용할 수 없다.

`quoted_identifier` 데이터베이스 옵션이 Off로 설정된 경우 큰 따옴표는 SQL 문자열을 구분하며 식별자를 구분하는 데 사용할 수 없다. `quoted_identifier`의 설정에 관계없이 대괄호 또는 역 따옴표를 사용하여 식별자를 구분할 수 있다. `quoted_identifier` 옵션의 기본 설정은 Open Client 및 jConnect 연결의 경우 `Off`이다. 그렇지 않으면 기본값은 `On`이다.

### 간접 식별자

런타임시 오브젝트 이름을 판별해야하거나 명령문에 기본 오브젝트의 이름이 노출되지 않도록하려면 간접 식별자를 사용 한다.
`@myVariable`은 작업중인 실제 개체의 이름을 저장하는 기존 변수의 이름이다. 여기서 대괄호와 백 따옴표 (예 : `[@myVariable]`)로 묶어 명령문에 간접 식별자를 지정한다.장된 값으로 대체되고 권한 검사가 수행됩니다.

간접 식별자 값의 최대 길이는 128 바이트이며 `CHAR`, `VARCHAR` 또는 `LONG VARCHAR` 유형일 수 있다.

다음 예는 `GROUPO.Employees` 테이블에서 열 이름 (`Surname`)을 보유하기 위해 `@col`이라는 변수를 작성한다. `SELECT` 문은 간접 식별자 ( `'[@col]'`)를 지정하여 `Employees.Surname` 열의 내용을 쿼리한다.

```sql
CREATE OR REPLACE VARIABLE @col LONG VARCHAR = 'Surname';
SELECT E.'[@col]' FROM GROUPO.Employees E;
```

다음 예는 간접 식별자를 사용하여 테이블을 쿼리하는 방법을 보여준다

```sql
CREATE OR REPLACE VARIABLE t_owner LONG VARCHAR = 'GROUPO';
CREATE OR REPLACE VARIABLE t_name LONG VARCHAR = 'Employees';
SELECT * FROM '[t_owner]'.'[t_name]';
```

다음 예는 테이블 참조를 사용하는 `IN` 매개 변수 (`@tableref`), 열의 이름을 사용하는 `IN` 매개 변수 (@columnname) 및 `IN`을 반영하는 정수 값을 사용하는 `IN` 매개 변수 (`@value`)를 사용하여 프로 시저를 작성한다.

```sql
CREATE PROCEDURE mydelete( IN @tableref TABLE REF,
                           IN @columnname LONG VARCHAR,
                           IN @value INT )
SQL SECURITY INVOKER
NO RESULT SET
BEGIN
    DELETE FROM TABLE REF (@tableref) AS T WHERE T.'[@columnname]' = @value;
END;

CALL mydelete( TABLE REF ( FTEmployee ), 'employee_id', @employee_to_delete);
CALL mydelete( TABLE REF ( FTStudent ), 'student_id',  @student_to_delete);
```

### String

2GB  크기를 가질 수 있다.

`CHAR`  또는 `NCHAR` 타입이 있다.

`'Hello world'`는 `CHAR`타입의 String이며 길이는 11이다.

`NCHAR`로 하려면 `N'Hello world'`로 하거나 `'Hello world' as NCHAR`로 한다.

하나의 String에 대해 byte-length 또는 character-length 두가지 길이가 있다. CP1252의 경우 byte-length와 character-length는 같다.

### Binary 리터럴

`0~9`, `A~F(a~f)`로 구성되며 `0x`로 시작하며 짝수 길이를 가져야 한다. 예를 들어 `39`는 16진수 `0027`이며 `0x0027`로 표현한다.

`CAST` 함수를 사용하여 변환 할 수 있다. 예를 들어.

```
SELECT CAST (0x0080000001 AS INT);
```

`HEXTOINT` 함수도 있다.

```
SELECT HEXTOINT( '0xFF80000001' );
```

### 관계 연산자

```
=
>
<
>=
<=
!=
<>
!>
!<
```

### 논리 연산자

```
... WHERE condition1 AND condition2
... WHERE condition1 OR condition2
... WHERE NOT condition
... IS [NOT] truth-value
```

### 산술연산자

```
expression + expression
expression - expression
-expression
expression * expression
expression / expression
expression % expression
```

### 문자열 연산자

```
expression || expression	; 문자열을 연결한다. NULL이면 빈 문자열로 대체 된다.
expression + expression		; 문자열을 연결한다.
```

### OPENXML 연산자

XML을 파싱하여 쿼리 문으로 대체 한다.

### 배열 연산자

### UNNEST 배열 연산자

### 비트열 연산자

```
&	; bitwise AND
|	; bitwise OR
^	; bitwise XOR
~	; bitwise NOT
```

### 연산자 우선 순위

1. 단항 연산자
2. &, |, ^, ~
3. *, /, %
4. +, -
5. ||
6. NOT
7. AND
8. OR

### 상수식

숫자 또는 문자열.

### 컬럼 이름

코럴레이션 이름은 선택적이다.

대개 테이블 이름이 코럴레이션 이름이다.

다른 문자를 포함하고 있으면 `"`로 감싸야 한다.

```
Employees.Name
address
"date hired"
"salary"."date paid"
```

### 하위 질의 (subquery)

괄호 `(...)`로 감싸야 한다.

### IF 식

식이 `TRUE`, `FALSE` 또는 `UNKNOWN`인지 판별 한다.

```
IF condition
THEN expression
[ ELSE expression2 ]
{ ENDIF | END IF }
```

condition이 UNKNOWN이면  `NULL`을 반환 한다.

IF 식은 IF 문과 다르다는 점을 주의 한다.

### CASE 식


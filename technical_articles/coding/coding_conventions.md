# 코딩 컨벤션

## 변수의 이름

- 변수는 상태를 나타낸다.
- 명사를 사용할 수 있다.
- 형용사를 사용할 수 있다.
- 부사를 사용할 수 있다.
- 과거 분사를 사용할 수 있다.
- 현재 진행형을 사용할 수 있다.
- 집합 변수는 복수형으로 나타낸다.

| 상태               | 집합 변수   | 의미                                                | 때로       |
| ------------------ | ----------- | --------------------------------------------------- | ---------- |
| count              |             | 셀 수 있는 수량                                     |            |
| size               |             | 셀 수 있는 수량                                     |            |
| length             |             | 셀 수 있는 수량                                     |            |
| color              | colors      | 색상                                                |            |
| message            | messages    | 메시지                                              |            |
| people             | peoples     | 사람                                                | is_opened  |
| opened             |             | 열린 상태                                           | is_closed  |
| closed             |             | 닫힌 상태                                           |            |
| running            |             | 구동 상태                                           |            |
| enabled, activated |             | 활성화된 상태                                       |            |
| sending            |             | 전송중                                              | is_sending |
| sent               |             | 전송됨, 전송된 것                                   |            |
| available          |             | 가능한 상태                                         |            |
| ~~number~~         | ~~numbers~~ | 많은 경우 변수는 수를 나타내므로 number는 의미 없다 |            |

## 함수 또는 메소드의 이름

- 동사로 시작 한다.

| 상태                | 집합에 대한 동작   | 의미          |
| ------------------- | ------------------ | ------------- |
| add                 | append, add_all    | 추가하라      |
| insert              | insert, insert_all | 삽입하라      |
| remove              | remove_range       | 제거하라      |
| open                |                    | 자원 열어라   |
| close               |                    | 자원을 닫아라 |
| enable, activate    |                    | 활성화 하라   |
| disable, inactivate |                    | 비활성화 하라 |
| send                |                    | 전송하라      |

## 표기법

### 스네이크 케이스 (snake case)

언더바(_) 가 들어 있는 표현 방식을 뱀처럼 생겼다고 하여 스네이크 케이스라고 한다. 

```
int snake_case;
```

### 파스칼 케이스 (pascal case)

첫 글자와 중간 글자들이 대문자인 경우 파스칼 언어의 표기법과 유사하다고 하여 파스칼 케이스라고 한다.

```
int PascalCase;
```

### 카멜 케이스 (camel case)

중간 글자들은 대문자로 시작하지만 첫 글자가 소문자인 경우에는 낙타와 모양이 비슷하다 하여 카멜 케이스라고 한다. 

```
int camelCase;
```

## 어떤 컨벤션을 선택할 것인가?

- 치열한 논쟁의 대상
- 하지만 이미 정해져 있다.
  - 언어마다 이미 제안되어 있다.
  - 문제는  프레임워크별로 다양한 컨벤션을 가진 C/C++ 언어
  - 프레임워크를 따라가는 방법
  - stl을 따라가는 방법

| 언어         | 컨벤션                                                       |
| ------------ | ------------------------------------------------------------ |
| 파이썬       | [PEP 8 – Style Guide for Python Code](https://peps.python.org/pep-0008/)<br>[Google Python Style Guide](https://google.github.io/styleguide/pyguide.html) |
| 자바스크립트 | [Google JavaScript Style Guide](https://google.github.io/styleguide/jsguide.html)<br>[Airbnb JavaScript Style Guide](https://github.com/airbnb/javascript)<br>[w3school JavaScript Style Guide](https://www.w3schools.com/js/js_conventions.asp) |
| 자바         | [Google Java Style Guide](https://google.github.io/styleguide/javaguide.html)<br>[Oracle Java Style Guide](https://www.oracle.com/technetwork/java/codeconventions-150003.pdf)<br>[코넬 대학교 Java 스타일 가이드](https://www.cs.cornell.edu/courses/JavaAndDS/JavaStyle.html) |
| C#           | [마이크로소프트 C# Coding Conventions](https://docs.microsoft.com/en-us/dotnet/csharp/fundamentals/coding-style/coding-conventions)<br>[C# at Google Style Guild](https://google.github.io/styleguide/csharp-style.html) |
| Dart         | [Efffective Dart: Style](https://dart.dev/guides/language/effective-dart/style) |
| Shell        | [Google Shell Style Guide](https://google.github.io/styleguide/shellguide.html) |
| C++          | [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)<br>[스타일 체크 도구: cpplint](https://github.com/cpplint/cpplint) |
| TypeScript   | [Google TypeScript Style Guide](https://google.github.io/styleguide/tsguide.html) |


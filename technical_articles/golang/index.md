# Golang

개발 환경은 vscode .devcontainers 사용

## Hello, World!

새 폴더:

```sh
mkdir -p hello
cd hello
```

**모듈 생성:**

```sh
go mode init <폴더경로>
```

예: `example/hello` 모듈 생성:

```sh
go mod init example/hello
```

출력: `go.mod` 파일 생성 된다:

```sh
module example/hello

go 1.20
```

**프로그램 예제:** `hello.go` 파일 작성:

```go
package main

import "fmt"

func main() {
	fmt.Println("Hello, World!")
}
```

**main 함수 실행:**

```sh
go run .
```

실행 결과:

```
Hello, World!
```

## 외부 패키지 사용

새 폴더:

```sh
mkdir -p quote
cd quote
```

**패키지 검색 및 확인:**

[search for a "quote" package](https://pkg.go.dev/search?q=quote):

특정 버전 패키지 지정 - `rsc.io/quote/v4`

최신 버전 패키지 지정 - `rsc.io/quote`

`example/quote` 모듈 생성:

```sh
go mod init example/quote
```

`quote.go` 파일 작성:

```go
package main

import (
	"fmt"
	"rsc.io/quote"
)

func main() {
	fmt.Println(quote.Go())
}
```

**사용된 모듈 추가:**

```sh
go mod tidy
```

**main 함수 실행:**

```sh
go run .
```

실행 결과:

```
Don't communicate by sharing memory, share memory by communicating.
```

## Golang Syntax

### 패키지:

첫 라인에서 패키지 이름 정하기:

```go
package <패키지 이름>
```

패키지 참조: https://pkg.go.dev/

```go
import {
	<참조할 패키지 1>
	<참조할 패키지 2>
	...
	<참조할 패키지 3>
}

import "<참조할 패키지 4>"
import "<참조할 패키지 5>"
```

자동으로 `pkg/mod`에 다운로드된 패키지 삭제:

```sh
go clean -modcache
```

### 기본 타입:

```
bool
string
int		int8	int16	int32	int64
uint	uint8	uint16	uint32	uint64	uintptr
byte
rune
float32		float64
complex64	complex128
```

## 변수:

```go
var <변수> <타입> = [초기값]
```

**0으로 초기화:**

```go
var i int
var f float64
var b bool
var s string
fmt.Println("%v %v %v %q", i, f, b, s)
```

결과:

```
0 0 0 0
```

**명시적 초기화:**

```
var i, j int = 1, 2
```

**대입:**

```
k := 3
```

### 함수:

```go
func <함수명>([매개변수 목록]) <반환타입> {
    ...
}
```

매개변수 목록:

```go
func myfunc(x int, y int) int {
	...
}
```

 타입을 생략하여 짧게 줄이기:

```go
func myfunc(x, y int) int {
	...
}
```

**다중 반환:**

```go
func swap(x, y string) (string, string) {
	return y, x
}
func main() {
    a, b := swap("hello", "world")
    print.Println(a, b)
}
```

**명시적 타입 변환:**

```go
var i int = 42
var f float64 = float64(i)
var u uint = uint(f)
```

**심플 명시적 타입 변환:**

```go
i := 42
f := float64(i)
u := uint(f)
```

**묵시적 변수 타입:**

```go
var i int
j := i
```

여기서 `j`는 `int` 타입.

묵시적 변수 타입 예제:

```go
i := 42				// int
f := 3.142			// float64
g := 0.867 + 0.5i	// complex128
```

**상수는 const와 대문자로 시작:**

```go
cont Pi = 3.14
const World = 'World'
const Truth = true
```

## 제어문

**for:**

```
for [변수 := 초기값]; <조건>; [증분] {
	...
}
```

**while:**

```go
for <조건> {
    ...
}
```

**forever loop:**

```go
for {
    ...
}
```

**if: 조건만 있는 경우:**

```go
if <조건> {
    ...
}
```

if: 변수 및 조건까지 있는 경우:

```go
if [변수 := 초기값;] <조건> {
    ...
}
```

if else:

```go
if [변수 := 초기값;] <조건> {
    ...
} else {
	...
}
```

**switch:**

```go
switch [변수 := 초기값;] [식] {
case 식1:
    ...
case 식2:
    ...
default:
    ...
}
```

Golang은 C와 다르게 아래 case를 실행하지 않는다.

case 에 식이 오므로 함수 등도 사용할 수 있으며 위에서 아래로 실행되다가 참이면 해당  case를 실행하고 멈춘다.

**무조건 switch:**

```go
switch {
    
}
```

### Defer 지연실행:

Defer 문은 스택에 푸시되고, 둘러싸인 함수가 리턴될때까지 실행이 지연된다. 인자는 즉시 전달되지만 함수는 실행되지 않는다. LIFO로 실행된다.

```go
{
    defer fmt.Println("world")
	fmt.Println("hello")
}
```

복수의 Deffer:

```go
{
	fmt.Println("counting")

	for i := 0; i < 10; i++ {
		defer fmt.Println(i)
	}

	fmt.Println("done")
}
```

### 포인터

포인터는 메모리 주소를 가리킨다. `*T`는 `T` 타입 값을 가리킨다. 제로 포인터는 `nil`이다.

```go
var p *int
```

`&` 연산자는 포인터를 생성한다:

```go
i = 42
p = &i
```

`*`연산자는 포인터가 가리키는 값을 말한다:

```go
fmt.Println(*p)
*p = 21
```

C언어와 다르게 포인터에 대한 연산을 할 수 없다.

### 구조체

```go
type Vertex struct {
    X int
    Y int
}
v := Vertex{1, 2}
v.X = 4
```

포인터의 경우 멤버 접근이 C언어와 다르다:

```
p := &v
p.X = 1e9
```

구조체 리터럴:

```
type Vertex struct {
	X, Y int
}

v1 = Vertex{1, 2}
v2 = Vertex{X: 1}	// Y:0 묵시적
v3 = Vertex{}		// X:0 and Y:0 묵시적
p = &Vertex{1, 2}
```

### 배열:

배열 타입 `[n]T`은 `n`개의 `T` 타입이다.

```go
var a [10]int
```

배열 길이는 타입의 일부로 크기를 변경할 수 없다.

```go
var a [2]string
a[0] = "Hello"
a[1] = "World"
primes := [6]int[2, 3, 5, 7, 11, 13]
```

슬라이스: `[low : high]`

슬라이스는 새로 저장하지 않고 참조 한다.

```go
names := [4]string{
    "John",
    "Paul",
    "George",
    "Ringo",
} // names=[John Paul George Ringo]

a := names[0:2] // a=[John Paul]
b := names[1:3] // b=[Paul George]

b[0] = "XXX"    // a=[John XXX]
				// b=[XXX George]
// names=[John XXX George Ringo]
```

**슬라이스 리터럴:**

배열

```go
[3]bool { true, true, false }
```

과 

```go
[]bool { true, true, false }
```

는 같다.

다음 배열에 대한

```go
var a [10]int
```

다음 슬라이스는 같다.

```go
a[0:10]
a[:10]
a[0:]
a[:]
```

슬라이스 길이(length) 및 용량(capacity):

```go
s := []int{2, 3, 5, 7, 11, 13}	// len=6 cap=6 s=[2 3 5 7 11 13]
s = s[:0]						// len=0 cap=6 s=[]
s = s[:4]						// len=4 cap=6 s=[2 3 5 7]
s = s[2:]						// len=2 cap=4 s=[5 7]
```

슬라이스가 배열에 대한 min, max를 가지는 참조라는 것을 알 수 있다.

**`nil` 슬라이스:**

배열을 가리키지 않는 `nil`슬라이스는 길이와 용량이 0이다.

내장 `make` 함수로 동적 길이를 가지는 슬라이스를 만들 수 있다.

```go
a := make([]int, 5)		// len(a)=5
```

용량도 지정 할 수 있다.

```go
b := make([]int, 0, 5)	// len(b)=0, cap(b)=5
b = b[:cap(b)]	// len(b)=5, cap(b)=5
b = b[1:]		// len(b)=4, cap(b)=4
```

**슬라이스에 대한 슬라이스:**

...

**슬라이스에 덧붙이기:**

```go
var s []int				// len=0 cap=0 s=[]
s = append(s, 0)		// len=1 cap=1 s=[0]
s = append(s, 1)		// len=2 cap=2 s=[0 1]
s = append(s, 2, 3, 4)	// len=5 cap=6 s=[0 1 2 3 4]
```

**range:** 슬라이스나 맵에 대해 순환:

```go
var pow = []int{1, 2, 4, 8, 16, 32, 64, 128}
for i, v := range pow {
    fmt.Printf("2**%d = %d\n", i, v)
}
```

range 변수 생략:

```
for i, _ := range pow
for _, v := range pow
for i, := range pow
```

### 맵

https://go.dev/tour/moretypes/19







### 출력:

문자열 출력:

```go
import "fmt"
fmt.Println(<문자열>)
```


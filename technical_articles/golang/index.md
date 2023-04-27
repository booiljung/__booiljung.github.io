# Golang

개발 환경은 vscode .devcontainers 사용

## Hello, World!

새 폴더:

```
mkdir -p hello
cd hello
```

`example/hello` 모듈 생성:

```
go mod init example/hello
```

`go.mod` 파일 생성 됨:

```
module example/hello

go 1.20
```

`hello.go` 파일 작성:

```go
package main

import "fmt"

func main() {
	fmt.Println("Hello, World!")
}
```

실행:

```
go run .
```

결과:

```
Hello, World!
```

## 외부 패키지 사용

새 폴더:

```
mkdir -p quote
cd quote
```

패키지 확인:

[search for a "quote" package](https://pkg.go.dev/search?q=quote):

특정 버전 지정 - `rsc.io/quote/v4`

현재 최신 버전 - `rsc.io/quote`

`example/quote` 모듈 생성:

```
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

사용된 모듈 추가:

```
go mod tidy
```

실행:

```
go run .
```

결과:

```
Don't communicate by sharing memory, share memory by communicating.
```






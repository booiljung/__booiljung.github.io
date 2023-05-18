# HTTP with Boost Beast

C++, Boost Beast로 개발하는  HTTP

#### c++ async

https://hwan-shell.tistory.com/199를 이해하기 위해 정리

##### std::async

- `std::async`는 `std::task` 클래스 기반으로 만들어진 클래스로 쓰레드 사용.
- `std::async`는 `std::thread`와 달리 쓰레드 풀을 통해 쓰레드 사용.
- `std::thread`보다 쉽고 편리.
- `std::async`는 반환 값은 `std::future`로 받음.

##### std::async constructor

```c++
#include <iostream>
#include <future>

void for_print(char c) {
  	for (int i = 0; i < 100; i++)
    	printf("%c번 Thread : %d\n", c, i);
}

int main() {
    // 바로 실행
  	std::future<void> a = std::async(std::launch::async, for_print, 'a');
    // 위와 동일, std::launch::async 인자 생략 가능
    std::future<void> c = std::async(for_print, 'a');
    
    // 실행하지 않고 대기하다가 .get()이나 .wait()를 만나면 실행
  	std::future<void> b = std::async(std::launch::deferred, for_print, 'b');  	
  	b.get();
 	return 0;
}
```

##### std::future_error

```c++
...
int main() {
    auto c = std::async(for_print, 'c');
    std::future<int> d; // 정의 되지 않은 future
    
  	try {
    	c.get(); 
    	d.get(); // 정의되지 않은 future d를 실행
  	}
    // 정의되지 않는 future d에 대한 처리
  	catch (const std::future_error& e) {
    	std::cout << "Caught a future_error with code \"" << e.code()
      		<< "\"\nMessage: \"" << e.what() << "\"\n";
  	}
  	return 0;
}
```

##### std::future.wait_for()

```c++
#include <iostream>
#include <future>
#include <chrono>

const int number = 444444443;

// 최적화되지 않은 소수 검사 함수
bool is_prime(int x) {
    for (int i = 2; i < x; ++i)
        if (x % i == 0)
            return false;
    return true;
}

int main()
{
    // 비동기적으로 함수 호출
    std::future<bool> fut = std::async(is_prime, number);
    
    std::cout << "checking, please wait";
    std::chrono::milliseconds span(100);

    // 100밀리초씩 대기하며 실행
    while (fut.wait_for(span) != std::future_status::ready) {
        std::cout << '.';
        std::cout.flush();
    }

    bool x = fut.get();		// retrieve return value
    std::cout << "\n" << number << " " << (x ? "is" : "is not") << " prime.\n";
    return 0;
}
```

##### std::async를 이용한 parallelsum

- 1 ~ 1000 까지의 합을 Thread 5개를 생성해 병렬로 계산

```c++
#include <iostream>
#include <future>
#include <algorithm>

template <typename it>
int parallel_sum(it start, it end) {
    int len = end - start;
    if (len <= 1)
        return 0;

    it newStart = start + 200;
    auto handle = std::async(parallel_sum<it>, newStart, end);
    int sum = 0;
    for_each(start, newStart, [&](int n) {
        sum += n; /* std::cout << n << "\n"; */
    });
    return sum + handle.get();
}

int main()
{
    std::vector<int> v;
    v.reserve(1001);
    for (int i = 1; i < 1001; i++)
        v.emplace_back(i);

    std::cout << "sum : " << parallel_sum(v.begin(), v.end()) << "\n";
    
    return 0;
}
```

#### c++ coroutine

[Coroutine](https://luncliff.github.io/coroutine/ppt/[Kor]ExploringTheCppCoroutine.pdf)를 기록하며 학습

- Invocation: 루틴의 시작점으로 점프
- Activation: 루틴 안의 임의 지점으로 점프
- Finalization: 루틴의 끝에 도달하여 루틴 상태 소멸 및 정리
- Suspension: Finalization하지 않고 다른 루틴의 지점으로 점프

| Operation | Subroutine | Coroutine |     C++ Coroutine      |
| :-------: | :--------: | :-------: | :--------------------: |
|  Invoke   |     O      |     O     |  서브루틴 호출과 동일  |
| Finalize  |     O      |     O     |      `co_return`       |
|  Suspend  |     X      |     O     | `co_await`, `co_yield` |
|  Resume   |     X      |     O     |    `coro.resume()`     |

다음 중 하나가 함수 안에 있으면 코루틴으로 처리:

- `co_await <expression>`
- `co_yield <expression>`
- `co_return <expression>`
- `for co_await <expression>`

컴파일러 지원 여부:

| Compiler | 버전                         | 명령 인자                                   |                  참조                   |
| -------- | ---------------------------- | ------------------------------------------- | :-------------------------------------: |
| msvc     | Visual Studio 20.15 or later | `/await`                                    |                                         |
| clang    | 5.0 or later                 | `-fcoroutines-ts –stdlib=libc++ -std=c++2a` |                                         |
| gcc      | 10.0 or later                | `gcc -fcoroutines `                         | https://gcc.gnu.org/wiki/cxx-coroutines |

#### c++ stackless

코루틴의 함수 프레임을 어디에 둘 것인가? stackfull or stackless


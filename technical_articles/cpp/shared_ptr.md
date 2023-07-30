# shared_ptr

- 둘 이상의 소유자가 메모리에 있는 개체 수명 관리

`<memory>` 를 포함해야 한다:

```c++
#include <memory>
```

`shared_ptr` 만들기:

```c++
auto song1 = make_shared<Song>(...);
auto song2 = make_shared<Song>(...);
```



소유권 공유:

```c++
auto song3 = song1;
```

객체 멤버 참조:

```c++
wcout << song3->artist << endl;
```

`shared_ptr` 멤버 참조:

```c++
if (song3.get() != nullptr) {}
```

# weak_ptr

`shared_ptr`의 참조 카운트를 증가시키지 않는다.

`<memory>` 를 포함해야 한다:

```c++
#include <memory>
```

참조:

```c++
std::weak_ptr<int> wsong1(song1);
std::weak_ptr<int> wsong2(song2);
```

사용 전에 반드시 만료 되었는지 확인:
```
if (!wsong1.expired()) {
    ...
}
```

두 객체 교환:

```c++
wsong1.swap(wsong2);
```

소유 리소스 해제:

```c++
wsong1.reset();
```

## 참조

- [`shared_ptr` 인스턴스 만들기 및 사용](https://learn.microsoft.com/ko-kr/cpp/cpp/how-to-create-and-use-shared-ptr-instances?view=msvc-170)
- [`weak_ptr` 클래스]()



## 참조

- [`shared_ptr` 인스턴스 만들기 및 사용](https://learn.microsoft.com/ko-kr/cpp/cpp/how-to-create-and-use-shared-ptr-instances?view=msvc-170)
- [`weak_ptr` 클래스]()
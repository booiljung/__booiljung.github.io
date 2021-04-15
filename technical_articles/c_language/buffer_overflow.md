# C 배열의 버퍼오버플로우

## 버퍼오버플로우

```c
#include <stdio.h>

int main() {
  int a1 = 1;
  int a2 = 2;
  int b[4];
  int c1 = 3;
  int c2 = 4;

  printf("a1: %d\n", a1);
  printf("a2: %d\n", a2);
  printf("c1: %d\n", c1);
  printf("c2: %d\n", c2);

  b[4] = 20;

  printf("a1: %d\n", a1);
  printf("a2: %d\n", a2);
  printf("c1: %d\n", c1);
  printf("c2: %d\n", c2);
}
```

## 멀웨어 심기

```c++
#include <iostream>

using namespace std;

void malware() {
  cout << "malware()" << endl;
}

void function() {
  cout << "function()" << endl;
  unsigned long long a[4];
  a[5] = (unsigned long long)malware;
}

int main() {
  cout << "main() 1" << endl;
  function();
  cout << "main() 2" << endl;
}
```

실행

```
main() 1
function1()
function2()
exited, segmentation fault
```


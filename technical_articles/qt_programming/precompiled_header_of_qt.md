# Qt에서 프리컴파일헤더

C/C++, Objective-C 같은 C 계열 언어는 나쁜 점들이 있는데, 그 중의 하나는 헤더파일의 존재 입니다. 때로 헤더 파일은 컴파일과 링크를 다른 버전에 임포트하게 하여 알 수 없는 오류를 만들어 내기도 합니다. 많은 헤더 파일을 소스에 포함하여 컴파일 하는 것은 컴파일 속도를 느리게 하기도 합니다.

그래서 C/C++ 컴파일러는 미리 컴파일 하는 방법을 제공하기도 합니다. CMake도 프리컴파일헤더를 제공하긴 하지만, VSCode C/C++ Extension이 프리컴파일 헤더를 제공하지 않아 정적 분석을 하지 못하고 오류를 표시합니다.

이 글은 Qt의 qmake파일을 편집하여 프리컴파일 헤더를 사용하는 방법입니다.

```qmake
...
CONFIG += precompile_header 	# Precomiple 모듈을 사용하겠다고 지정해야 합니다.
PRECOMPILED_HEADER = stable.h	# 프리컴파일할 헤더파일들을 이 파일에 지정합니다.
precompile_header:!isEmpty(PRECOMPILED_HEADER) { # 프리컴파일 헤더들이 지정되면 
    DEFINES += USING_PCH		# 프리컴파일을 하겠다고 지정 합니다.
}
...
```

`stable.h` 파일은 다음과 같을 것입니다.

```c++
#ifndef STABLE_HPP
#define STABLE_HPP

// Add C includes here
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#if defined __cplusplus
// Add C++ includes here
#include <cstdlib>
#include <iostream>
#include <vector>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <mavsdk/mavsdk.h>
#endif

#endif // STABLE_HPP
```

## 참조

- Qt


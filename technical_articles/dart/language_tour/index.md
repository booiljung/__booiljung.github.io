[Up](../index.md)

이 **다트 언어로의 여행** 문서는 다트 [A tour of the Dart languag](https://dart.dev/guides/language/language-tour)를 번역으로 시작하였으며 문서의 권리는 [CC BY 3.0](http://creativecommons.org/licenses/by/3.0/)를 따릅니다. 이 문서는 프로그래밍 언어 안내서이지 프로그래밍 안내서가 아닙니다. 다른 프로그래밍 언어에 대한 지식을 가지고 있다면 유용하지만, 다른 프로그래밍 언어에 대한 지식이 없다면 이 문서로 프로그래밍을 배우기에 적합하지 않습니다.

# A tour of the Dart language

이 페이지는 다른 언어를 통해 프로그래밍하는 방법을 이미 알고 있다는 가정 하에 변수 및 연산자에서 클래스 및 라이브러리까지 Dart의 주요 기능을 사용하는 방법을 보여줍니다. 언어에 대한 간략하고 완전한 소개는 [언어 샘플 페이지](https://dart.dev/samples)를 참조하세요.

Dart의 핵심 라이브러리에 대해 자세히 알아보려면 [라이브러리 둘러보기](https://dart.dev/guides/libraries/library-tour)를 참조하세요 . 언어 기능에 대한 자세한 내용을 원할 때는 [Dart 언어 사양](https://dart.dev/guides/language/spec)을 참조하세요.

**참고:** DartPad를 사용하여 대부분의 Dart 언어 기능을 사용할 수 있습니다([자세히 알아보기](https://dart.dev/tools/dartpad)). **[다트패드 열기](https://dartpad.dev/).**

이 페이지는 포함된 DartPad를 사용하여 몇 가지 예제를 나타냅니다. DartPad 대신 빈 상자가 표시되면 [DartPad 문제 해결 페이지](https://dart.dev/tools/dartpad/troubleshoot) 로 이동 하세요.

- [Dart 프로그램의 기본 (A basic Dart program)](./a_basic_dart_program.md)
- [변수 (Variables)](./variables.md)
- [내장 타입 (Built in types)](./built_in_types.md)
- [함수 (Functions)](./functions.md)
- [연산자 (Operators)](./operators.md)
- [흐름 제어문 (Control flow statements)](./control_flow_statements.md)
- [예외 (Exceptions)](./exceptions.md)
- [클래스 (Classes)](./classes.md)
- [제네릭 (Generics)](./generics.md)
- [라이브러리와 가시성 (Libraries and visibility)](./libraries_and_visibility.md)
- [비동기 지원 (Asynchorony support)](./asynchrony_support.md)
- [제너레이터 (Generators)](./generators.md)
- [Callable classes](./callable_classes.md)
- [격리 (Isolates)](./Isolates.md)
- [타입 정의 (Typedefs)](./typedefs.md)
- [메타데이터 (Metadata)](./metadata.md)
- [주석 (Comments)](./comments.md)

이 페이지는 다트 언어에서 일반적으로 사용되는 기능을 요약 한 것입니다. 더 많은 기능이 구현되고 있지만 기존 코드를 손상시키지 않을 것으로 기대합니다. 자세한 내용은 [다트 언어 사양](https://dart.dev/guides/language/spec) 및 [이펙티브 다트](https://dart.dev/guides/language/effective-dart)를 참조하십시오. 다트의 핵심 라이브러리에 대한 자세한 내용은 [다트 라이브러리 둘러보기](https://dart.dev/guides/libraries/library-tour)를 참조하십시오.

## 문서 변경 이력

2019년 5월 15일: 첫 작성.

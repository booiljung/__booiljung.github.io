[Up](../index.md)

# Flutter

- [2022년 플러터 단상](a_shoft_thought_of_flutter_in_2022.md)

## Tips

- Ubuntu 18.04 LTS에서 Android studio 보다 Visual Studio Code가 경쾌하고 빠릅니다.
- Ubuntu 18.04 LTS에서 메모리 16기가가 원할하게 작동합니다.

## Contents

- [Installation of Flutter on Ubuntu](installation_of_flutter_on_ubuntu.md)
-  [Ubuntu에서 재빠르게 VSCode와 Flutter 개발 환경 구성하기](ubuntu_flutter_vscode_quick.md)
- [Flutter+Youtube: Flutter Widget of the Week Playlist](https://www.youtube.com/playlist?list=PLOU2XLYxmsIL0pH0zWe_ZOHgGhZ7UasUE)
- [Flutter: 상태 관리하기](state_management/index.md)
- [Flutter: 머터리얼 테마](material_theme.md)
- [Flutter: Unit Test](./unit_test/index.md)
- [Flutter: Using BoxIcons in Flutter](using_boxicons_in_flutter.md)
- [Flutter: Flutter 1.9에서 새로운 점](what_is_new_in_flutter_1_9.md)

### Flutter for Mobile

- [Flutter: Adding Google Maps to Flutter](https://medium.com/flutter-io/google-maps-and-flutter-cfb330f9a245)

### Flutter for Web

- [번역: 허밍버드: Flutter for the Web를 개발하기 까지](hummingboard_building_flutter_for_the_web.md)
- [번역: 웹 개발을 위한 webdev](https://dart.dev/tools/webdev#using-webdev-and-build_runner-commands)
- [github: Flutter 웹데모: flutter/samples/web/README.md](https://github.com/flutter/samples/tree/master/web)
- [번역: 웹을 위한 Flutter: 웹사이트를 밑바닥부터 만들고 배포하기](flutter_for_web_create_and_deploy_a_website_from_scratch.md)
- [Ins and Outs of Flutter Web](https://medium.com/flutter-community/ins-and-outs-of-flutter-web-7a82721dc19a)

## Library for Fultter

- [Flutter Redux package](https://pub.dev/packages/flutter_redux): Flutter 위젯을 만들기 위해 Redux 저장소를 쉽게 사용할 수 있게 해주는 유틸리티 세트.

## 상태관리

사용해본 플러터 상태 관리는 아래와 같다.

- `setState()`
- Bloc
- Provider
- GetX
- get_it

각기 컨셉이 다르며 장단점을 짧게 정리해본다.

**`setState()`**

flutter에서 기본 제공 한다. 상태관리하고자 하는 변수가 클래스 범위를 벗어나면 복잡해진다.

**Bloc**:

많은 기능을 가지고 있다. 그만큼 난해하고 복잡하다. 워드프로세서를 만들때 적합하다.

**Provider**:

심플하고 단순하다. `BuildContext` 벗어난 곳에서 다루기가 복잡해진다. 예를 들면 타이머 핸들러에서 처리하고자 하는 경우다.

**GetX**:

Provider와 달리 `BuildContext` 밖에서 사용할 수 있다. Dart Rx 기반으로 프로퍼티도 Dart Rx 타입을 사용해야 하는 문제가 있다.

**get_it**:

Provider와 GetX의 장점을 취하였다고 볼 수 있다. 구조가 상당히 다르므로 실수하기 쉽다.

**상태관리 관련 문서:**

- [번역: Bloc](./bloc/index.md): Dart 및 Flutter용 BLoC 아키텍쳐 패턴
- [Provider](./provider/index.md)
- [get_it](get_it/index.md)

## Quick Examples

- [Quick Examples](./quick_examples.md)

## Demo

- [Widget Demo](widget_demo.md)


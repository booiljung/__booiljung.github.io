[Up](../index.md)

# get_it

2022년 2월 21일

`pubspec.yaml`에 패키지를 등록하여 설치한다.

```yaml
dependencies:
  flutter:
    sdk: flutter
  get_it: any
  cupertino_icons: ^1.0.2
```

`get_it.dart` 파일이다.

```dart
import 'package:get_it/get_it.dart';

GetIt getIt = GetIt.instance;
```

공식 예제는 `main.dart`에 `getIt` 전역 변수를 만들어 두었는데 전역변수 없이 `GetIt.instance`나 `GetIt.I`를 사용해도  된다. 

공식 예제에서 사용된 모델이다.

```dart
import 'package:flutter/material.dart';

import 'get_it.dart';

abstract class AppModel extends ChangeNotifier {
  void incrementCounter();

  int get counter;
}

class AppModelImplementation extends AppModel {
  int _counter = 0;

  AppModelImplementation() {
    /// lets pretend we have to do some async initilization
    Future.delayed(Duration(seconds: 3)).then((_) => getIt.signalReady(this));
  }

  @override
  int get counter => _counter;

  @override
  void incrementCounter() {
    _counter++;
    notifyListeners();
  }
}
```

이 예제 코드는 꼼꼼하게 보지 않으면 실수할 가능성이 있다.

생성자에서

```
Future.delayed(Duration(seconds: 3)).then((_) => getIt.signalReady(this));
```

라는 코드가 있는데 생성자에서 할 필요가 없는 코드다. 오해를 불러오는 전형적인 나쁜 예제. 딜레이된 초기화는 별도의 예지를 통해 설명해야 했다.

단순하게 하자면:

```dart
import 'package:flutter/material.dart';

import 'get_it.dart';

class AppModel extends ChangeNotifier {
  void incrementCounter();  
  void incrementCounter() {
    counter++;
    notifyListeners();
  }
}
```

생성자를 제거하였는데 생성자에서 했던:

```
getIt.signalReady(this)
```

는 싱글톤 등록 후 반드시 준비해 주어야 한다:

```dart
void main() {
  var appModel = AppModel();
  getIt.registerSingleton<AppModel>(appModel, signalsReady: true);
  getIt.signalReady(appModel);
  runApp(MyApp());
}
```

등록과 준비를 동시해 하는 함수를 확장할 수 있다:

```dart
extension GetItExtension on GetIt {
  void registerSingletonAndReady<T extends Object>(
    T instance, {
    String? instanceName,
    bool? signalsReady,
    DisposingFunc<T>? dispose,
  }) {
    this.registerSingleton<T>(
      instance,
      instanceName: instanceName,
      signalsReady: signalsReady,
      dispose: dispose,
    );
    this.signalReady(instance);
  }
}
```

시그널을 받는 위젯의 `initState()`와 `dispose()`에서 `setState()`를 처리해 주어야 한다.

```dart
@override
  void initState() {
    super.initState();
    getIt
        .isReady<AppModel>()
        .then((_) => getIt<AppModel>().addListener(update));
    // Alternative
    // getIt.getAsync<AppModel>().addListener(update);
  }

  @override
  void dispose() {
    getIt<AppModel>().removeListener(update);
    super.dispose();
  }

  void update() => setState(() => {});
```

상태 반영은 아래와 같다:

```dart
  @override
  Widget build(BuildContext context) {
    return Material(
      child: FutureBuilder(
          future: getIt.allReady(),
          builder: (context, snapshot) {
            if (snapshot.hasData) {
              return Scaffold(
                appBar: AppBar(
                  title: Text(widget.title),
                ),
                body: Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      Text(
                        'You have pushed the button this many times:',
                      ),
                      Text(
                        getIt<AppModel>().counter.toString(),
                        style: Theme.of(context).textTheme.headline4,
                      ),
                    ],
                  ),
                ),
                floatingActionButton: FloatingActionButton(
                  onPressed: getIt<AppModel>().incrementCounter,
                  tooltip: 'Increment',
                  child: Icon(Icons.add),
                ),
              );
            } else {
              return Column(
                mainAxisAlignment: MainAxisAlignment.center,
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text('Waiting for initialisation'),
                  SizedBox(
                    height: 16,
                  ),
                  CircularProgressIndicator(),
                ],
              );
            }
          }),
    );
  }
```








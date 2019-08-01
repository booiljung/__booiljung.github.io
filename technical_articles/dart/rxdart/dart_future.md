# Future

### 생성자

#### Future

`Timer.run`과 비동기적으로 `computation`을 호출 한 결과를 포함하는 `Future`를 만듭니다.

`computation` 실행 결과가 throw되면 반환 된 Future가 error와 함께 완료됩니다.

반환 값 자체가 `Future` 인 경우 작성한 future의 completion는 반환 된 future가 완료d 될 때까지 대기 한 다음 동일한 결과로 `완료`됩니다.

non-Future 값이 리턴되면 리턴 된 Future는 해당 값으로 완료d됩니다.

#### Future.delayed

delay 후에 계산을 실행하는 Future를 만듭니다.

주어진 `duration`이 지난 후에 `computation`이 실행되고 Future가 계산 결과와 함께 완료됩니다.

`computation`이 future를 리턴하면, 이 생성자가 리턴한 future는 그 future의 값 또는 error로 완료됩니다.

지속 시간이 0 이하이면 모든 마이크로태스크가 실행 된 후 다음 이벤트 루프 반복보다 더 빨리 완료됩니다.

`computation`을 생략하면 `computation`은 `() => null` 인 것처럼 처리되고 future는 결국 `null` 값으로 완료됩니다.

`computation` 호출이 throw되면 생성된 future가 오류와 함께 완료됩니다.

추후에 future를 창조하고 완료하는 방법에 관해서는 `Completer`를 참조하십시오.

#### Future.error

오류로 완료되는 future를 만듭니다.

생성 된 future는 future의 마이크로태스크에서 오류와 함께 완료 될 것입니다. 이것은 누군가가 future에 오류 처리기를 추가 할 수있는 충분한 시간을 허용합니다. future가 완료되기 전에 오류 핸들러가 추가되지 않으면, 오류는 처리되지 않은 것으로 간주됩니다.

`error`가 `null`의 경우는, `NullThrownError`에 옮겨 놓을 수 있습니다.

`Completer`를 사용하여 future를 만들고 나중에 완료하십시오.

#### Future.microtask

`scheduleMicrotask`와 비동기적으로 `computatoion`을 호출 한 결과가 포함 된 future를 작성합니다.

`computation`을 실행하면 throw 된 오류로 반환된 future가 완료됩니다.

`computation`을 호출하여 Future가 반환되면 작성된 future의 completion는 반환된 future가 완료d 될 때까지 기다린 다음 동일한 결과로 완료됩니다.

`computation`을 호출하여 non-future 값이 반환되면 반환 값은 해당 값으로 완료됩니다.

#### Future.sync

`computatuon`을 즉시 호출 한 결과를 포함하는 future를 리턴합니다.

`computation`을 호출하면 오류가 발생하면, 반환된 future가 완료됩니다.

`computation`을 호출하여 `Future<T>`가 반환되면 해당 future가 반환됩니다.

`computation` 호출이 non-future 값을 리턴하면, 해당 값으로 완료된 future가 리턴됩니다.

#### Future.value

`value`가 있는 future를 만듭니다.

`value`가 future 라면, 생성 된 future는 future의 `value`가 완료d될 때까지 기다린 다음 동일한 결과로 완료됩니다. `vlaue` future는 오류로 끝날 수 있기 때문에 `Future.value`가 future에 만들 수있는 이름은 그 이름이 다른 경우에도 마찬가지입니다.

`value`가 `Future`가 아닌 경우, 생성 된 future는 `value` 값으로 완료됩니다. 이는 새로운 `Future<T> .sync(() => value)`와 동일합니다.

`Completer`를 사용하여 `future`를 만들고 나중에 `완료`하십시오.

### 인스턴스 메서드

#### asStream()

이 future의 결과를 포함한 `Stream`을 작성합니다.

stream은 단일 데이터 또는 이 future의 완료 결과가 포함 된 오류 이벤트를 생성 한 다음 done 이벤트로 닫힙니다.

future가 완료d 되지 않으면 stream은 이벤트를 생성하지 않습니다.

#### catchError()

이 `Future`가 발행한 에러를 처리합니다.

이것은 "catch"블록과 비동기식입니다.

이 `Future`의 결과 또는 `onError` 콜백의 호출 결과로 완료되는 새로운 `Future`를 리턴합니다.

이 future가 값으로 완료되면, 리턴 된 future는 동일한 값으로 완료됩니다.

이 future가 오류로 완료되면 오류 값으로 `test`가 먼저 호출됩니다.

`test`가 `false`를 반환하면 예외는 이 `catchError`에 의해 처리되지 않고 반환 된 future는이 오류와 스택 추적으로 완료됩니다.

`test`가 `true`를 반환하면 `onError`가 오류 및 스택 추적과 함께 호출되고 반환 된 future가 다음 `onError`와 완전히 동일한 방식으로 이 호출의 결과로 완료됩니다.

`test`가 생략되면, 항상 `true`를 반환하는 함수가 기본값으로 사용됩니다. 테스트 함수는 throw를 해서는 안되지만, 한다면 `onError` 함수가 던져진 것처럼 처리됩니다.

future는 listener가 추가 될 때까지 오류보고를 지연시키지 않습니다. 이 future가 오류로 완료된 후 첫 번째 `catchError` (또는 `then`) 호출이 발생하면 오류는 처리되지 않은 오류로 보고됩니다. `Future`에 대한 설명을 참조하십시오.

#### then()

이 future가 complte 될 때 호출 할 콜백을 등록합니다.

이 future가 값으로 완료되면 그 값으로 `onValue` 콜백이 호출됩니다. 이 미래가 이미 완료된 경우 콜백은 즉시 호출되지 않고 나중의 마이크로태스크에서 예약됩니다.

`onError`가 제공되고 이 future가 오류로 완료되면 해당 오류 및 스택 추적과 함께 `onError` 콜백이 호출됩니다. `onError` 콜백은 하나의 인수 또는 두 개의 인수를 받아 들여야 합니다. 여기서 두 번째 인수는 `StackTrace`입니다. `onError`가 두 개의 인수를 받아들이면 오류와 스택추적 모두로 호출되며 그렇지 않으면 오류 객체만으로 호출됩니다. `onError` 콜백은 반환 된 미래를 완료하는 데 사용할 수 있는 값 또는 future를 반환해야하므로 `FutureOr<R>`에 할당 할 수있는 값이어야 합니다.

`onValue` (이 future가 값으로 완료 될 경우) 또는 `onError` (이 future가 오류로 완료될 경우)의 호출 결과로 완료되는 새로운 Future를 리턴합니다.

호출 된 콜백이 throw되면 반환 된 미래는 throw 된 오류 및 오류에 대한 스택 추적으로 완료됩니다. `onError`의 경우 throw되는 예외가 `onError`에 대한 오류 인수와 동일하면 throw가 다시 발생 된 것으로 간주되고 원본 스택추적이 대신 사용됩니다.

콜백이 Future를 반환하면 future에 반환 된 미래는 콜백에 의해 반환 된 future와 동일한 결과로 완료됩니다.

`onError`가 주어지지 않고이 future가 오류로 완료되면 오류는 반환 된 future로 직접 전달됩니다.

대부분의 경우, 단일 호출에서 값과 오류를 모두 처리하는 대신 테스트 매개 변수를 사용하여 `catchError`를 별도로 사용할 수 있습니다.

future는 listener가 추가 될 때까지 오류 보고를 지연시키지 않습니다. 이 미래가 오류로 완료된 후 첫 번째 `then` 또는 `catchError` 호출이 발생하면 오류는 처리되지 않은 오류로 보고됩니다. `Future`에 대한 설명을 참조하십시오.

#### timeout()

`timeLimit`이 경과 한 후 future의 computation을 Time-out합니다.

이 future가 끝나면, 이 future와 같은 값으로 완료하는 새로운 future를 돌려줍니다.

`timeLimit`이 지나기 전에 이 future가 완료되지 않으면 대신 `onTimeout` 액션이 실행되고 반환 된 값 또는 반환 된 값의 결과가 반환 된 future의 결과로 사용됩니다. `onTimeout` 함수는 `T` 또는 `Future<T>`를 반환해야합니다.

`onTimeout`을 생략하면 시간 초과로 인해 반환 된 future가 `TimeoutException`으로 완료됩니다.

#### whenComplete

이 future가 완료 될 때 호출 할 함수를 등록합니다.

이 future가 완료되면 `action` 함수가 호출되며, 값 또는 오류가 있는지 여부에 관계없이 호출됩니다.

이것은 "finally"블록과 비동기식입니다.

이 호출 `f`가 반환하는 future는 `action` 호출이나 future의 `action` 호출에서 에러가 발생하지 않는 한이 future와 같은 방식으로 완료됩니다. `action`이 future를 반환하지 않으면 반환 값은 무시됩니다.

`action` 호출이 던지면 던져진 오류로 `f`가 완료됩니다.

`action` 호출이 `Future`, `f2`를 반환하면 `f2`의 완료는 `f2`가 완료 될 때까지 지연됩니다. `f2`가 오류로 완료되면 `f`가 결과로 나타납니다. `f2`의 값은 항상 무시됩니다.

### 클래스 메서드

#### any()

`Future`의 첫 번째 future의 결과를 반환합니다.

반환 된 future는 값이든 오류든 간에 그것이 완료되었다는 보고를 위해 future의 첫 번째 미래의 결과로 완료됩니다. 다른 모든 future의 결과는 폐기됩니다.

`future`이 비어 있거나 future가 없으면 반환 된 future는 결코 완료되지 않습니다.

#### doWhile()

`false`를 반환 할 때까지 반복적으로 작업을 수행합니다.

조작, `action`는 동기 또는 비동기 중 하나 일 수 있습니다.

`bool` 값 `true` 또는 값 `true`로 완료되는 `Future<bool>`을 반환하는 한 반복적으로 작업이 호출됩니다.

`action`에 대한 호출이 `false` 또는 `Future`가 `false`로 완료되면 반복이 종료되고 `doWhile`에 의해 반환 된 future가 `null` 값으로 완료됩니다.

`action` 호출이 발생하거나 `action`로 리턴 된 future가 오류와 함께 완료되면 반복이 종료되고 `doWhile`에 의해 리턴 된 future가 동일한 오류로 완료됩니다.

`doWhile`을 호출 한 직후를 포함하여 `action`은 언제든지 발생할 수 있습니다. 유일한 제한은 이전 호출이 반환되기 전에 새로운 호출을 요구하고 future가 완료 될 때까지 `Future<bool>`을 반환하는 경우입니다.

#### forEach

차례대로 iterable의 각 요소에 대해 작업을 수행합니다.

`action`은 동기식 또는 비동기식 중 하나 일 수 있습니다.

요소의 각 요소를 순서대로 호출합니다. `action`에 대한 호출이 `Future<T>`를 리턴하면, 반복은 다음 요소가 계속되기 전에 future가 완료 될 때까지 대기합니다.

모든 요소가 처리 될 때 `null`로 완료되는 `Future`를 리턴합니다.

future가 아닌 반환 값과 반환 된 future의 완료 값은 무시됩니다.

동기 또는 비동기 `action`의 오류는 반복을 중지하고 리턴 된 future에 보고됩니다.

#### wait

여러 future가 완료 될 때까지 기다렸다가 결과를 수집합니다.

제공되는 모든 future이 결과와 함께 완료되거나 제공된 future 중 하나라도 실패하면 오류가 발생하여 완료 할 future를 반환합니다.

반환되는 future의 값은 future를 반복하여 future이 제공되는 순서로 생성 된 모든 값의 목록입니다.

어떤 future가 오류로 완료되면 반환 된 future는 그 오류로 완료됩니다. 추가 future에도 오류가 있으면 오류가 삭제됩니다.

`eagerError`가 `true` 인 경우 반환 된 future는 future 중 하나의 첫 번째 오류에서 즉시 오류로 완료됩니다. 그렇지 않으면 반환 future가 완료되기 전에 모든 future이 완료되어야합니다 (여전히 첫 번째 오류가 있고 나머지 오류는 자동으로 삭제됩니다).

오류가 발생한 경우 `cleanup` (제공된 경우)은 성공적인 future의 `null`이 아닌 결과에 대해 호출됩니다. 따라서 반환 된 future가 이러한 값에 액세스 할 수 없으므로 손실 될 수있는 리소스를 정리할 수 있습니다. 오류가 없는 경우 `cleanUp` 함수는 사용되지 않습니다.

`cleanUp`은 throw를 하지 말아야 합니다. 오류가 throw되면 캐치되지 않은 비동기 오류가 발생합니다.


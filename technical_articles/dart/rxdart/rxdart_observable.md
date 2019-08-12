# Observable<T> class

A wrapper class that extends Stream. It combines all the Streams and StreamTransformers contained in this library into a fluent api.

Stream을 확장하는 래퍼 클래스입니다. 이 라이브러리에 포함 된 모든 Streams 및 StreamTransformer를 유창한 API로 결합합니다.

### Example

```dart
new Observable(new Stream.fromIterable([1]))
  .interval(new Duration(seconds: 1))
  .flatMap((i) => new Observable.just(2))
  .take(1)
  .listen(print); // prints 2
```

### Learning RxDart

This library contains documentation and examples for each method. In addition, more complex examples can be found in the [RxDart github repo](https://github.com/ReactiveX/rxdart) demonstrating how to use RxDart with web, command line, and Flutter applications.

이 라이브러리에는 각 메소드에 대한 문서와 예제가 들어 있습니다. 또한 웹, 명령 줄 및 Flutter 응용 프로그램과 함께 RxDart를 사용하는 방법을 보여주는 [RxDart github repo](https://github.com/ReactiveX/rxdart)에서보다 복잡한 예제를 찾을 수 있습니다.

#### Additional Resources

In addition to the RxDart documentation and examples, you can find many more articles on Dart Streams that teach the fundamentals upon which RxDart is built.

RxDart 설명서와 예제 외에도 DxStreams에 대한 RxDart의 기본 사항을 가르치는 기사가 더 많이 있습니다.

- [Asynchronous Programming: Streams](https://www.dartlang.org/tutorials/language/streams)
- [Single-Subscription vs. Broadcast Streams](https://www.dartlang.org/articles/libraries/broadcast-streams)
- [Creating Streams in Dart](https://www.dartlang.org/articles/libraries/creating-streams)
- [Testing Streams: Stream Matchers](https://pub.dartlang.org/packages/test#stream-matchers)

### Dart Streams vs Observables

In order to integrate fluently with the Dart ecosystem, the Observable class extends the Dart `Stream` class. This provides several advantages:

Observable 클래스는 다트 생태계와 유창하게 통합되기 위해 다트 `Stream` 클래스를 확장합니다. 이것은 몇 가지 장점을 제공합니다:

- Observables work with any API that expects a Dart Stream as an input.
- Observables는 Dart Stream을 입력으로 기대하는 모든 API에서 작동합니다.
- Inherit the many methods and properties from the core Stream API.
- 핵심 Stream API에서 많은 메소드와 속성을 상속받습니다.
- Ability to create Streams with language-level syntax.
- 언어 수준 구문으로 스트림을 생성 할 수 있습니다.

Overall, we attempt to follow the Observable spec as closely as we can, but prioritize fitting in with the Dart ecosystem when a trade-off must be made. Therefore, there are some important differences to note between Dart's `Stream` class and standard Rx `Observable`.

전반적으로 우리는 Observable 스펙을 가능한 한 가깝게 따르려고 하지만, 절충안을 작성해야하는 경우 Dart 생태계에 우선 순위를 부여합니다. 그러므로 Dart의 `Stream` 클래스와 표준 Rx ` Observable` 클래스 사이에는 몇 가지 중요한 차이점이 있습니다.

First, Cold Observables in Dart are single-subscription. In other words, you can only listen to Observables once, unless it is a hot (aka broadcast) Stream. If you attempt to listen to a cold stream twice, a StateError will be thrown. If you need to listen to a stream multiple times, you can simply create a factory function that returns a new instance of the stream.

첫째, Dart의 Cold Observables는 단일 구독입니다. 다시 말해서 Observables가 뜨거운 (일명 boradcast) 스트림이 아닌 한 Observables를 한 번만 들을 수 있습니다. 콜드 스트림을 두 번 청취하려고 하면 StateError가 발생합니다. 스트림을 여러 번 청취해야하는 경우 스트림의 새 인스턴스를 반환하는 팩토리 함수를 만들면됩니다.

Second, many methods contained within, such as `first` and `last` do not return a `Single` nor an `Observable`, but rather must return a Dart Future. Luckily, Dart Futures are easy to work with, and easily convert back to a Stream using the `myFuture.asStream()` method if needed.

둘째, `first`와 `last`와 같은 많은 메서드는 `Single`이나 `Observable`을 반환하지 않고 다트 퓨쳐를 반환해야합니다. 운좋게도 Dart Futures는 작업하기 쉽고 필요할 경우 `myFuture.asStream()`메소드를 사용하여 쉽게 스트림으로 다시 변환 할 수 있습니다.

Third, Streams in Dart do not close by default when an error occurs. In Rx, an Error causes the Observable to terminate unless it is intercepted by an operator. Dart has mechanisms for creating streams that close when an error occurs, but the majority of Streams do not exhibit this behavior.

셋째, 오류가 발생하면 Dart의 스트림이 기본적으로 닫히지 않습니다. Rx에서 오류는 Observable이 운영자에 의해 인터셉트 되지 않는 한 종료 되도록 합니다. 다트는 오류가 발생할 때 닫히는 스트림을 만드는 메커니즘을 가지고 있지만 대부분의 스트림은 이 동작을 나타내지 않습니다.

Fourth, Dart streams are asynchronous by default, whereas Observables are synchronous by default, unless you schedule work on a different Scheduler. You can create synchronous Streams with Dart, but please be aware the the default is simply different.

넷째, Dart 스트림은 기본적으로 비동기이지만 Observables는 다른 Scheduler에서 작업을 예약하지 않는 한 기본적으로 동기식입니다. Dart로 동기식 스트림을 만들 수 있지만 기본값이 다르다는 점에 유의하십시오.

Finally, when using Dart Broadcast Streams (similar to Hot Observables), please know that `onListen` will only be called the first time the broadcast stream is listened to.

마지막으로, Dart Broadcast Streams (Hot Observables와 유사)를 사용할 때 `onListen`은 방송 스트림을 처음 들었을 때만 호출된다는 것을 알아 두십시오.

### Constructors

#### Observable<T>.concat 

Concatenates all of the specified stream sequences, as long as the previous stream sequence terminated successfully.

이전의 스트림 순서가 정상적으로 종료하고 있는 한, 지정된 모든 스트림 순서를 연결합니다.

It does this by subscribing to each stream one by one, emitting all items and completing before subscribing to the next stream.

각 스트림을 하나씩 구독하고 모든 항목을 내보내고 다음 스트림에 가입하기 전에 완료하면 됩니다.

[Interactive marble diagram](http://rxmarbles.com/#concat)

Example

```dart
new Observable.concat([
  new Observable.just(1),
  new Observable.timer(2, new Duration(days: 1)),
  new Observable.just(3)
])
.listen(print); // prints 1, 2, 3
```

#### Observable<T>.concatEager

Concatenates all of the specified stream sequences, as long as the previous stream sequence terminated successfully.

이전의 스트림 순서가 정상적으로 종료하고 있는 한, 지정된 모든 스트림 순서를 연결합니다.

In the case of concatEager, rather than subscribing to one stream after the next, all streams are immediately subscribed to. The events are then captured and emitted at the correct time, after the previous stream has finished emitting items.

concatEager의 경우 다음 스트림 이후에 하나의 스트림을 구독하지 않고 모든 스트림을 즉시 구독합니다. 그런 다음 이벤트는 이전 스트림에서 항목 방출이 완료된 후 올바른 시간에 캡처되고 방출됩니다.

[Interactive marble diagram](http://rxmarbles.com/#concat)

Example

```dart
new Observable.concatEager([
  new Observable.just(1),
  new Observable.timer(2, new Duration(days: 1)),
  new Observable.just(3)
])
.listen(print); // prints 1, 2, 3
```

#### Observable<T>.defer 

The defer factory waits until an observer subscribes to it, and then it creates an Observable with the given factory function.

지연 공장은 관찰자가 그것에 가입 할 때까지 기다린 다음 주어진 공장 기능으로 Observable을 생성합니다.

In some circumstances, waiting until the last minute (that is, until subscription time) to generate the Observable can ensure that this Observable contains the freshest data.

경우에 따라 Observable을 생성하기 위해 마지막 순간 (즉 가입 시간까지)을 기다리면 이 Observable에 가장 최신 데이터가 포함될 수 있습니다.

By default, DeferStreams are single-subscription. However, it's possible to make them reusable.

기본적으로 DeferStream은 단일 구독입니다. 그러나 재사용이 가능합니다.

#### Observable<T>.empty 

Creates an Observable that contains no values.

값이 없는 Observable을 작성합니다.

No items are emitted from the stream, and done is called upon listening.

스트림에서 방출되는 항목이 없으며 청취시 완료됩니다.

Example

```dart
new Observable.empty().listen(
  (_) => print("data"), onDone: () => print("done")); // prints "done"
```

#### Observable<T>.error 

Returns an observable sequence that emits an `error`, then immediately completes.

`error`를 내보내는 관측 가능한 시퀀스를 반환하고 즉시 완료합니다.

The error operator is one with very specific and limited behavior. It is mostly useful for testing purposes.

오류 연산자는 아주 구체적이고 제한된 동작을하는 연산자입니다. 주로 테스트 목적으로 유용합니다.

Example

```dart
new Observable.error(new ArgumentError());
```

#### Observable<T>.eventTransformed 

Creates an Observable where all events of an existing stream are piped  through a sink-transformation.

기존 스트림의 모든 이벤트가 싱크 변환을 통해 파이프되는 Observable을 만듭니다.

The given `mapSink` closure is invoked when the returned stream is  listened to. All events from the `source` are added into the event sink  that is returned from the invocation. The transformation puts all  transformed events into the sink the `mapSink` closure received during  its invocation. Conceptually the `mapSink` creates a transformation pipe  with the input sink being the returned `EventSink` and the output sink  being the sink it received.

주어진`mapSink` 클로저는 리턴 된 스트림이 청취 될 때 호출됩니다. `source`의 모든 이벤트는 호출에서 리턴 된 이벤트 싱크에 추가됩니다. 변환은`mapSink` 클로저가 호출되는 동안 수신 된 모든 변환 이벤트를 싱크에 넣습니다. 개념적으로`mapSink`는 리턴 된`EventSink` 인 입력 싱크와 변환 파이프를 생성하고 출력 싱크는받은 싱크입니다.

#### Observable<T>.fromFuture

Creates an Observable from the future.

미래의 Observable을 만듭니다.

When the future completes, the stream will fire one event, either data or error, and then close with a done-event.

미래가 완료되면 스트림은 데이터 또는 오류 중 하나의 이벤트를 발생시킨 다음 완료 이벤트로 종료합니다.

### Example

```dart
new Observable.fromFuture(new Future.value("Hello"))
  .listen(print); // prints "Hello"
```

#### Observable<T>.fromIterable 

Creates an Observable that gets its data from `data`.

`data`로부터 데이터를 얻는 Observable을 생성합니다.

The iterable is iterated when the stream receives a listener, and stops iterating if the listener cancels the subscription.

iterable은 스트림이 리스너를 수신하면 반복되고 리스너가 구독을 취소하면 반복을 중지합니다.

If iterating `data` throws an error, the stream ends immediately with that error. No done event will be sent (iteration is not complete), but no further data events will be generated either, since iteration cannot continue.

`data`를 반복하면 에러가 발생하고, 에러는 즉시 종료됩니다. 완료 이벤트는 보내지지 않지만 (반복은 완료되지 않음) 반복 작업을 계속할 수 없기 때문에 더 이상의 데이터 이벤트도 생성되지 않습니다.

Example

```dart
new Observable.fromIterable([1, 2]).listen(print); // prints 1, 2
```

#### Observable<T>.just 

Creates an Observable that contains a single value

단일 값을 포함하는 Observable을 만듭니다.

The value is emitted when the stream receives a listener.

스트림이 청취자를 받으면 값이 방출됩니다.

Example

```dart
 new Observable.just(1).listen(print); // prints 1
```

#### Observable<T>.merge 

Flattens the items emitted by the given `streams` into a single Observable sequence.

주어진`streams`에 의해 방출 된 항목을 하나의 Observable 순서로 평평하게 만듭니다.

[Interactive marble diagram](http://rxmarbles.com/#merge)

Example

```dart
new Observable.merge([
  new Observable.timer(1, new Duration(days: 10)),
  new Observable.just(2)
])
.listen(print); // prints 2, 1
```

#### Observable<T>.never 

Returns a non-terminating observable sequence, which can be used to denote an infinite duration.

끝나지 않는 관찰 가능 시퀀스를 반환합니다.이 시퀀스는 무한 기간을 나타내는 데 사용할 수 있습니다.

The never operator is one with very specific and limited behavior. These are useful for testing purposes, and sometimes also for combining with other Observables or as parameters to operators that expect other Observables as parameters.

절대 운영자는 아주 구체적이고 제한된 행동을하는 운영자입니다. 이들은 테스트 목적으로 유용하며 때로는 다른 Observable과 결합하거나 다른 Observable을 매개 변수로 기대하는 연산자에 대한 매개 변수로 유용합니다.

Example

```dart
new Observable.never().listen(print); // Neither prints nor terminates
```

#### Observable<T>.periodic 

Creates an Observable that repeatedly emits events at `period` intervals.

`period` 간격으로 반복적으로 이벤트를 내보내는 Observable을 생성합니다.

The event values are computed by invoking `computation`. The argument to this callback is an integer that starts with 0 and is incremented for every event.

이벤트 값은 '계산'을 호출하여 계산됩니다. 이 콜백에 대한 인수는 0으로 시작하고 모든 이벤트에 대해 증가하는 정수입니다.

If `computation` is omitted the event values will all be `null`.

`computation`가 생략되면 이벤트 값은 모두 'null'이됩니다.

Example

```dart
 new Observable.periodic(new Duration(seconds: 1), (i) => i).take(3)
   .listen(print); // prints 0, 1, 2
```

#### Observable<T>.race 

Given two or more source `streams`, emit all of the items from only the first of these `streams` to emit an item or notification.

두 개 이상의 소스 스트림이 주어지면이 스트림 중 첫 번째 스트림에서만 모든 항목을 내보내 항목이나 알림을 내 보냅니다.

[Interactive marble diagram](http://rxmarbles.com/#amb)

Example

```dart
new Observable.race([
  new Observable.timer(1, new Duration(days: 1)),
  new Observable.timer(2, new Duration(days: 2)),
  new Observable.timer(3, new Duration(seconds: 1))
]).listen(print); // prints 3
```

#### Observable<T>.repeat 

Creates a `Stream` that will recreate and re-listen to the source Stream the specified number of times until the `Stream` terminates successfully.

`Stream`이 성공적으로 끝날 때까지 지정된 횟수만큼 소스 스트림을 재생성하고 다시 듣게 될`Stream`을 생성합니다.

If `count` is not specified, it repeats indefinitely.

`count`가 지정되지 않으면 무기한 반복됩니다.

Example

```dart
new RepeatStream((int repeatCount) =>
  Observable.just('repeat index: $repeatCount'), 3)
    .listen((i) => print(i)); // Prints 'repeat index: 0, repeat index: 1, repeat index: 2'
```

#### Observable<T>.retry

Creates an Observable that will recreate and re-listen to the source Stream the specified number of times until the Stream terminates successfully.

스트림이 성공적으로 끝날 때까지 지정된 횟수만큼 원본 스트림을 다시 만들고 다시 듣기위한 Observable을 만듭니다.

If the retry count is not specified, it retries indefinitely. If the retry count is met, but the Stream has not terminated successfully, a [RetryError](https://pub.dev/documentation/rxdart/latest/rx/RetryError-class.html) will be thrown. The RetryError will contain all of the Errors and StackTraces that caused the failure.

재시도 = 수를 지정하지 않으면 무한 재 시도합니다. 재시도 횟수가 충족되었지만 스트림이 성공적으로 종료되지 않은 경우 [RetryError](https://pub.dev/documentation/rxdart/latest/rx/RetryError-class.html)가 발생합니다. RetryError에는 오류를 일으킨 모든 오류 및 스택 추적이 포함됩니다.

Example

```dart
new Observable.retry(() { new Observable.just(1); })
    .listen((i) => print(i)); // Prints 1

new Observable
   .retry(() {
     new Observable.just(1).concatWith([new Observable.error(new Error())]);
   }, 1)
   .listen(print, onError: (e, s) => print(e)); // Prints 1, 1, RetryError
```

#### Observable<T>.retryWhen

Creates a Stream that will recreate and re-listen to the source Stream when the notifier emits a new value. If the source Stream emits an error or it completes, the Stream terminates.

Notifier가 새로운 값을 발행했을 때에, 소스 Stream를 재생성 해 재 듣는 Stream을 작성합니다. 소스 스트림이 오류를 발생 시키거나 완료되면 스트림이 종료됩니다.

If the `retryWhenFactory` emits an error a [RetryError](https://pub.dev/documentation/rxdart/latest/rx/RetryError-class.html) will be thrown. The RetryError will contain all of the `Error`s and `StackTrace`s that caused the failure.

`retryWhenFactory`가 에러를 내면, [RetryError](https://pub.dev/documentation/rxdart/latest/rx/RetryError-class.html)가 던져 질 것입니다. RetryError는 실패를 일으킨 모든`Error`와`StackTrace`를 포함합니다.

Basic Example

```dart
new RetryWhenStream<int>(
  () => new Stream<int>.fromIterable(<int>[1]),
  (dynamic error, StackTrace s) => throw error,
).listen(print); // Prints 1
```

Periodic Example

```dart
new RetryWhenStream<int>(
  () => new Observable<int>
      .periodic(const Duration(seconds: 1), (int i) => i)
      .map((int i) => i == 2 ? throw 'exception' : i),
  (dynamic e, StackTrace s) {
    return new Observable<String>
        .timer('random value', const Duration(milliseconds: 200));
  },
).take(4).listen(print); // Prints 0, 1, 0, 1
```

Complex Example

```dart
bool errorHappened = false;
new RetryWhenStream(
  () => new Observable
      .periodic(const Duration(seconds: 1), (i) => i)
      .map((i) {
        if (i == 3 && !errorHappened) {
          throw 'We can take this. Please restart.';
        } else if (i == 4) {
          throw 'It\'s enough.';
        } else {
          return i;
        }
      }),
  (e, s) {
    errorHappened = true;
    if (e == 'We can take this. Please restart.') {
      return new Observable.just('Ok. Here you go!');
    } else {
      return new Observable.error(e);
    }
  },
).listen(
  print,
  onError: (e, s) => print(e),
); // Prints 0, 1, 2, 0, 1, 2, 3, RetryError
```

#### Observable<T>.switchLatest 

Convert a Stream that emits Streams (aka a "Higher Order Stream") into a single Observable that emits the items emitted by the most-recently-emitted of those Streams.

스트림 (가장 높은 순서 스트림이라고도 함)을 방출하는 스트림을 가장 최근에 방출 된 스트림에서 방출 된 항목을 방출하는 단일 Observable로 변환하십시오.

This Observable will unsubscribe from the previously-emitted Stream when a new Stream is emitted from the source Stream and subscribe to the new Stream.

이 Observable은 새로운 Stream이 소스 Stream에서 나오고 새로운 Stream에 등록 할 때 이전에 보내진 Stream을 구독 취소합니다.

Example

```dart
final switchLatestStream = new SwitchLatestStream<String>(
  new Stream.fromIterable(<Stream<String>>[
    new Observable.timer('A', new Duration(seconds: 2)),
    new Observable.timer('B', new Duration(seconds: 1)),
    new Observable.just('C'),
  ]),
);

// Since the first two Streams do not emit data for 1-2 seconds, and the
// 3rd Stream will be emitted before that time, only data from the 3rd
// Stream will be emitted to the listener.
switchLatestStream.listen(print); // prints 'C'
```

#### Observable<T>.timer 

Emits the given value after a specified amount of time.

지정된 시간 후에 지정된 값을 내 보냅니다.

Example

```dart
new Observable.timer("hi", new Duration(minutes: 1))
    .listen((i) => print(i)); // print "hi" after 1 minute
```

### 인스턴스 메소드

#### any()

Checks whether `test` accepts any element provided by this stream.

Calls `test` on each element of this stream. If the call returns `true`, the returned future is completed with `true` and processing stops.

If this stream ends without finding an element that `test` accepts, the returned future is completed with `false`.

If this stream emits an error, or if the call to `test` throws, the returned future is completed with that error, and processing stops.

`test`가이 스트림에 의해 제공되는 요소를 받아 들일지 어떨지를 판정합니다.

이 스트림의 각 요소에 대해`test`를 호출합니다. 호출이`true`를 반환하면 반환 된 미래는`true`로 완료되고 처리가 중지됩니다.

이 스트림이`test`가 받아 들일 수있는 요소를 찾지 못하고 끝나면 반환 된 미래는`false`로 끝납니다.

이 스트림이 오류를 발생 시키거나`test '호출이 발생하면 반환 된 미래가 해당 오류로 완료되고 처리가 중지됩니다.

#### asBroadcastStream()

Returns a multi-subscription stream that produces the same events as this.

The returned stream will subscribe to this stream when its first subscriber is added, and will stay subscribed until this stream ends, or a callback cancels the subscription.

If onListen is provided, it is called with a subscription-like object that represents the underlying subscription to this stream. It is possible to pause, resume or cancel the subscription during the call to onListen. It is not possible to change the event handlers, including using StreamSubscription.asFuture.

If onCancel is provided, it is called in a similar way to onListen when the returned stream stops having listener. If it later gets a new listener, the onListen function is called again.

Use the callbacks, for example, for pausing the underlying subscription while having no subscribers to prevent losing events, or canceling the subscription when there are no listeners.

이 이벤트와 동일한 이벤트를 생성하는 다중 구독 스트림을 반환합니다.

반환 된 스트림은 첫 번째 구독자가 추가 될 때이 스트림을 구독하고이 스트림이 끝날 때까지 구독 상태를 유지하거나 콜백이 구독을 취소 할 때까지 구독 상태를 유지합니다.

onListen가 제공되면,이 스트림에 대한 기본 구독을 나타내는 구독과 유사한 객체로 호출됩니다. onListen을 호출하는 동안 구독을 일시 중지, 다시 시작 또는 취소 할 수 있습니다. StreamSubscription.asFuture를 사용하는 것을 포함하여 이벤트 핸들러를 변경할 수 없습니다.

onCancel가 제공되면 반환되는 스트림에서 수신기가 없어지면 onListen과 비슷한 방식으로 호출됩니다. 나중에 새 리스너를 얻으면 onListen 함수가 다시 호출됩니다.

예를 들어 기본 구독을 일시 중지하고 구독자가 없어도 이벤트가 손실되지 않도록 콜백을 사용하거나 수신기가 없을 때 구독을 취소 할 수 있습니다.

#### asyncExpand<S>()

Maps each emitted item to a new `Stream` using the given mapper, then subscribes to each new stream one after the next until all values are emitted.

asyncExpand is similar to flatMap, but ensures order by guaranteeing that all items from the created stream will be emitted before moving to the next created stream. This process continues until all created streams have completed.

This is functionally equivalent to `concatMap`, which exists as an alias for a more fluent Rx API.

지정된 매퍼를 사용해, 배출 된 각 항목을 새로운`Stream`에 매핑 해, 다음에 새로운 스트림을 1 개씩 구독 해, 모든 값이 출력 될 때까지 구독합니다.

asyncExpand는 flatMap과 비슷하지만 생성 된 스트림의 모든 항목이 다음에 생성 된 스트림으로 이동하기 전에 내보내도록 보장하여 순서를 보장합니다. 생성 된 모든 스트림이 완료 될 때까지이 프로세스가 계속됩니다.

이는 기능적으로 concatMap과 동일합니다. concatMap은보다 유창한 Rx API의 별명으로 존재합니다.

Example

```dart
Observable.range(4, 1)
  .asyncExpand((i) =>
    new Observable.timer(i, new Duration(minutes: i))
  .listen(print); // prints 4, 3, 2, 1
```

#### asyncMap<S>()

Creates an Observable with each data event of this stream asynchronously mapped to a new event.

This acts like map, except that convert may return a Future, and in that case, the stream waits for that future to complete before continuing with its result.

The returned stream is a broadcast stream if this stream is.

이 스트림의 각 데이터 이벤트가 비동기 적으로 새 이벤트에 매핑되는 Observable을 작성합니다.

이것은 맵과 같은 역할을하지만, convert가 Future를 반환 할 수 있다는 점을 제외하고, 스트림은 그 결과가 계속되기 전에 완료 될 때까지 기다립니다.

반환 된 스트림은 이 스트림이있는 경우 브로드 캐스트 스트림입니다.

#### buffer()

Creates an Observable where each item is a `List` containing the items from the source sequence.

This `List` is emitted every time `window` emits an event.

각 항목이 소스 시퀀스의 항목을 포함하는`List `인 Observable을 만듭니다.

이`List`는`window`가 이벤트를 낼 때마다 발생합니다.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (i) => i)
  .buffer(new Stream.periodic(const Duration(milliseconds: 160), (i) => i))
  .listen(print); // prints [0, 1] [2, 3] [4, 5] ...
```

#### bufferCount()

Buffers a number of values from the source Observable by `count` then emits the buffer and clears it, and starts a new buffer each `startBufferEvery` values. If `startBufferEvery` is not provided, then new buffers are started immediately at the start of the source and when each buffer closes and is emitted.

`count`에 의해 Observable 소스로부터의 많은 값을 버퍼링하고 버퍼를 방출하고 지우고, 각각의 새로운 버퍼를 startBufferEvery 값으로 시작합니다. `startBufferEvery`가 제공되지 않으면, 소스의 시작과 각 버퍼가 닫히고 나올 때 즉시 새로운 버퍼가 시작됩니다.

Example

`count` is the maximum size of the buffer emitted

```dart
Observable.range(1, 4)
  .bufferCount(2)
  .listen(print); // prints [1, 2], [3, 4] done!
```

Example

if `startBufferEvery` is 2, then a new buffer will be started on every other value from the source. A new buffer is started at the beginning of the source by default.

```dart
Observable.range(1, 5)
  .bufferCount(3, 2)
  .listen(print); // prints [1, 2, 3], [3, 4, 5], [5] done!
```

#### bufferTest()

Creates an Observable where each item is a `List` containing the items from the source sequence, batched whenever test passes.

Observable을 생성합니다. 각 항목은 테스트가 통과 할 때마다 소스 시퀀스의 항목을 포함하는`List`입니다.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (int i) => i)
  .bufferTest((i) => i % 2 == 0)
  .listen(print); // prints [0], [1, 2] [3, 4] [5, 6] ...
```

#### bufferTime()

Creates an Observable where each item is a `List` containing the items from the source sequence, sampled on a time frame with `duration`.

Observable을 만듭니다. 각 항목은 소스 시퀀스의 항목을 포함하는`List`이며, 시간 프레임에서`duration`으로 샘플링됩니다.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (int i) => i)
  .bufferTime(const Duration(milliseconds: 220))
  .listen(print); // prints [0, 1] [2, 3] [4, 5] ...
```

#### cast<R>()

Adapt this stream to be a `Stream<R>`.

If this stream already has the desired type, its returned directly. Otherwise it is wrapped as a `Stream<R>` which checks at run-time that each data event emitted by this stream is also an instance of `R`.

이 스트림을`Stream <R>`으로 변경하십시오.

이 스트림에 이미 원하는 형식이 있으면 해당 형식이 직접 반환됩니다. 그렇지 않은 경우는, 실행시에,이 스트림에 의해 발행 된 각 데이터 이벤트가`R` 인스턴스이기도 어떤가를 조사하는`Stream <R>`로서 랩됩니다.

#### concatMap<S>()

Maps each emitted item to a new `Stream` using the given mapper, then subscribes to each new stream one after the next until all values are emitted.

ConcatMap is similar to flatMap, but ensures order by guaranteeing that all items from the created stream will be emitted before moving to the next created stream. This process continues until all created streams have completed.

This is a simple alias for Dart Stream's `asyncExpand`, but is included to ensure a more consistent Rx API.

지정된 매퍼를 사용해, 배출 된 각 항목을 새로운`Stream`에 매핑 해, 다음에 새로운 스트림을 1 개씩 구독 해, 모든 값이 출력 될 때까지 구독합니다.

ConcatMap은 flatMap과 비슷하지만 생성 된 스트림의 모든 항목이 다음에 생성 된 스트림으로 이동하기 전에 방출되도록 보장하여 순서를 보장합니다. 생성 된 모든 스트림이 완료 될 때까지이 프로세스가 계속됩니다.

이것은 Dart Stream의 asyncExpand에 대한 단순한 별명이지만 더 일관된 Rx API를 보장하기 위해 포함됩니다.

Example

```dart
Observable.range(4, 1)
  .concatMap((i) =>
    new Observable.timer(i, new Duration(minutes: i))
  .listen(print); // prints 4, 3, 2, 1
```

#### concatWith()

Returns an Observable that emits all items from the current Observable, then emits all items from the given observable, one after the next.

현재 Observable에서 모든 항목을 내보내는 Observable을 반환 한 다음 주어진 관찰 가능 항목에서 다음 항목까지 하나씩 모든 항목을 내 보냅니다.

Example

```dart
new Observable.timer(1, new Duration(seconds: 10))
    .concatWith([new Observable.just(2)])
    .listen(print); // prints 1, 2
```

#### contains()

Returns whether `needle` occurs in the elements provided by this stream.

Compares each element of this stream to `needle` using `Object.==`. If an equal element is found, the returned future is completed with `true`. If this stream ends without finding a match, the future is completed with `false`.

If this stream emits an error, or the call to `Object.==` throws, the returned future is completed with that error, and processing stops.

이 스트림에 의해 제공되는 요소로 「니들」이 발생할지 어떨지를 돌려줍니다.

이 스트림의 각 요소를`Object. ==`를 사용해`needle '와 비교합니다. 동일한 원소가 발견되면 반환 된 미래는 'true'로 완료됩니다. 이 스트림이 일치하지 않고 끝나면 미래는 'false'로 완료됩니다.

이 스트림이 에러를 발생 시키거나`Object. ==`를 호출하면 그 에러로 반환 된 미래가 완료되고 처리가 중지됩니다.

#### debounce()

Transforms a `Stream` so that will only emit items from the source sequence if a `window` has completed, without the source sequence emitting another item.

This `window` is created after the last debounced event was emitted. You can use the value of the last debounced event to determine the length of the next `window`.

A `window` is open until the first `window` event emits.

debounce filters out items emitted by the source [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) that are rapidly followed by another emitted item.

'스트림'을 변환하여 소스 시퀀스가 다른 항목을 방출하지 않고 '창'이 완료된 경우에만 소스 시퀀스의 항목을 방출합니다.

이 '창'은 마지막 디 바운싱 이벤트가 생성 된 후에 생성됩니다. 마지막 debounced 이벤트의 값을 사용하여 다음 '창의'길이를 결정할 수 있습니다.

`window`는 첫 번째`window` 이벤트가 발생할 때까지 열려 있습니다.

디 바운스 (obounce)는 출처 [Observable] (https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html)에 의해 방출 된 항목을 필터링하여 다른 방출 된 항목이 빠르게 뒤 따릅니다.

[Interactive marble diagram](http://rxmarbles.com/#debounce)

Example

```dart
new Observable.fromIterable([1, 2, 3, 4])
  .debounce((_) => TimerStream(true, const Duration(seconds: 1)))
  .listen(print); // prints 4
```

#### debounceTime()

transforms a `Stream` so that will only emit items from the source sequence whenever the time span defined by `duration` passes, without the source sequence emitting another item.

This time span start after the last debounced event was emitted.

debounceTime filters out items emitted by the source [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) that are rapidly followed by another emitted item.

소스 시퀀스가 다른 아이템을 방출하지 않으면 서 'duration'에 의해 정의 된 시간 범위가 지나갈 때마다 소스 시퀀스에서 아이템을 방출하도록 '스트림'을 변환합니다.

마지막 디 바운스 이벤트가 발생한 후이 시간 범위 시작.

debounceTime은 [Observable] (https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) 소스에 의해 방출 된 항목을 필터링하여 다른 방출 된 항목을 빠르게 찾습니다.

[Interactive marble diagram](http://rxmarbles.com/#debounce)

Example

```dart
new Observable.fromIterable([1, 2, 3, 4])
  .debounceTime(const Duration(seconds: 1))
  .listen(print); // prints 4
```

#### defaultIfEmpty()

Emit items from the source Stream, or a single default item if the source Stream emits nothing.

원본 Stream에서 항목을 내보내거나 원본 Stream에서 아무 것도 방출하지 않으면 단일 기본 항목을 내 보냅니다.

Example

```dart
new Observable.empty().defaultIfEmpty(10).listen(print); // prints 10
```

#### delay()

The Delay operator modifies its source Observable by pausing for a particular increment of time (that you specify) before emitting each of the source Observable’s items. This has the effect of shifting the entire sequence of items emitted by the Observable forward in time by that specified increment.

지연 연산자는 소스를 수정합니다. Observable 소스의 각 항목을 방출하기 전에 특정 시간 증가분 (지정한 값) 동안 일시 중지함으로써 관찰 가능합니다. 이것은 Observable에 의해 방출 된 항목의 전체 시퀀스를 지정된 증가분만큼 시간상으로 이동시키는 효과가 있습니다.

[Interactive marble diagram](http://rxmarbles.com/#delay)

Example

```dart
new Observable.fromIterable([1, 2, 3, 4])
  .delay(new Duration(seconds: 1))
  .listen(print); // [after one second delay] prints 1, 2, 3, 4 immediately
```

#### dematerialize<S>()

Converts the onData, onDone, and onError [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects from a materialized stream into normal onData, onDone, and onError events.

onData, onDone 및 onError [Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 객체를 materialized 스트림에서 일반 onData, onDone 및 onError 이벤트로 변환합니다.

When a stream has been materialized, it emits onData, onDone, and onError events as [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects. Dematerialize simply reverses this by transforming [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects back to a normal stream of events.

스트림이 구체화되면 onData, onDone 및 onError 이벤트가 [Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 객체로 방출됩니다. Dematerialize는 [Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 객체를 정상적인 이벤트 스트림으로 변환하여이를 간단히 되돌립니다.

Example

```dart
new Observable<Notification<int>>
    .fromIterable([new Notification.onData(1), new Notification.onDone()])
    .dematerialize()
    .listen((i) => print(i)); // Prints 1
```

Error example

```dart
new Observable<Notification<int>>
    .just(new Notification.onError(new Exception(), null))
    .dematerialize()
    .listen(null, onError: (e, s) { print(e) }); // Prints Exception
```

#### distinct()

WARNING: More commonly known as distinctUntilChanged in other Rx implementations. Creates an Observable where data events are skipped if they are equal to the previous data event.

경고 : 다른 Rx 구현에서는 일반적으로 distinctUntilChanged로 알려져 있습니다. 이전 데이터 이벤트와 동일한 경우 데이터 이벤트를 건너 뛸 Observable을 만듭니다.

The returned stream provides the same events as this stream, except that it never provides two consecutive data events that are equal.

돌려 주어지는 스트림은,이 스트림과 같은 이벤트를 제공합니다. 다만, 2 개의 연속 한 데이터 이벤트는 결코 제공되지 않습니다.

Equality is determined by the provided equals method. If that is omitted, the '==' operator on the last provided data element is used.

동등성은 제공된 equals 메서드에 의해 결정됩니다. 이것이 생략되면, 마지막으로 제공된 데이터 요소의 '=='연산자가 사용됩니다.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the equals test.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림이 두 번 이상 수신되면 각 구독은 개별적으로 동등 테스트를 수행합니다.

[Interactive marble diagram](http://rxmarbles.com/#distinctUntilChanged)

#### distinctUnique()

WARNING: More commonly known as distinct in other Rx implementations. Creates an Observable where data events are skipped if they have already been emitted before.

경고 : 다른 Rx 구현에서보다 일반적으로 뚜렷한 것으로 알려져 있습니다. Observable를 작성합니다. Observable는, 전에 이미 방아쇠되었을 경우, 데이터 이벤트는 스킵됩니다.

Equality is determined by the provided equals and hashCode methods. If these are omitted, the '==' operator and hashCode on the last provided data element are used.

동등성은 제공된 equals 및 hashCode 메소드에 의해 결정됩니다. 이것들을 생략하면 마지막에 제공된 데이터 요소의 '=='연산자와 hashCode가 사용됩니다.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the equals and hashCode tests.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림이 두 번 이상 수신되면 각 구독은 equals 및 hashCode 테스트를 개별적으로 수행합니다.

[Interactive marble diagram](http://rxmarbles.com/#distinct)

#### doOnCancel()

Invokes the given callback function when the stream subscription is cancelled. Often called doOnUnsubscribe or doOnDispose in other implementations.

스트림 구독이 취소 될 때 지정된 콜백 함수를 호출합니다. 종종 다른 구현에서는 doOnUnsubscribe 또는 doOnDispose라고합니다.

Example

```dart
final subscription = new Observable.timer(1, new Duration(minutes: 1))
  .doOnCancel(() => print("hi"));
  .listen(null);

subscription.cancel(); // prints "hi"
```

#### doOnData()

Invokes the given callback function when the stream emits an item. In other implementations, this is called doOnNext.

스트림이 항목을 내보낼 때 지정된 콜백 함수를 호출합니다. 다른 구현에서는 이것을 doOnNext라고합니다.

Example

```dart
new Observable.fromIterable([1, 2, 3])
  .doOnData(print)
  .listen(null); // prints 1, 2, 3
```

#### doOnDone()

Invokes the given callback function when the stream finishes emitting items. In other implementations, this is called doOnComplete(d).

스트림이 항목을 출력 할 때 지정된 콜백 함수를 호출합니다. 다른 구현에서는 이것을 doOnComplete (d)라고합니다.

Example

```dart
new Observable.fromIterable([1, 2, 3])
  .doOnDone(() => print("all set"))
  .listen(null); // prints "all set"
```

#### doOnEach

Invokes the given callback function when the stream emits data, emits an error, or emits done. The callback receives a [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) object.

스트림이 데이터를 내보내거나 오류가 발생하거나 완료되면 지정된 콜백 함수를 호출합니다. 콜백은 [Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 객체를받습니다.

The [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) object contains the [Kind](https://pub.dev/documentation/rxdart/latest/rx/Kind-class.html) of event (OnData, onDone, or OnError), and the item or error that was emitted. In the case of onDone, no data is emitted as part of the [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html).

[Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 객체에는 [종류] (https://pub.dev/documentation/rxdart/latest/rx/)가 있습니다. Kind-class.html), 발생한 항목 또는 오류 (OnData, onDone 또는 OnError) onDone의 경우 [알림] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html)의 일부로 데이터가 방출되지 않습니다.

Example

```dart
new Observable.just(1)
  .doOnEach(print)
  .listen(null); // prints Not
```

#### doOnError()

Invokes the given callback function when the stream emits an error.

스트림이 오류를 내면 지정된 콜백 함수를 호출합니다.

Example

```dart
new Observable.error(new Exception())
  .doOnError((error, stacktrace) => print("oh no"))
  .listen(null); // prints "Oh no"
```

#### doOnListen()

Invokes the given callback function when the stream is first listened to.

스트림을 처음 청취 할 때 지정된 콜백 함수를 호출합니다.

Example

```dart
new Observable.just(1)
  .doOnListen(() => print("Is someone there?"))
  .listen(null); // prints "Is someone there?"
```

#### doOnPause()

Invokes the given callback function when the stream subscription is paused.

스트림 구독이 일시 중지 된 경우 지정된 콜백 함수를 호출합니다.

Example

```dart
final subscription = new Observable.just(1)
  .doOnPause(() => print("Gimme a minute please"))
  .listen(null);

subscription.pause(); // prints "Gimme a minute please"
```

#### doOnResume()

Invokes the given callback function when the stream subscription resumes receiving items.

스트림 구독이 항목 수신을 다시 시작할 때 주어진 콜백 함수를 호출합니다.

Example

```dart
final subscription = new Observable.just(1)
  .doOnResume(() => print("Let's do this!"))
  .listen(null);

subscription.pause();
subscription.resume(); "Let's do this!"
```

#### rain<S>()

Discards all data on this stream, but signals when it is done or an error occurred.

이 스트림의 모든 데이터를 삭제하지만 완료되거나 오류가 발생한 경우 신호를 보냅니다.

When subscribing using [drain](https://pub.dev/documentation/rxdart/latest/rx/Observable/drain.html), cancelOnError will be true. This means that the future will complete with the first error on this stream and then cancel the subscription. If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

[drain] (https://pub.dev/documentation/rxdart/latest/rx/Observable/drain.html)을 사용하여 구독하면 cancelOnError가 true가됩니다. 이것은 미래가이 스트림의 첫 번째 오류로 완료되고 구독을 취소한다는 것을 의미합니다. 이 스트림에서 오류가 발생하거나`combine '호출이 throw되면 해당 오류로 반환 된 미래가 완료되고 처리가 중지됩니다.

In case of a `done` event the future completes with the given `futureValue`.

`done` 이벤트의 경우 미래는 주어진`futureValue`로 완료됩니다.

#### elementAt()

Returns the value of the `index`th data event of this stream.

Stops listening to this stream after the `index`th data event has been received.

Internally the method cancels its subscription after these elements. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

If an error event occurs before the value is found, the future completes with this error.

If a done event occurs before the value is found, the future completes with a `RangeError`.

이 스트림의`index` 번째 데이터 이벤트의 값을 리턴합니다.

`index` 데이터 이벤트가 수신 된 후이 스트림을 청취하지 않습니다.

내부적으로 메소드는 이러한 요소 다음에 구독을 취소합니다. 이것은 단일 구독 (비 브로드 캐스트) 스트림이 닫히고이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

값이 발견되기 전에 오류 이벤트가 발생하면이 오류로 인해 미래가 완료됩니다.

값이 발견되기 전에 완료 이벤트가 발생하면 미래는 'RangeError'로 완료됩니다.

#### every()

Checks whether `test` accepts all elements provided by this stream.

Calls `test` on each element of this stream. If the call returns `false`, the returned future is completed with `false` and processing stops.

If this stream ends without finding an element that `test` rejects, the returned future is completed with `true`.

If this stream emits an error, or if the call to `test` throws, the returned future is completed with that error, and processing stops.

`test`가이 스트림에 의해 제공되는 모든 요소를 받아들이는지 어떤지를 판정합니다.

이 스트림의 각 요소에 대해`test`를 호출합니다. 호출이`false`를 반환하면, 반환 된 미래는`false`로 끝나고 처리는 멈 춥니 다.

이 스트림이`test`가 거부하는 요소를 찾지 않고 끝나면 리턴 된 미래는`true`로 완료됩니다.

이 스트림이 오류를 발생 시키거나`test '호출이 발생하면 반환 된 미래가 해당 오류로 완료되고 처리가 중지됩니다.

#### exhaustMap<S>()

Converts items from the source stream into a new Stream using a given mapper. It ignores all items from the source stream until the new stream completes.

Useful when you have a noisy source Stream and only want to respond once the previous async operation is finished.

지정된 매퍼를 사용하여 소스 스트림의 항목을 새 스트림으로 변환합니다. 새 스트림이 완료 될 때까지 원본 스트림의 모든 항목을 무시합니다.

시끄러운 소스 스트림이있을 때 유용하며 이전 비동기 작업이 끝나면 응답해야합니다.

Example

```dart
Observable.range(0, 2).interval(new Duration(milliseconds: 50))
  .exhaustMap((i) =>
    new Observable.timer(i, new Duration(milliseconds: 75)))
  .listen(print); // prints 0, 2
```

#### expand<S>()

Creates an Observable from this stream that converts each element into zero or more events.

Each incoming event is converted to an Iterable of new events, and each of these new events are then sent by the returned Observable in order.

The returned Observable is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually call convert and expand the events.

각 요소를 0 개 이상의 이벤트로 변환하는 Observable을이 스트림에서 작성합니다.

들어오는 각 이벤트는 새로운 이벤트의 반복 가능으로 변환되고 이러한 새 이벤트 각각은 반환 된 Observable에 의해 순서대로 전송됩니다.

반환 된 Observable은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림이 두 번 이상 수신되면 각 구독은 개별적으로 변환을 호출하고 이벤트를 확장합니다.

#### firstWhere()

Finds the first element of this stream matching `test`.

Returns a future that is completed with the first element of this stream that `test` returns `true` for.

If no such element is found before this stream is done, and a `orElse` function is provided, the result of calling `orElse` becomes the value of the future. If `orElse` throws, the returned future is completed with that error.

If this stream emits an error before the first matching element, the returned future is completed with that error, and processing stops.

Stops listening to this stream after the first matching element or error has been received.

Internally the method cancels its subscription after the first element that matches the predicate. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

If an error occurs, or if this stream ends without finding a match and with no `orElse` function provided, the returned future is completed with an error.

`test`와 일치하는이 스트림의 첫 번째 요소를 찾습니다.

`test`가`true`를 리턴하는이 스트림의 첫 번째 요소로 완료되는 미래를 리턴합니다.

이 스트림이 끝나기 전에 그러한 요소가 발견되지 않고`orElse` 함수가 제공되면,`orElse`를 호출 한 결과는 미래의 값이됩니다. `orElse`가 throw되면 리턴 된 미래가 그 에러로 완료됩니다.

이 스트림이 첫 번째 일치하는 요소 앞에 오류를 내면 반환 된 미래가 해당 오류로 완료되고 처리가 중지됩니다.

최초로 일치하는 요소 또는 에러가 수신 된 후,이 스트림의 수신을 정지합니다.

내부적으로 메소드는 술어와 일치하는 첫 x 째 요소 다음에 서브 스크립 션을 취소합니다. 이것은 단일 구독 (비 브로드 캐스트) 스트림이 닫히고이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

오류가 발생하거나이 스트림이 일치하지 않고 종료되었거나`orElse` 함수가 제공되지 않으면 반환 된 미래가 오류와 함께 완료됩니다.

#### flatMap<S>

Converts each emitted item into a new Stream using the given mapper function. The newly created Stream will be be listened to and begin emitting items downstream.

The items emitted by each of the new Streams are emitted downstream in the same order they arrive. In other words, the sequences are merged together.

지정된 매퍼 함수를 사용하여 방출 된 각 항목을 새 스트림으로 변환합니다. 새로 생성 된 스트림은 청취되어 다운 스트림 항목을 시작합니다.

새로운 각 스트림에 의해 방출 된 항목은 도착한 순서와 동일한 순서로 다운 스트림으로 방출됩니다. 즉, 시퀀스가 병합됩니다.

Example

```dart
Observable.range(4, 1)
  .flatMap((i) =>
    new Observable.timer(i, new Duration(minutes: i))
  .listen(print); // prints 1, 2, 3, 4
```

#### flatMapIterable<S>()

Converts each item into a new Stream. The Stream must return an Iterable. Then, each item from the Iterable will be emitted one by one.

Use case: you may have an API that returns a list of items, such as a Stream<List>. However, you might want to operate on the individual items rather than the list itself. This is the job of `flatMapIterable`.

각 항목을 새 스트림으로 변환합니다. Stream은 Iterable을 반환해야합니다. 그런 다음, Iterable의 각 항목이 하나씩 배출됩니다.

사용 사례 : 스트림 <List>과 같은 항목 목록을 반환하는 API가있을 수 있습니다. 그러나 목록 자체가 아닌 개별 항목에 대한 작업을 원할 수 있습니다. 이것은 'flatMapIterable`의 작업입니다.

Example

```dart
Observable.range(1, 4)
  .flatMapIterable((i) =>
    new Observable.just([i])
  .listen(print); // prints 1, 2, 3, 4
```

#### fold<S>()

Combines a sequence of values by repeatedly applying `combine`.

Similar to [Iterable.fold](https://pub.dev/documentation/rxdart/latest/rx/Observable/fold.html), this function maintains a value, starting with `initialValue` and updated for each element of this stream. For each element, the value is updated to the result of calling `combine` with the previous value and the element.

When this stream is done, the returned future is completed with the value at that time. For an empty stream, the future is completed with `initialValue`.

If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

`combine '를 반복적으로 적용하여 값의 시퀀스를 결합합니다.

[Iterable.fold] (https://pub.dev/documentation/rxdart/latest/rx/Observable/fold.html)와 마찬가지로이 함수는 'initialValue'로 시작하여이 스트림의 각 요소에 대해 업데이트 된 값을 유지합니다. . 각 요소에 대해 값은 이전 값 및 요소와`combine '를 호출 한 결과로 업데이트됩니다.

이 스트림이 완료되면 반환 된 미래가 그 시점의 값으로 완료됩니다. 빈 스트림의 경우, future는`initialValue`로 완료됩니다.

이 스트림에서 오류가 발생하거나`combine '호출이 throw되면 해당 오류로 반환 된 미래가 완료되고 처리가 중지됩니다.

#### forEach()

Executes `action` on each element of this stream.

Completes the returned `Future` when all elements of this stream have been processed.

If this stream emits an error, or if the call to `action` throws, the returned future completes with that error, and processing stops.

이 스트림의 각 요소에 대해`action`을 실행합니다.

이 스트림의 모든 요소가 처리 될 때 반환 된 'Future'를 완료합니다.

이 스트림이 오류를 발생 시키거나`action`에 대한 호출이 발생하면 반환 된 미래는 그 오류로 완료되고 처리가 중지됩니다.

#### groupBy<S>()

The GroupBy operator divides an [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) that emits items into an [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) that emits [GroupByObservable](https://pub.dev/documentation/rxdart/latest/rx/GroupByObservable-class.html), each one of which emits some subset of the items from the original source [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html).

[GroupByObservable](https://pub.dev/documentation/rxdart/latest/rx/GroupByObservable-class.html) acts like a regular [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html), yet adding a 'key' property, which receives its `Type` and value from the `grouper` Function.

All items with the same key are emitted by the same [GroupByObservable](https://pub.dev/documentation/rxdart/latest/rx/GroupByObservable-class.html).

#### handleError()

Creates a wrapper Stream that intercepts some errors from this stream.

If this stream sends an error that matches test, then it is intercepted by the handle function.

The onError callback must be of type void onError(error) or void onError(error, StackTrace stackTrace). Depending on the function type the stream either invokes onError with or without a stack trace. The stack trace argument might be null if the stream itself received an error without stack trace.

An asynchronous error e is matched by a test function if test(e) returns true. If test is omitted, every error is considered matching.

If the error is intercepted, the handle function can decide what to do with it. It can throw if it wants to raise a new (or the same) error, or simply return to make the stream forget the error.

If you need to transform an error into a data event, use the more generic Stream.transform to handle the event by writing a data event to the output sink.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the test and handle the error.

이 스트림의 일부 오류를 가로 채기위한 래퍼 Stream을 작성합니다.

이 스트림이 test와 일치하는 오류를 보내면 핸들 함수가 인터셉트합니다.

onError 콜백은 void onError (error) 또는 void onError (error, StackTrace stackTrace) 유형이어야합니다. 함수 유형에 따라 스트림은 스택 추적이 있거나없는 onError를 호출합니다. 스트림 자체가 스택 추적없이 오류를 수신하면 스택 추적 인수가 널 (NULL) 일 수 있습니다.

비동기 오류 e는 test (e)가 true를 반환하면 테스트 함수와 일치합니다. test가 생략되면 모든 오류가 일치하는 것으로 간주됩니다.

오류가 인터셉트되면, 핸들 기능은 처리 기능을 결정할 수 있습니다. 새로운 (또는 같은) 에러를 일으키고 싶다면, 또는 단순히 스트림이 에러를 잊어 버리기 위해 리턴한다면 던져 질 수 있습니다.

오류를 데이터 이벤트로 변환해야하는 경우보다 일반적인 Stream.transform을 사용하여 데이터 이벤트를 출력 싱크에 기록하여 이벤트를 처리하십시오.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림이 두 번 이상 수신되면 각 구독은 개별적으로 테스트를 수행하고 오류를 처리합니다.

#### ignoreElements()

Creates an Observable where all emitted items are ignored, only the error / completed notifications are passed

모든 방출 된 항목이 무시되고 오류 / 완료된 알림 만 전달되는 Observable을 만듭니다.

Example

```dart
new Observable.merge(
    new Observable.just(1),
    new Observable.error(new Exception())
).listen(print, onError: print); // prints Exception
```

#### interval()

Creates an Observable that emits each item in the Stream after a given duration.

지정된 지속 기간 후 Stream의 각 항목을 내보내는 Observable을 만듭니다.

Example

```dart
new Observable.fromIterable([1, 2, 3])
  .interval(new Duration(seconds: 1))
  .listen((i) => print("$i sec"); // prints 1 sec, 2 sec, 3 sec
```

#### join()

Combines the string representation of elements into a single string.

Each element is converted to a string using its `Object.toString` method. If `separator` is provided, it is inserted between element string representations.

The returned future is completed with the combined string when this stream is done.

If this stream emits an error, or the call to `Object.toString` throws, the returned future is completed with that error, and processing stops.

요소의 문자열 표현을 단일 문자열로 결합합니다.

각 요소는`Object.toString` 메소드를 사용하여 문자열로 변환됩니다. `separator '가 제공되면 요소 문자열 표현 사이에 삽입됩니다.

이 스트림이 완료되면 반환 된 미래는 결합 된 문자열로 완료됩니다.

이 스트림이 에러를 발행했을 경우, 또는`Object.toString`의 호출이 슬로우되었을 경우, 그 에러로 반환 된 장래가 완료 해 처리는 정지합니다.

#### lastWhere()

Finds the last element in this stream matching `test`.

If this stream emits an error, the returned future is completed with that error, and processing stops.

Otherwise as [firstWhere](https://pub.dev/documentation/rxdart/latest/rx/Observable/firstWhere.html), except that the last matching element is found instead of the first. That means that a non-error result cannot be provided before this stream is done.

이 스트림 내에서`test`와 일치하는 마지막 요소를 찾습니다.

이 스트림에서 오류가 발생하면 반환 된 오류가 해당 오류와 함께 완료되고 처리가 중지됩니다.

그렇지 않으면 [firstWhere] (https://pub.dev/documentation/rxdart/latest/rx/Observable/firstWhere.html)와 같지만 첫 번째 일치 요소 대신 마지막으로 일치하는 요소가 발견됩니다. 즉,이 스트림이 완료되기 전에 오류가 아닌 결과를 제공 할 수 없습니다.

#### listen()

Adds a subscription to this stream. Returns a `StreamSubscription` which handles events from the stream using the provided `onData`, `onError` and `onDone` handlers.

The handlers can be changed on the subscription, but they start out as the provided functions.

On each data event from this stream, the subscriber's `onData` handler is called. If `onData` is `null`, nothing happens.

On errors from this stream, the `onError` handler is called with the error object and possibly a stack trace.

The `onError` callback must be of type `void onError(error)` or `void onError(error, StackTrace stackTrace)`. If `onError` accepts two arguments it is called with the error object and the stack trace (which could be `null` if the stream itself received an error without stack trace). Otherwise it is called with just the error object. If `onError` is omitted, any errors on the stream are considered unhandled, and will be passed to the current `Zone`'s error handler. By default unhandled async errors are treated as if they were uncaught top-level errors.

If this stream closes and sends a done event, the `onDone` handler is called. If `onDone` is `null`, nothing happens.

If `cancelOnError` is true, the subscription is automatically cancelled when the first error event is delivered. The default is `false`.

While a subscription is paused, or when it has been cancelled, the subscription doesn't receive events and none of the event handler functions are called.

이 스트림에 구독을 추가합니다. 제공된`onData`,`onError` 및`onDone` 핸들러를 사용하여 스트림의 이벤트를 처리하는`StreamSubscription`을 리턴합니다.

핸들러는 서브 스크립 션에서 변경할 수 있지만 제공된 기능으로 시작합니다.

이 스트림의 각 데이터 이벤트에서 가입자의 onData 처리기가 호출됩니다. `onData`가`null`이라면 아무 일도 일어나지 않습니다.

이 스트림의 오류가 발생하면 onError 핸들러가 오류 객체 및 스택 추적과 함께 호출됩니다.

onError 콜백은 반드시 void onError (error) 또는 void onError (error, StackTrace stackTrace) 형이어야합니다. `onError`가 두 개의 인자를 받아 들인다면 에러 객체와 스택 트레이스와 함께 호출됩니다 (스트림 자체가 스택 추적없이 에러를 받으면`null '이 될 수 있습니다). 그렇지 않으면 오류 오브젝트만으로 호출됩니다. `onError`가 생략되면, 스트림의 모든 에러는 처리되지 않은 것으로 간주되어 현재의`Zone` 에러 핸들러로 전달 될 것입니다. 기본적으로 처리되지 않은 비동기 오류는 캐치되지 않은 최상위 오류 인 것처럼 처리됩니다.

이 스트림이 닫히고 done 이벤트를 보내면 onDone 핸들러가 호출됩니다. `onDone`이`null`이라면 아무 일도 일어나지 않습니다.

`cancelOnError`가 참이면 첫 x 째 오류 이벤트가 전달되면 서브 스크립 션이 자동으로 취소됩니다. 기본값은`false`입니다.

구독이 일시 중지되었거나 취소되었을 때 구독은 이벤트를받지 않으며 이벤트 처리기 기능이 호출되지 않습니다.

Example

```dart
new Observable.just(1).listen(print); // prints 1
```

#### map<S>()

Maps values from a source sequence through a function and emits the returned values.

The returned sequence completes when the source sequence completes. The returned sequence throws an error if the source sequence throws an error.

소스 시퀀스의 값을 함수를 통해 매핑하고 반환 된 값을 내 보냅니다.

리턴 된 시퀀스는 소스 시퀀스가 완료 될 때 완료됩니다. 원본 시퀀스에서 오류가 발생하면 반환 된 시퀀스에서 오류가 발생합니다.

#### mapTo<S>()

Emits the given constant value on the output Observable every time the source Observable emits a value.

Observable 소스가 값을 내보낼 때마다 관찰 가능한 출력에 주어진 상수 값을 내 보냅니다.

Example

```dart
Observable.fromIterable([1, 2, 3, 4])
  .mapTo(true)
  .listen(print); // prints true, true, true, true
```

#### materialize()

Converts the onData, on Done, and onError events into [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects that are passed into the downstream onData listener.

The [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) object contains the [Kind](https://pub.dev/documentation/rxdart/latest/rx/Kind-class.html) of event (OnData, onDone, or OnError), and the item or error that was emitted. In the case of onDone, no data is emitted as part of the [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html).

onData, Done 및 onError 이벤트를 다운 스트림 onData 수신기에 전달 된 [Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 개체로 변환합니다.

[Notification] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) 객체에는 [종류] (https://pub.dev/documentation/rxdart/latest/rx/)가 있습니다. Kind-class.html), 발생한 항목 또는 오류 (OnData, onDone 또는 OnError) onDone의 경우 [알림] (https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html)의 일부로 데이터가 방출되지 않습니다.

Example:

```dart
new Observable<int>.error(new Exception())
    .materialize()
    .listen((i) => print(i)); // Prints onError Notification
```

#### max()

Converts a Stream into a Future that completes with the largest item emitted by the Stream.

This is similar to finding the max value in a list, but the values are asynchronous.

Stream을 Future에 변환 해, Stream에 의해 발행되는 최대의 항목을 완성합니다.

이것은 목록에서 최대 값을 찾는 것과 비슷하지만 값은 비동기입니다.

Example

```dart
final max = await new Observable.fromIterable([1, 2, 3]).max();

print(max); // prints 3
```

Example with custom `Comparator`

```dart
final observable = new Observable.fromIterable(["short", "looooooong"]);
final max = await observable.max((a, b) => a.length - b.length);

print(max); // prints "looooooong"
```

#### mergeWith()

Combines the items emitted by multiple streams into a single stream of items. The items are emitted in the order they are emitted by their sources.

여러 스트림에서 방출 된 항목을 단일 항목 스트림으로 결합합니다. 아이템은 소스에 의해 방출 된 순서대로 방출됩니다.

Example

```dart
new Observable.timer(1, new Duration(seconds: 10))
    .mergeWith([new Observable.just(2)])
    .listen(print); // prints 2, 1
```

#### min()

Converts a Stream into a Future that completes with the smallest item emitted by the Stream.

This is similar to finding the min value in a list, but the values are asynchronous!

Stream을 Future에 변환 해, Stream에 의해 발행되는 최소의 항목으로 완성합니다.

이것은 목록에서 최소값을 찾는 것과 비슷하지만 값은 비동기입니다!

Example

```dart
final min = await new Observable.fromIterable([1, 2, 3]).min();

print(min); // prints 1
```

Example with custom `Comparator`

```dart
final observable = new Observable.fromIterable(["short", "looooooong"]);
final min = await observable.min((a, b) => a.length - b.length);

print(min); // prints "short"
```

#### ofType<S>()

Filters a sequence so that only events of a given type pass

In order to capture the Type correctly, it needs to be wrapped in a [TypeToken](https://pub.dev/documentation/rxdart/latest/rx/TypeToken-class.html) as the generic parameter.

Given the way Dart generics work, one cannot simply use the `is T` / `as T` checks and castings with this method alone. Therefore, the [TypeToken](https://pub.dev/documentation/rxdart/latest/rx/TypeToken-class.html) class was introduced to capture the type of class you'd like `ofType` to filter down to.

주어진 유형의 이벤트 만 통과하도록 시퀀스를 필터링합니다.

Type을 올바르게 캡처하려면 generic 매개 변수로 [TypeToken] (https://pub.dev/documentation/rxdart/latest/rx/TypeToken-class.html)에 래핑해야합니다.

Dart generics가 작동하는 방식을 감안할 때이 방법만으로`T`` /``T`` 검사와 주조를 간단히 사용할 수는 없습니다. 그러므로`TypeToken '(https://pub.dev/documentation/rxdart/latest/rx/TypeToken-class.html) 클래스가`ofType`을 필터링 할 클래스 유형을 포착하기 위해 도입되었습니다.

Examples

```dart
new Observable.fromIterable([1, "hi"])
  .ofType(new TypeToken<String>())
  .listen(print); // prints "hi"
```

As a shortcut, you can use some pre-defined constants to write the above in the following way:

```dart
new Observable.fromIterable([1, "hi"])
  .ofType(kString)
  .listen(print); // prints "hi"
```

If you'd like to create your own shortcuts like the example above, simply create a constant:

```dart
const TypeToken<Map<Int, String>> kMapIntString =
  const TypeToken<Map<Int, String>>();
```

#### onErrorResume()

Intercepts error events and switches to a recovery stream created by the provided `recoveryFn`.

The onErrorResume operator intercepts an onError notification from the source Observable. Instead of passing the error through to any listeners, it replaces it with another Stream of items created by the `recoveryFn`.

The `recoveryFn` receives the emitted error and returns a Stream. You can perform logic in the `recoveryFn` to return different Streams based on the type of error that was emitted.

If you do not need to perform logic based on the type of error that was emitted, please consider using [onErrorResumeNext](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorResumeNext.html) or [onErrorReturn](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturn.html).

오류 이벤트를 차단하고 제공된 'recoveryFn'에 의해 생성 된 복구 스트림으로 전환합니다.

onErrorResume 연산자는 Observable 소스에서 onError 알림을 가로 챕니다. 오류를 모든 청취자에게 전달하는 대신 'recoveryFn'에 의해 생성 된 다른 항목 스트림으로 대체합니다.

`recoveryFn`는 발생한 에러를 받아서 Stream을 반환합니다. `recoveryFn`에서 로직을 수행하여 생성 된 에러 유형에 따라 다른 스트림을 반환 할 수 있습니다.

생성 된 오류 유형을 기반으로 논리를 수행 할 필요가없는 경우 [onErrorResumeNext] (https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorResumeNext.html) 또는 [onErrorReturn ] (https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturn.html).

Example

```dart
new Observable<int>.error(new Exception())
  .onErrorResume((dynamic e) =>
      new Observable.just(e is StateError ? 1 : 0)
  .listen(print); // prints 0
```

#### onErrorResumeNext()

Intercepts error events and switches to the given recovery stream in that case

The onErrorResumeNext operator intercepts an onError notification from the source Observable. Instead of passing the error through to any listeners, it replaces it with another Stream of items.

If you need to perform logic based on the type of error that was emitted, please consider using [onErrorResume](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorResume.html).

이 경우 오류 이벤트를 차단하고 해당 복구 스트림으로 전환합니다.

onErrorResumeNext 연산자는 Observable 소스에서 onError 알림을 가로 챕니다. 오류를 모든 수신기에 전달하는 대신 다른 항목 스트림으로 바꿉니다.

생성 된 오류 유형을 기반으로 논리를 수행해야하는 경우 [onErrorResume] (https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorResume.html)을 사용해보십시오.

Example

```dart
new Observable.error(new Exception())
  .onErrorResumeNext(new Observable.fromIterable([1, 2, 3]))
  .listen(print); // prints 1, 2, 3
```

#### onErrorReturn()

instructs an Observable to emit a particular item when it encounters an error, and then terminate normally

The onErrorReturn operator intercepts an onError notification from the source Observable. Instead of passing it through to any observers, it replaces it with a given item, and then terminates normally.

If you need to perform logic based on the type of error that was emitted, please consider using [onErrorReturnWith](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturnWith.html).

Observable에 오류가 발생하면 특정 항목을 내보내고 정상적으로 종료되도록 지시합니다.

onErrorReturn 연산자는 Observable 소스에서 onError 알림을 가로 챕니다. 관찰자에게 전달하는 대신 지정된 항목으로 바꾼 다음 정상적으로 종료됩니다.

생성 된 오류 유형을 기반으로 논리를 수행해야하는 경우 [onErrorReturnWith] (https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturnWith.html)를 사용하는 것이 좋습니다.

Example

```dart
new Observable.error(new Exception())
  .onErrorReturn(1)
  .listen(print); // prints 1
```

#### onErrorReturnWith()

instructs an Observable to emit a particular item created by the `returnFn` when it encounters an error, and then terminate normally.

The onErrorReturnWith operator intercepts an onError notification from the source Observable. Instead of passing it through to any observers, it replaces it with a given item, and then terminates normally.

The `returnFn` receives the emitted error and returns a Stream. You can perform logic in the `returnFn` to return different Streams based on the type of error that was emitted.

If you do not need to perform logic based on the type of error that was emitted, please consider using [onErrorReturn](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturn.html).

Observable은 오류가 발생하면`returnFn`에 의해 생성 된 특정 항목을 내 보낸 다음 정상적으로 종료하도록 지시합니다.

onErrorReturnWith 연산자는 Observable 소스에서 onError 알림을 가로 챕니다. 관찰자에게 전달하는 대신 지정된 항목으로 바꾼 다음 정상적으로 종료됩니다.

`returnFn`은 발생한 에러를 받아서 Stream을 반환합니다. `returnFn`에서 로직을 수행하여 방출 된 에러 유형에 따라 다른 스트림을 반환 할 수 있습니다.

생성 된 오류 유형에 따라 논리를 수행 할 필요가없는 경우 [onErrorReturn] (https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturn.html)을 사용하십시오.

Example

```dart
new Observable.error(new Exception())
  .onErrorReturnWith((e) => e is Exception ? 1 : 0)
  .listen(print); // prints 1
```

#### pairwise()

Emits the n-th and n-1th events as a pair.

n 번째 및 n-1 번째 이벤트를 한 쌍으로 내 보냅니다.

Example

```dart
Observable.range(1, 4)
  .pairwise()
  .listen(print); // prints [1, 2], [2, 3], [3, 4]
```

#### pipe()

Pipes the events of this stream into `streamConsumer`.

All events of this stream are added to `streamConsumer` using `StreamConsumer.addStream`. The `streamConsumer` is closed when this stream has been successfully added to it - when the future returned by `addStream` completes without an error.

Returns a future which completes when this stream has been consumed and the consumer has been closed.

The returned future completes with the same result as the future returned by `StreamConsumer.close`. If the call to `StreamConsumer.addStream` fails in some way, this method fails in the same way.

이 스트림의 이벤트를`streamConsumer`에 파이프합니다.

이 스트림의 모든 이벤트는`StreamConsumer.addStream`을 사용하여`streamConsumer`에 추가됩니다. `streamConsumer`는이 스트림이 성공적으로 추가되었을 때 닫힙니다.`addStream`에 의해 리턴 된 미래가 오류없이 완료 될 때입니다.

이 스트림이 소비되어 컨슈머 (consumer)가 클로즈되었을 때에 완료하는 장래를 돌려줍니다.

반환 된 미래는`StreamConsumer.close`에 의해 반환되는 미래와 같은 결과로 완료됩니다. `StreamConsumer.addStream`의 호출이 어떤 식 으로든 실패하면, 같은 방법으로이 메소드가 실패합니다.

#### publish()

Convert the current Observable into a [ConnectableObservable](https://pub.dev/documentation/rxdart/latest/rx/ConnectableObservable-class.html) that can be listened to multiple times. It will not begin emitting items from the original Observable until the `connect` method is invoked.

This is useful for converting a single-subscription stream into a broadcast Stream.

현재 Observable을 여러 번 청취 할 수있는 [ConnectableObservable] (https://pub.dev/documentation/rxdart/latest/rx/ConnectableObservable-class.html)로 변환하십시오. `connect` 메소드가 호출 될 때까지 원래 Observable에서 항목을 방출하기 시작하지 않습니다.

이 기능은 단일 구독 스트림을 브로드 캐스트 스트림으로 변환 할 때 유용합니다.

Example

```dart
final source = Observable.fromIterable([1, 2, 3]);
final connectable = source.publish();

// Does not print anything at first
connectable.listen(print);

// Start listening to the source Observable. Will cause the previous
// line to start printing 1, 2, 3
final subscription = connectable.connect();

// Stop emitting items from the source stream and close the underlying
// Subject
subscription.cancel();
```

#### publishReplay()

Convert the current Observable into a [ReplayConnectableObservable](https://pub.dev/documentation/rxdart/latest/rx/ReplayConnectableObservable-class.html) that can be listened to multiple times. It will not begin emitting items from the original Observable until the `connect` method is invoked.

This is useful for converting a single-subscription stream into a broadcast Stream that replays a given number of items to any new listener. It also provides access to the emitted values synchronously.

현재 Observable을 여러 번 청취 할 수있는 [ReplayConnectableObservable] (https://pub.dev/documentation/rxdart/latest/rx/ReplayConnectableObservable-class.html)로 변환하십시오. `connect` 메소드가 호출 될 때까지 원래 Observable에서 항목을 방출하기 시작하지 않습니다.

이 기능은 단일 구독 스트림을 새로운 수신기에 지정된 수의 항목을 재생하는 방송 스트림으로 변환하는 데 유용합니다. 또한 방출 된 값에 동 기적으로 액세스 할 수 있습니다.

Example

```dart
final source = Observable.fromIterable([1, 2, 3]);
final connectable = source.publishReplay();

// Does not print anything at first
connectable.listen(print);

// Start listening to the source Observable. Will cause the previous
// line to start printing 1, 2, 3
final subscription = connectable.connect();

// Late subscribers will receive the emitted value, up to a specified
// maxSize
connectable.listen(print); // Prints 1, 2, 3

// Can access a list of the emitted values synchronously. Prints [1, 2, 3]
print(connectable.values);

// Stop emitting items from the source stream and close the underlying
// ReplaySubject
subscription.cancel();
```

#### publishValue()

Convert the current Observable into a [ValueConnectableObservable](https://pub.dev/documentation/rxdart/latest/rx/ValueConnectableObservable-class.html) that can be listened to multiple times. It will not begin emitting items from the original Observable until the `connect` method is invoked.

This is useful for converting a single-subscription stream into a broadcast Stream that replays the latest emitted value to any new listener. It also provides access to the latest value synchronously.

현재 Observable을 여러 번 수신 할 수있는 [ValueConnectableObservable] (https://pub.dev/documentation/rxdart/latest/rx/ValueConnectableObservable-class.html)로 변환하십시오. `connect` 메소드가 호출 될 때까지 원래 Observable에서 항목을 방출하기 시작하지 않습니다.

이 기능은 단일 구독 스트림을 최신 수신 값을 모든 새 수신기에 재생하는 방송 스트림으로 변환 할 때 유용합니다. 또한 최신 값에 동 기적으로 액세스 할 수 있습니다.

Example

```dart
final source = Observable.fromIterable([1, 2, 3]);
final connectable = source.publishValue();

// Does not print anything at first
connectable.listen(print);

// Start listening to the source Observable. Will cause the previous
// line to start printing 1, 2, 3
final subscription = connectable.connect();

// Late subscribers will receive the last emitted value
connectable.listen(print); // Prints 3

// Can access the latest emitted value synchronously. Prints 3
print(connectable.value);

// Stop emitting items from the source stream and close the underlying
// BehaviorSubject
subscription.cancel();
```

#### publishValueSeeded()

Convert the current Observable into a [ValueConnectableObservable](https://pub.dev/documentation/rxdart/latest/rx/ValueConnectableObservable-class.html) that can be listened to multiple times, providing an initial seeded value. It will not begin emitting items from the original Observable until the `connect` method is invoked.

This is useful for converting a single-subscription stream into a broadcast Stream that replays the latest emitted value to any new listener. It also provides access to the latest value synchronously.

현재 Observable을 여러 번 수신 할 수있는 [ValueConnectableObservable] (https://pub.dev/documentation/rxdart/latest/rx/ValueConnectableObservable-class.html)로 변환하여 초기 시드 값을 제공합니다. `connect` 메소드가 호출 될 때까지 원래 Observable에서 항목을 방출하기 시작하지 않습니다.

이 기능은 단일 구독 스트림을 최신 수신 값을 모든 새 수신기에 재생하는 방송 스트림으로 변환 할 때 유용합니다. 또한 최신 값에 동 기적으로 액세스 할 수 있습니다.

Example

```dart
final source = Observable.fromIterable([1, 2, 3]);
final connectable = source.publishValueSeeded(0);

// Does not print anything at first
connectable.listen(print);

// Start listening to the source Observable. Will cause the previous
// line to start printing 0, 1, 2, 3
final subscription = connectable.connect();

// Late subscribers will receive the last emitted value
connectable.listen(print); // Prints 3

// Can access the latest emitted value synchronously. Prints 3
print(connectable.value);

// Stop emitting items from the source stream and close the underlying
// BehaviorSubject
subscription.cancel();
```

#### reduce()

Combines a sequence of values by repeatedly applying `combine`.

Similar to [Iterable.reduce](https://pub.dev/documentation/rxdart/latest/rx/Observable/reduce.html), this function maintains a value, starting with the first element of this stream and updated for each further element of this stream. For each element after the first, the value is updated to the result of calling `combine` with the previous value and the element.

When this stream is done, the returned future is completed with the value at that time.

If this stream is empty, the returned future is completed with an error. If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

`combine '를 반복적으로 적용하여 값의 시퀀스를 결합합니다.

[Iterable.reduce] (https://pub.dev/documentation/rxdart/latest/rx/Observable/reduce.html)와 마찬가지로이 함수는이 스트림의 첫 번째 요소부터 시작하여 추가로 업데이트되는 값을 유지합니다. 이 스트림의 요소 첫 번째 요소 이후의 각 요소에 대해 값은 이전 값 및 요소와`combine '를 호출 한 결과로 업데이트됩니다.

이 스트림이 완료되면 반환 된 미래가 그 시점의 값으로 완료됩니다.

이 스트림이 비어 있으면 리턴 된 미래가 오류와 함께 완료됩니다. 이 스트림에서 오류가 발생하거나`combine '호출이 throw되면 해당 오류로 반환 된 미래가 완료되고 처리가 중지됩니다.

#### sample()

Emits the most recently emitted item (if any) emitted by the source `Stream` since the previous emission from the `sampleStream`.

`sampleStream`로부터의 이전의 방출 이후에, 소스`Stream`에 의해 방출 된 가장 최근에 방출 된 항목 (있는 경우)을 방출합니다.

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .sample(new TimerStream(1, const Duration(seconds: 1)))
  .listen(print); // prints 3
```

#### sampleTime()

Emits the most recently emitted item (if any) emitted by the source `Stream` since the previous emission within the recurring time span, defined by `duration`

'duration'에 의해 정의 된 반복 시간 범위 내에서 이전 방출 이후 소스 '스트림'에 의해 방출 된 가장 최근에 방출 된 항목 (있는 경우)을 방출합니다

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .sampleTime(const Duration(seconds: 1))
  .listen(print); // prints 3
```

#### scan<S>()

Applies an accumulator function over an observable sequence and returns each intermediate result. The optional seed value is used as the initial accumulator value.

관측 가능한 시퀀스에 누적 기 함수를 적용하고 각 중간 결과를 반환합니다. 선택적 시드 값은 초기 누적 기 값으로 사용됩니다.

Example

```dart
new Observable.fromIterable([1, 2, 3])
   .scan((acc, curr, i) => acc + curr, 0)
   .listen(print); // prints 1, 3, 6
```

#### share()

Convert the current Observable into a new Observable that can be listened to multiple times. It will automatically begin emitting items when first listened to, and shut down when no listeners remain.

This is useful for converting a single-subscription stream into a broadcast Stream.

현재 Observable을 여러 번 청취 할 수있는 새로운 Observable로 변환합니다. 처음 청취 할 때 항목을 자동으로 시작하고 청취자가 없을 때 종료합니다.

이 기능은 단일 구독 스트림을 브로드 캐스트 스트림으로 변환 할 때 유용합니다.

Example

```dart
// Convert a single-subscription fromIterable stream into a broadcast
// stream
final observable = Observable.fromIterable([1, 2, 3]).share();

// Start listening to the source Observable. Will start printing 1, 2, 3
final subscription = observable.listen(print);

// Stop emitting items from the source stream and close the underlying
// PublishSubject
subscription.cancel();
```

#### shareReplay()

Convert the current Observable into a new [ReplayObservable](https://pub.dev/documentation/rxdart/latest/rx/ReplayObservable-class.html) that can be listened to multiple times. It will automatically begin emitting items when first listened to, and shut down when no listeners remain.

This is useful for converting a single-subscription stream into a broadcast Stream. It's also useful for gaining access to the l

It will replay the emitted values to any new listener, up to a given `maxSize`.

현재 Observable을 여러 번 청취 할 수있는 새로운 [ReplayObservable] (https://pub.dev/documentation/rxdart/latest/rx/ReplayObservable-class.html)로 변환하십시오. 처음 청취 할 때 항목을 자동으로 시작하고 청취자가 없을 때 종료합니다.

이 기능은 단일 구독 스트림을 브로드 캐스트 스트림으로 변환 할 때 유용합니다. l에 대한 액세스 권한을 얻는데도 유용합니다.

지정된 maxSize까지 새로운 값으로 재생 된 값을 재생합니다.

Example

```dart
// Convert a single-subscription fromIterable stream into a broadcast
// stream that will emit the latest value to any new listeners
final observable = Observable.fromIterable([1, 2, 3]).shareReplay();

// Start listening to the source Observable. Will start printing 1, 2, 3
final subscription = observable.listen(print);

// Synchronously print the emitted values up to a given maxSize
// Prints [1, 2, 3]
print(observable.values);

// Subscribe again later. This will print 1, 2, 3 because it receives the
// last emitted value.
final subscription2 = observable.listen(print);

// Stop emitting items from the source stream and close the underlying
// ReplaySubject by cancelling all subscriptions.
subscription.cancel();
subscription2.cancel();
```

#### shareValue()

Convert the current Observable into a new [ValueObservable](https://pub.dev/documentation/rxdart/latest/rx/ValueObservable-class.html) that can be listened to multiple times. It will automatically begin emitting items when first listened to, and shut down when no listeners remain.

This is useful for converting a single-subscription stream into a broadcast Stream. It's also useful for providing sync access to the latest emitted value.

It will replay the latest emitted value to any new listener.

현재 Observable을 여러 번 청취 할 수있는 새로운 [ValueObservable] (https://pub.dev/documentation/rxdart/latest/rx/ValueObservable-class.html)로 변환하십시오. 처음 청취 할 때 항목을 자동으로 시작하고 청취자가 없을 때 종료합니다.

이 기능은 단일 구독 스트림을 브로드 캐스트 스트림으로 변환 할 때 유용합니다. 또한 최신 방출 값에 대한 동기화 액세스를 제공하는 데 유용합니다.

새로운 청취자에게 최신 방출 값을 재생합니다.

Example

```dart
// Convert a single-subscription fromIterable stream into a broadcast
// stream that will emit the latest value to any new listeners
final observable = Observable.fromIterable([1, 2, 3]).shareValue();

// Start listening to the source Observable. Will start printing 1, 2, 3
final subscription = observable.listen(print);

// Synchronously print the latest value
print(observable.value);

// Subscribe again later. This will print 3 because it receives the last
// emitted value.
final subscription2 = observable.listen(print);

// Stop emitting items from the source stream and close the underlying
// BehaviorSubject by cancelling all subscriptions.
subscription.cancel();
subscription2.cancel();
```

#### shareValueSeeded()

Convert the current Observable into a new [ValueObservable](https://pub.dev/documentation/rxdart/latest/rx/ValueObservable-class.html) that can be listened to multiple times, providing an initial value. It will automatically begin emitting items when first listened to, and shut down when no listeners remain.

This is useful for converting a single-subscription stream into a broadcast Stream. It's also useful for providing sync access to the latest emitted value.

It will replay the latest emitted value to any new listener.

현재 Observable을 여러 번 청취 할 수있는 새로운 [ValueObservable] (https://pub.dev/documentation/rxdart/latest/rx/ValueObservable-class.html)로 변환하여 초기 값을 제공합니다. 처음 청취 할 때 항목을 자동으로 시작하고 청취자가 없을 때 종료합니다.

이 기능은 단일 구독 스트림을 브로드 캐스트 스트림으로 변환 할 때 유용합니다. 또한 최신 방출 값에 대한 동기화 액세스를 제공하는 데 유용합니다.

새로운 청취자에게 최신 방출 값을 재생합니다.

Example

```dart
// Convert a single-subscription fromIterable stream into a broadcast
// stream that will emit the latest value to any new listeners
final observable = Observable.fromIterable([1, 2, 3]).shareValueSeeded(0);

// Start listening to the source Observable. Will start printing 0, 1, 2, 3
final subscription = observable.listen(print);

// Synchronously print the latest value
print(observable.value);

// Subscribe again later. This will print 3 because it receives the last
// emitted value.
final subscription2 = observable.listen(print);

// Stop emitting items from the source stream and close the underlying
// BehaviorSubject by cancelling all subscriptions.
subscription.cancel();
subscription2.cancel();
```

#### singleWhere()

Finds the single element in this stream matching `test`.

Like [lastWhere](https://pub.dev/documentation/rxdart/latest/rx/Observable/lastWhere.html), except that it is an error if more than one matching element occurs in this stream.

이 스트림 내의`test`와 일치하는 단일 요소를 찾습니다.

[lastWhere] (https://pub.dev/documentation/rxdart/latest/rx/Observable/lastWhere.html)와 유사하지만이 스트림에서 일치하는 요소가 두 개 이상 발생하면 오류가 발생합니다.

#### skip()

Skips the first count data events from this stream.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only counted from the time the returned stream is listened to.

이 스트림으로부터 최초의 카운트 데이터 이벤트를 스킵합니다.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림의 경우 이벤트는 반환 된 스트림을 수신 한 시간부터 계산됩니다.

#### skipUntil<S>

Starts emitting items only after the given stream emits an item.

지정된 스트림이 항목을 방출 한 후에 만 항목을 시작합니다.

Example

```dart
new Observable.merge([
    new Observable.just(1),
    new Observable.timer(2, new Duration(minutes: 2))
  ])
  .skipUntil(new Observable.timer(true, new Duration(minutes: 1)))
  .listen(print); // prints 2;
```

#### skipWhile()

Skip data events from this stream while they are matched by test.

Error and done events are provided by the returned stream unmodified.

Starting with the first data event where test returns false for the event data, the returned stream will have the same events as this stream.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only tested from the time the returned stream is listened to.

테스트로 일치하는 동안이 스트림에서 데이터 이벤트를 건너 뜁니다.

오류 및 완료 이벤트는 수정되지 않은 리턴 된 스트림에 의해 제공됩니다.

테스트가 이벤트 데이터에 대해 false를 반환하는 첫 번째 데이터 이벤트부터 시작하여 반환 된 스트림은이 스트림과 동일한 이벤트를 갖게됩니다.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림의 경우 이벤트는 반환 된 스트림을 수신 한 시점부터 만 테스트됩니다.

#### startWith()

Prepends a value to the source Observable.

Observable 소스에 값을 앞에 추가합니다.

Example

```dart
new Observable.just(2).startWith(1).listen(print); // prints 1, 2
```

#### startWithMany()

Prepends a sequence of values to the source Observable.

Observable 소스에 일련의 값을 앞에 붙입니다.

Example

```dart
new Observable.just(3).startWithMany([1, 2])
  .listen(print); // prints 1, 2, 3
```

#### switchIfEmpty()

When the original observable emits no items, this operator subscribes to the given fallback stream and emits items from that observable instead.

This can be particularly useful when consuming data from multiple sources. For example, when using the Repository Pattern. Assuming you have some data you need to load, you might want to start with the fastest access point and keep falling back to the slowest point. For example, first query an in-memory database, then a database on the file system, then a network call if the data isn't on the local machine.

This can be achieved quite simply with switchIfEmpty!

원래 관찰 가능 항목이 항목을 방출하지 않으면이 연산자는 주어진 대체 스트림을 구독하고 대신 해당 관찰 항목에서 항목을 방출합니다.

이는 여러 소스에서 데이터를 사용할 때 특히 유용 할 수 있습니다. 예를 들어, 저장소 패턴을 사용할 때. 로드해야하는 데이터가 있다고 가정하면 가장 빠른 액세스 포인트로 시작하여 가장 느린 지점으로 계속 떨어지는 것이 좋습니다. 예를 들어 먼저 메모리 내 데이터베이스를 쿼리 한 다음 파일 시스템의 데이터베이스를 쿼리 한 다음 데이터가 로컬 컴퓨터에없는 경우 네트워크 호출을 쿼리합니다.

switchIfEmpty를 사용하면 매우 간단하게이 작업을 수행 할 수 있습니다!

Example

```dart
// Let's pretend we have some Data sources that complete without emitting
// any items if they don't contain the data we're looking for
Observable<Data> memory;
Observable<Data> disk;
Observable<Data> network;

// Start with memory, fallback to disk, then fallback to network.
// Simple as that!
Observable<Data> getThatData =
    memory.switchIfEmpty(disk).switchIfEmpty(network);
```

#### switchMap<S>()

Converts each emitted item into a new Stream using the given mapper function. The newly created Stream will be be listened to and begin emitting items, and any previously created Stream will stop emitting.

The switchMap operator is similar to the flatMap and concatMap methods, but it only emits items from the most recently created Stream.

This can be useful when you only want the very latest state from asynchronous APIs, for example.

지정된 매퍼 함수를 사용하여 방출 된 각 항목을 새 스트림으로 변환합니다. 새로 생성 된 스트림은 청취되어 항목을 내보내고 이전에 생성 된 스트림은 방출을 중지합니다.

switchMap 연산자는 flatMap 및 concatMap 메소드와 비슷하지만 가장 최근에 작성된 스트림의 항목 만 방출합니다.

예를 들어, 비동기 API에서만 최신 상태를 원할 때 유용합니다.

Example

```dart
Observable.range(4, 1)
  .switchMap((i) =>
    new Observable.timer(i, new Duration(minutes: i))
  .listen(print); // prints 1
```

#### take()

Provides at most the first `n` values of this stream. Forwards the first n data events of this stream, and all error events, to the returned stream, and ends with a done event.

If this stream produces fewer than count values before it's done, so will the returned stream.

Stops listening to the stream after the first n elements have been received.

Internally the method cancels its subscription after these elements. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only counted from the time the returned stream is listened to

이 스트림의 최초의`n '값을 제공합니다. 이 스트림의 처음 n 개의 데이터 이벤트와 모든 오류 이벤트를 반환 된 스트림으로 전달하고 done 이벤트로 끝냅니다.

이 스트림이 완료되기 전에 카운트 값보다 적은 값을 생성하면 반환 된 스트림도 반환됩니다.

최초의 n 요소가 수신 된 후 스트림을 청취하지 않습니다.

내부적으로 메소드는 이러한 요소 다음에 구독을 취소합니다. 이것은 단일 구독 (비 브로드 캐스트) 스트림이 닫히고이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림의 경우 이벤트는 반환 된 스트림을 수신 한 시간부터 계산됩니다

#### takeUntil<S>()

Returns the values from the source observable sequence until the other observable sequence produces a value.

다른 관찰 가능 시퀀스가 값을 생성 할 때까지 소스 관찰 가능 시퀀스의 값을 반환합니다.

Example

```dart
new Observable.merge([
    new Observable.just(1),
    new Observable.timer(2, new Duration(minutes: 1))
  ])
  .takeUntil(new Observable.timer(3, new Duration(seconds: 10)))
  .listen(print); // prints 1
```

#### takeWhile()

Forwards data events while test is successful.

The returned stream provides the same events as this stream as long as test returns true for the event data. The stream is done when either this stream is done, or when this stream first provides a value that test doesn't accept.

Stops listening to the stream after the accepted elements.

Internally the method cancels its subscription after these elements. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only tested from the time the returned stream is listened to.

테스트가 성공적으로 진행되는 동안 데이터 이벤트를 전달합니다.

반환 된 스트림은 테스트가 이벤트 데이터에 대해 true를 반환하는 한이 스트림과 동일한 이벤트를 제공합니다. 스트림은이 스트림이 완료되거나이 스트림이 처음에 test가 승인하지 않는 값을 제공 할 때 완료됩니다.

받아 들여진 요소의 다음에 스트림의 청취를 정지합니다.

내부적으로 메소드는 이러한 요소 다음에 구독을 취소합니다. 이것은 단일 구독 (비 브로드 캐스트) 스트림이 닫히고이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

반환 된 스트림은이 스트림이있는 경우 브로드 캐스트 스트림입니다. 브로드 캐스트 스트림의 경우 이벤트는 반환 된 스트림을 수신 한 시점부터 만 테스트됩니다.

#### throttle()

Emits only the first item emitted by the source `Stream` while `window` is open.

if `trailing` is true, then the last item is emitted instead

You can use the value of the last throttled event to determine the length of the next `window`.

`window`가 열린 상태에서`Stream` 소스가 방출 한 첫 번째 항목 만 방출합니다.

`trailing`이 true이면 마지막 항목이 대신 방출됩니다.

마지막 throttled 이벤트의 값을 사용하여 다음 '창의'길이를 결정할 수 있습니다.

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .throttle((_) => TimerStream(true, const Duration(seconds: 1)))
```

#### throttleTime()

Emits only the first item emitted by the source `Stream` within a time span of `duration`.

if `trailing` is true, then the last item is emitted instead

`duration` 시간대 내에`Stream` 소스가 방출 한 첫 번째 항목 만 방출합니다.

`trailing`이 true이면 마지막 항목이 대신 방출됩니다.

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .throttleTime(const Duration(seconds: 1))
```

#### timeInterval()

Records the time interval between consecutive values in an observable sequence.

관찰 가능한 순서에서 연속 값 사이의 시간 간격을 기록합니다.

Example

```dart
new Observable.just(1)
  .interval(new Duration(seconds: 1))
  .timeInterval()
  .listen(print); // prints TimeInterval{interval: 0:00:01, value: 1}
```

#### timeout()

The Timeout operator allows you to abort an Observable with an onError termination if that Observable fails to emit any items during a specified duration.  You may optionally provide a callback function to execute on timeout.

Timeout 연산자를 사용하면 Observable이 지정된 기간 동안 항목을 방출하지 못하는 경우 onError 종료로 Observable을 중단 할 수 있습니다. 선택적으로 타임 아웃시 실행할 콜백 함수를 제공 할 수 있습니다.

#### timestamp()

Wraps each item emitted by the source Observable in a [Timestamped](https://pub.dev/documentation/rxdart/latest/rx/Timestamped-class.html) object that includes the emitted item and the time when the item was emitted.

방출 된 항목과 방출 된 시간을 포함하는 [Timestamped] (https://pub.dev/documentation/rxdart/latest/rx/Timestamped-class.html) 개체에서 관찰 할 수있는 소스에서 방출 한 각 항목을 래핑합니다. .

Example

```dart
new Observable.just(1)
   .timestamp()
   .listen((i) => print(i)); // prints 'TimeStamp{timestamp: XXX, value: 1}';
```

#### toList()

Collects all elements of this stream in a `List`.

Creates a `List<T>` and adds all elements of this stream to the list in the order they arrive. When this stream ends, the returned future is completed with that list.

If this stream emits an error, the returned future is completed with that error, and processing stops.

#### toSet()

Collects the data of this stream in a `Set`.

Creates a `Set<T>` and adds all elements of this stream to the set. in the order they arrive. When this stream ends, the returned future is completed with that set.

The returned set is the same type as returned by `new Set<T>()`. If another type of set is needed, either use [forEach](https://pub.dev/documentation/rxdart/latest/rx/Observable/forEach.html) to add each element to the set, or use `toList().then((list) => new SomeOtherSet.from(list))` to create the set.

If this stream emits an error, the returned future is completed with that error, and processing stops.

#### transform<S>()

Applies  `streamTransformer` to this stream.

Returns the transformed stream, that is, the result of `streamTransformer.bind(this)`. This method simply allows writing the call to `streamTransformer.bind` in a chained fashion, like

이 스트림에`streamTransformer`를 적용합니다.

변환 된 스트림, 즉`streamTransformer.bind (this)`의 결과를 리턴합니다. 이 메소드는 단순히 'streamTransformer.bind`에 대한 호출을 체인 방식으로 작성하는 것을 허용합니다.

```dart
stream.map(mapping).transform(transformation).toList()
```

which can be more convenient than calling `bind` directly.

The `streamTransformer` can return any stream. Whether the returned stream is a broadcast stream or not, and which elements it will contain, is entirely up to the transformation.

This method should always be used for transformations which treat the entire stream as representing a single value which has perhaps been split into several parts for transport, like a file being read from disk or being fetched over a network. The transformation will then produce a new stream which transforms the stream's value incrementally (perhaps using `Converter.startChunkedConversion`). The resulting stream may again be chunks of the result, but does not have to correspond to specific events from the source string.

`bind`를 직접 호출하는 것보다 더 편리 할 수 있습니다.

`streamTransformer`는 모든 스트림을 반환 할 수 있습니다. 돌려 주어지는 스트림이 브로드 캐스트 스트림인가 어떤가, 어느 요소가 포함되는지는 전적으로 변환에 달려 있습니다.

이 메소드는 파일 전체를 디스크에서 읽어들이거나 네트워크에서 가져 오는 것과 같이 전송을 위해 여러 부분으로 분할 된 단일 값을 나타내는 것으로 전체 스트림을 처리하는 변환에 항상 사용해야합니다. 그런 다음 변환은 스트림의 값을 점진적으로 변환하는 새로운 스트림을 생성합니다 (아마도`Converter.startChunkedConversion`을 사용하여). 결과 스트림은 결과의 청크 일 수 있지만 소스 문자열의 특정 이벤트와 일치 할 필요는 없습니다.

#### where()

Filters the elements of an observable sequence based on the test.

테스트를 기반으로 관찰 가능한 시퀀스의 요소를 필터링합니다.

#### window()

Creates an Observable where each item is a `Stream` containing the items from the source sequence.

This `List` is emitted every time `window` emits an event.

각 항목이 소스 시퀀스의 항목을 포함하는 'Stream'인 Observable을 만듭니다.

이`List`는`window`가 이벤트를 낼 때마다 발생합니다.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (i) => i)
  .window(new Stream.periodic(const Duration(milliseconds: 160), (i) => i))
  .asyncMap((stream) => stream.toList())
  .listen(print); // prints [0, 1] [2, 3] [4, 5] ...
```

#### windowCount()

Buffers a number of values from the source Observable by `count` then emits the buffer as a `Stream` and clears it, and starts a new buffer each `startBufferEvery` values. If `startBufferEvery` is not provided, then new buffers are started immediately at the start of the source and when each buffer closes and is emitted.

Observable Observable에서`count` 값을 버퍼링 한 다음 버퍼를`Stream`으로 내보내고 지우고, 각각의 새로운 버퍼를 startBufferEvery 값으로 시작합니다. `startBufferEvery`가 제공되지 않으면, 소스의 시작과 각 버퍼가 닫히고 나올 때 즉시 새로운 버퍼가 시작됩니다.

Example

`count` is the maximum size of the buffer emitted

```dart
Observable.range(1, 4)
  .windowCount(2)
  .asyncMap((stream) => stream.toList())
  .listen(print); // prints [1, 2], [3, 4] done!
```

Example

if `startBufferEvery` is 2, then a new buffer will be started on every other value from the source. A new buffer is started at the beginning of the source by default.

```dart
Observable.range(1, 5)
  .bufferCount(3, 2)
  .listen(print); // prints [1, 2, 3], [3, 4, 5], [5] done!
```

#### windowTest()

Creates an Observable where each item is a `Stream` containing the items from the source sequence, batched whenever test passes.

Observable을 생성합니다. 각 항목은 소스 시퀀스의 항목을 포함하는 'Stream'이며, 테스트가 통과 할 때마다 배치됩니다.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (int i) => i)
  .windowTest((i) => i % 2 == 0)
  .asyncMap((stream) => stream.toList())
  .listen(print); // prints [0], [1, 2] [3, 4] [5, 6] ...
```

#### windowTime()

Creates an Observable where each item is a `Stream` containing the items from the source sequence, sampled on a time frame with `duration`.

Observable을 생성합니다. 각 항목은 소스 시퀀스의 항목을 포함하는`Stream`이며`duration`으로 시간 프레임에서 샘플링됩니다.

Example

```dart
new Observable.periodic(
    const Duration(milliseconds: 100),
    (int i) => i
)
.windowTime(const Duration(milliseconds: 220))
.doOnData((_) => print('next window'))
.flatMap((s) => s))
.listen(print); // prints next window 0, 1, next window 2, 3, ...
```

#### withLatestFrom<S, R> ()

Creates an Observable that emits when the source stream emits, combining the latest values from the two streams using the provided function.

If the latestFromStream has not emitted any values, this stream will not emit either.

제공된 함수를 사용하여 두 스트림의 최신 값을 결합하여 소스 스트림이 방출 될 때 나오는 Observable을 만듭니다.

latestFromStream이 값을 내보내지 않으면이 스트림도 방출되지 않습니다.

[Interactive marble diagram](http://rxmarbles.com/#withLatestFrom)

Example

```dart
new Observable.fromIterable([1, 2]).withLatestFrom(
  new Observable.fromIterable([2, 3]), (a, b) => a + b)
  .listen(print); // prints 4 (due to the async nature of streams)
```

#### zipWith<S, R>()

Returns an Observable that combines the current stream together with another stream using a given zipper function.

지정된 지퍼 함수를 사용하여 현재 스트림을 다른 스트림과 결합하는 Observable을 반환합니다.

Example

```dart
new Observable.just(1)
    .zipWith(new Observable.just(2), (one, two) => one + two)
    .listen(print); // prints 3
```

### 클래스 메소드

#### combineLatest<T, R>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item. This is helpful when you need to combine a dynamic number of Streams.

The Observable will not emit any lists of values until all of the source streams have emitted at least one value.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest(
    [new Observable.just("a"), new Observable.fromIterable("b", "c", "d")], 	(list) => list.join()
)
.listen(print); // prints "ab", "ac", "ad"
```

#### combineLatest2<A, B, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest2(
  new Observable.just(1),
  new Observable.fromIterable([0, 1, 2]),
  (a, b) => a + b)
.listen(print); //prints 1, 2, 3
```

#### combineLatest3<A, B, C, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest3(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.fromIterable(["c", "c"]),
  (a, b, c) => a + b + c)
.listen(print); //prints "abc", "abc"
```

#### combineLatest4<A, B, C, D, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest4(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.fromIterable(["d", "d"]),
  (a, b, c, d) => a + b + c + d)
.listen(print); //prints "abcd", "abcd"
```

#### combineLatest5<A, B, C, D, E, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest5(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.fromIterable(["e", "e"]),
  (a, b, c, d, e) => a + b + c + d + e)
.listen(print); //prints "abcde", "abcde"
```

#### combineLatest6<A, B, C, D, E, F, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest6(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.fromIterable(["f", "f"]),
  (a, b, c, d, e, f) => a + b + c + d + e + f)
.listen(print); //prints "abcdef", "abcdef"
```

#### combineLatest7<A, B, C, D, E, F, G, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest7(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.fromIterable(["g", "g"]),
  (a, b, c, d, e, f, g) => a + b + c + d + e + f + g)
.listen(print); //prints "abcdefg", "abcdefg"
```

#### combineLatest8<A, B, C, D, E, F, G, H, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest8(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.just("g"),
  new Observable.fromIterable(["h", "h"]),
  (a, b, c, d, e, f, g, h) => a + b + c + d + e + f + g + h)
.listen(print); //prints "abcdefgh", "abcdefgh"
```

#### combineLatest9<A, B, C, D, E, F, G, H, I, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function whenever any of the observable sequences emits an item.

The Observable will not emit until all streams have emitted at least one item.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatest9(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.just("g"),
  new Observable.just("h"),
  new Observable.fromIterable(["i", "i"]),
  (a, b, c, d, e, f, g, h, i) => a + b + c + d + e + f + g + h + i)
.listen(print); //prints "abcdefghi", "abcdefghi"
```

#### combineLatestList<T>()

Merges the given Streams into one Observable that emits a List of the values emitted by the source Stream. This is helpful when you need to combine a dynamic number of Streams.

The Observable will not emit any lists of values until all of the source streams have emitted at least one value.

[Interactive marble diagram](http://rxmarbles.com/#combineLatest)

Example

```dart
Observable.combineLatestList([
  Observable.just(1),
  Observable.fromIterable([0, 1, 2]),
])
.listen(print); // prints [1, 0], [1, 1], [1, 2]
```

#### forkJoin<T, R>()

Creates an [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) where all last events of existing stream(s) are piped  through a sink-transformation.

This operator is best used when you have a group of observables and only care about the final emitted value of each. One common use case for this is if you wish to issue multiple requests on page load (or some other event) and only want to take action when a response has been received for all.

In this way it is similar to how you might use `Future`.wait.

Be aware that if any of the inner observables supplied to forkJoin error you will lose the value of any other observables that would or have already completed if you do not catch the error correctly on the inner observable.

If you are only concerned with all inner observables completing successfully you can catch the error on the outside. It's also worth noting that if you have an observable that emits more than one item, and you are concerned with the previous emissions forkJoin is not the correct choice.

In these cases you may better off with an operator like combineLatest or zip.

Example

```dart
Observable.forkJoin(
    [
    	new Observable.just("a"),
    	new Observable.fromIterable("b", "c", "d")
    ],
	(list) => list.join(', ')
).listen(print); // prints "a, d"
```

#### forkJoin2<A, B, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin2(
  new Observable.just(1),
  new Observable.fromIterable([0, 1, 2]),
  (a, b) => a + b)
.listen(print); //prints 3
```

#### forkJoin3<A, B, C, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin3(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.fromIterable(["c", "d"]),
  (a, b, c) => a + b + c)
.listen(print); //prints "abd"
```

#### forkJoin4<A, B, C, D, T> ()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin4(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.fromIterable(["d", "e"]),
  (a, b, c, d) => a + b + c + d)
.listen(print); //prints "abce"
```

#### forkJoin5<A, B, C, D, E, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin5(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.fromIterable(["e", "f"]),
  (a, b, c, d, e) => a + b + c + d + e)
.listen(print); //prints "abcdf"
```

#### forkJoin6<A, B, C, D, E, F, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin6(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.fromIterable(["f", "g"]),
  (a, b, c, d, e, f) => a + b + c + d + e + f)
.listen(print); //prints "abcdeg"
```

#### forkJoin7<A, B, C, D, E, F, G, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin7(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.fromIterable(["g", "h"]),
  (a, b, c, d, e, f, g) => a + b + c + d + e + f + g)
.listen(print); //prints "abcdefh"
```

#### forkJoin8<A, B, C, D, E, F, G, H, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin8(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.just("g"),
  new Observable.fromIterable(["h", "i"]),
  (a, b, c, d, e, f, g, h) => a + b + c + d + e + f + g + h)
.listen(print); //prints "abcdefgi"
```

#### forkJoin9<A, B, C, D, E, F, G, H, I, T>()

Merges the given Streams into one Observable sequence by using the `combiner` function when all of the observable sequences emits their last item.

Example

```dart
Observable.forkJoin9(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.just("g"),
  new Observable.just("h"),
  new Observable.fromIterable(["i", "j"]),
  (a, b, c, d, e, f, g, h, i) => a + b + c + d + e + f + g + h + i)
.listen(print); //prints "abcdefghj"
```

#### forkJoinList<T>()

Merges the given Streams into one Observable that emits a List of the last values emitted by the source stream(s). This is helpful when you need to forkJoin a dynamic number of Streams.

Example

```dart
Observable.forkJoinList([
  Observable.just(1),
  Observable.fromIterable([0, 1, 2]),
])
.listen(print); // prints [1, 2]
```

#### range()

Returns an Observable that emits a sequence of Integers within a specified range.

Example

```dart
Observable.range(1, 3).listen((i) => print(i)); // Prints 1, 2, 3
Observable.range(3, 1).listen((i) => print(i)); // Prints 3, 2, 1
```

#### sequenceEqual<A, B>()

Determine whether two Observables emit the same sequence of items. You can provide an optional `equals` handler to determine equality.

[Interactive marble diagram](https://rxmarbles.com/#sequenceEqual)

Example

```dart
Observable.sequenceEqual([
  Stream.fromIterable([1, 2, 3, 4, 5]),
  Stream.fromIterable([1, 2, 3, 4, 5])
])
.listen(print); // prints true
```

#### zip<T, R>()

Merges the iterable streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip(
  [
    Observable.just("Hi "),
    Observable.fromIterable(["Friend", "Dropped"]),
  ],
  (values) => values.first + values.last
)
.listen(print); // prints "Hi Friend"
```

#### zip2<A, B, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip2(
  new Observable.just("Hi "),
  new Observable.fromIterable(["Friend", "Dropped"]),
  (a, b) => a + b)
.listen(print); // prints "Hi Friend"
```

#### zip3<A, B, C, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip3(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.fromIterable(["c", "dropped"]),
  (a, b, c) => a + b + c)
.listen(print); //prints "abc"
```

#### zip4<A, B, C, D, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip4(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.fromIterable(["d", "dropped"]),
  (a, b, c, d) => a + b + c + d)
.listen(print); //prints "abcd"
```

#### zip5<A, B, C, D, E, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip5(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.fromIterable(["e", "dropped"]),
  (a, b, c, d, e) => a + b + c + d + e)
.listen(print); //prints "abcde"
```

#### zip6<A, B, C, D, E, F, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip6(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.fromIterable(["f", "dropped"]),
  (a, b, c, d, e, f) => a + b + c + d + e + f)
.listen(print); //prints "abcdef"
```

#### zip7<A, B, C, D, E, F, G, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip7(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.fromIterable(["g", "dropped"]),
  (a, b, c, d, e, f, g) => a + b + c + d + e + f + g)
.listen(print); //prints "abcdefg"
```

#### zip8<A, B, C, D, E, F, G, H, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip8(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.just("g"),
  new Observable.fromIterable(["h", "dropped"]),
  (a, b, c, d, e, f, g, h) => a + b + c + d + e + f + g + h)
.listen(print); //prints "abcdefgh"
```

#### zip9<A, B, C, D, E, F, G, H, I, T>()

Merges the specified streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zip9(
  new Observable.just("a"),
  new Observable.just("b"),
  new Observable.just("c"),
  new Observable.just("d"),
  new Observable.just("e"),
  new Observable.just("f"),
  new Observable.just("g"),
  new Observable.just("h"),
  new Observable.fromIterable(["i", "dropped"]),
  (a, b, c, d, e, f, g, h, i) => a + b + c + d + e + f + g + h + i)
.listen(print); //prints "abcdefghi"
```

#### zipList<T>()

Merges the iterable streams into one observable sequence using the given zipper function whenever all of the observable sequences have produced an element at a corresponding index.

It applies this function in strict sequence, so the first item emitted by the new Observable will be the result of the function applied to the first item emitted by Observable #1 and the first item emitted by Observable #2; the second item emitted by the new zip-Observable will be the result of the function applied to the second item emitted by Observable #1 and the second item emitted by Observable #2; and so forth. It will only emit as many items as the number of items emitted by the source Observable that emits the fewest items.

[Interactive marble diagram](http://rxmarbles.com/#zip)

Example

```dart
Observable.zipList(
  [
    Observable.just("Hi "),
    Observable.fromIterable(["Friend", "Dropped"]),
  ],
)
.listen(print); // prints ['Hi ', 'Friend']
```


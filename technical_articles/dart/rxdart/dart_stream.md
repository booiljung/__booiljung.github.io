# Dart stream

비동기 데이터 이벤트의 소스.

stream은 일련의 이벤트를 listen하는 방법을 제공합니다. 각 이벤트는 stream의 element라고도 하는 데이터 이벤트이거나 실패한 이벤트인 오류 이벤트입니다. stream이 모든 이벤트를 내 보낸 경우 단일 "완료"이벤트가 listener에게 도달했음을 알립니다.

stream을 `listen`하여 이벤트 생성을 시작하고 이벤트를 listen하는 listener를 설정합니다. listen하면 이벤트를 제공하는 활성 객체인 `streamSubscription` 객체를 listen하고 다시 listen을 중지하거나 subscription에서 이벤트를 일시적으로 일시 중지하는 데 사용할 수 있습니다.

stream에는 두 가지 종류가 있습니다. "Single-subscription" stream과 "Bradcast" stream.

Single-subscription stream을 사용하면 stream의 전체 수명 동안 single listener만 허용됩니다. listener가 있을 때까지 이벤트 생성을 시작하지 않으며 이벤트 소스가 더 많은 정보를 제공 할 수있는 경우에도 listener가 등록 취소 될 때 이벤트 보내기를 중지합니다.

첫 번째 subscription이 취소 된 후에도 single-subscription stream에서 두 번 듣는 것은 허용되지 않습니다.

Single-subscription stream은 일반적으로 파일 I/O와 같이 더 큰 연속 데이터 청크를 streaming하는 데 사용됩니다.

Broadcast stream은 여러 개의 listener를 허용하며 listener가 있는지 여부에 상관없이 준비가되면 이벤트를 시작합니다.

Broadcast stream은 independent event / observers에 사용됩니다.

여러 listener가 single-subscription stream을 listen하려는 경우 `asBroadcaststream`을 사용하여 non-broadcast stream 상단에 broadcast stream을 작성하십시오.

어느 쪽의 종류의 stream에서도, `where`와 `skip`과 같은 stream transformation은, 다른 method가 없는 한, method가 불려 갔던 것과 같은 타입의 stream을 돌려줍니다.

이벤트가 시작되면 해당 시간에 listener가 이벤트를 listen합니다. 이벤트가 시작되는 동안 listener가 broadcast stream에 추가되면 해당 listener는 현재 실행중인 이벤트를 listen하지 않습니다. listener가 취소되면 이벤트 listen이 즉시 중단됩니다. broadcast stream에서 listen는 listen 호출이 발생할 때 아직 fire되지 않은 이벤트 만 포함하는 새 stream에서 listen하는 것으로 처리 될 수 있습니다. 예를 들어 첫 번째 getter는 stream을 listen 한 다음 listener가 listen하는 첫 번째 이벤트를 반환합니다. 이것은 stream에 의해 fire된 최초의 것일 필요는 없지만, broadcast stream의 나머지 이벤트들 중 첫 번째입니다.

"done"이벤트가 시작되면 subscriptor는 이벤트를 받기 전에 subscription 취소됩니다. 이벤트가 보내진 후 stream에는 subscriptor가 없습니다. 이 시점 이후 broadcast stream에 새로운 subscriptor를 추가하는 것은 가능하지만 가능한 한 빨리 새로운 "done"이벤트를 listen합니다.

stream subscription은 항상 "pause"요청을 존중합니다. 필요하다면 입력을 버퍼링해야하지만 종종 입력을 일시 중지하도록 요청할 수도 있습니다.

`isBroadcast`의 기본 구현은 false를 반환합니다. stream에서 상속받은 broadcast stream은 `true`를 반환하려면 `isBroadcast`를 재정의 해야 합니다.

### 생성자

#### stream.empty()

Creates an empty broadcast stream.

빈 broadcast stream을 생성합니다.

This is a stream which does nothing except sending a done event when it's listened to.

이것은 listen 할 때 완료 이벤트를 보내는 것 외에는 아무것도 수행하지 않는 stream입니다.

#### stream.eventTransformed

Creates a stream where all events of an existing stream are piped through a sink-transformation.

기존 stream의 모든 이벤트가 싱크 변환을 통해 pipe 되는 stream을 만듭니다.

The given `mapSink` closure is invoked when the returned stream is listened to. All events from the `source` are added into the event sink that is returned from the invocation. The transformation puts all transformed events into the sink the `mapSink` closure received during its invocation. Conceptually the `mapSink` creates a transformation pipe with the input sink being the returned [EventSink](https://api.dart.dev/stable/2.4.0/dart-async/EventSink-class.html) and the output sink being the sink it received.

주어진 `mapSink` 클로저는 리턴 된 stream이 listen 될 때 호출됩니다. `source`의 모든 이벤트는 호출에서 리턴 된 이벤트 sink에 추가됩니다. 변환은 `mapSink` 클로저가 호출되는 동안 listen 된 모든 변환 이벤트를 sink에 넣습니다. 개념적으로 `mapSink`는 반환 된 [EventSink](https://api.dart.dev/stable/2.4.0/dart-async/EventSink-class.html) 인 입력 sink와 출력 sink를 가진 transformation pipe를 생성합니다.

This constructor is frequently used to build transformers.

이 생성자는 자주 transformer를 만드는 데 사용됩니다.

#### stream.fromFuture

Creates a new single-subscription stream from the future.

향후 single-subscription stream을 새로 만듭니다.

When the future completes, the stream will fire one event, either data or error, and then close with a done-event.

future가 완료되면 stream은 데이터 또는 오류 중 하나의 이벤트를 발생시킨 다음 완료 이벤트로 종료합니다.

#### stream.fromFutures

Create a stream from a group of futures.

future 그룹에서 stream을 만듭니다.

The stream reports the results of the futures on the stream in the order in which the futures complete. Each future provides either a data event or an error event, depending on how the future completes.

stream은 future의 결과를 stream에서 future가 완료된 순서로 보고 합니다. 각각의 future는 future가 어떻게 완료 될지에 따라 데이터 이벤트 또는 오류 이벤트를 제공합니다.

If some futures have already completed when `stream.fromFutures` is called, their results will be emitted in some unspecified order.

`stream.fromFutures`가 호출 될 때 일부 future가 이미 완료된 경우 결과는 지정되지 않은 순서로 방출됩니다.

When all futures have completed, the stream is closed.

모든 future가 완료되면 stream이 닫힙니다.

If `futures` is empty, the stream closes as soon as possible.

`futures`가 비어 있으면 stream은 가능한 빨리 닫힙니다.

#### stream.fromIterable

Creates a single-subscription stream that gets its data from `elements`.

element로부터 데이터를 가져 오는 single-subscription stream을 만듭니다.

The iterable is iterated when the stream receives a listener, and stops iterating if the listener cancels the subscription, or if the [Iterator.moveNext](https://api.dart.dev/stable/2.4.0/dart-core/Iterator/moveNext.html) method returns `false` or throws. Iteration is suspended while the stream subscription is paused.

iterable은 stream이 listener를 listen하면 반복되고 listener가 subscription을 취소하면 반복을 중지하거나 [Iterator.moveNext](https://api.dart.dev/stable/2.4.0/dart-core/Iterator/moveNext.html) 메소드는 `false`를 반환하거나 throw합니다. stream subscription이 일시 중지 된 동안 반복이 일시 중단됩니다.

If calling [Iterator.moveNext](https://api.dart.dev/stable/2.4.0/dart-core/Iterator/moveNext.html) on `elements.iterator` throws, the stream emits that error and then it closes. If reading [Iterator.current](https://api.dart.dev/stable/2.4.0/dart-core/Iterator/current.html) on `elements.iterator` throws, the stream emits that error, but keeps iterating.

`elements.iterator`에 대해 [Iterator.moveNext](https://api.dart.dev/stable/2.4.0/dart-core/Iterator/moveNext.html)를 호출하면 stream이 오류를 내 보낸 다음 닫는다. `elements.iterator`에서 [Iterator.current](https://api.dart.dev/stable/2.4.0/dart-core/Iterator/current.html)를 읽으면 해당 오류가 발생하지만 계속 유지됩니다. 반복을 유지합니다.

#### stream.periodic

Creates a stream that repeatedly emits events at `period` intervals.

`period` 간격으로 이벤트를 반복적으로 내보내는 stream을 만듭니다.

The event values are computed by invoking `computation`. The argument to this callback is an integer that starts with 0 and is incremented for every event.

이벤트 값은 `computation`을 호출하여 계산됩니다. 이 콜백에 대한 인수는 0으로 시작하고 모든 이벤트에 대해 증가하는 정수입니다.

If `computation` is omitted the event values will all be `null`.

`computation`가 생략되면 이벤트 값은 모두 `null`이됩니다.

### 인스턴스 메소드

#### any()

Checks whether `test` accepts any element provided by this stream.

`test`가이 stream에 의해 제공되는 element를 받아 들일지 어떨지를 판정합니다.

Calls `test` on each element of this stream. If the call returns `true`, the returned future is completed with `true` and processing stops.

이 stream의 각 element에 대해 `test`를 호출합니다. 호출이 `true`를 반환하면 반환 된 future는 `true`로 완료되고 처리가 중지됩니다.

If this stream ends without finding an element that `test` accepts, the returned future is completed with `false`.

이 stream이 `test`가 받아 들일 수 있는 element를 찾지 못하고 끝나면 반환 된 future는 `false`로 끝납니다.

If this stream emits an error, or if the call to `test` throws, the returned future is completed with that error, and processing stops.

이 stream이 오류를 발생 시키거나 `test`호출이 발생하면 반환 된 future가 해당 오류로 완료되고 처리가 중지됩니다.

#### asBroadcaststream()

Returns a multi-subscription stream that produces the same events as this.

이 이벤트와 동일한 이벤트를 생성하는 multi-subscription stream을 반환합니다.

The returned stream will subscribe to this stream when its first subscriber is added, and will stay subscribed until this stream ends, or a callback cancels the subscription.

반환 된 stream은 첫 번째 subscriptor가 추가 될 때 이 stream을 subscription하고 이 stream이 끝날 때까지 subscription 상태를 유지하거나 콜백이 subscription을 취소 할 때까지 subscription 상태를 유지합니다.

If `onListen` is provided, it is called with a subscription-like object that represents the underlying subscription to this stream. It is possible to pause, resume or cancel the subscription during the call to `onListen`. It is not possible to change the event handlers, including using [streamSubscription.asFuture](https://api.dart.dev/stable/2.4.0/dart-async/streamSubscription/asFuture.html).

`onListen`이 제공되면, 이 stream에 대한 기본 subscription을 나타내는 subscription과 유사한 객체로 호출됩니다. `onListen`을 호출하는 동안 subscription을 일시 중지, 재개 또는 취소 할 수 있습니다. [streamSubscription.asFuture](https://api.dart.dev/stable/2.4.0/dart-async/streamSubscription/asFuture.html) 사용을 포함하여 이벤트 처리기를 변경할 수 없습니다.

If `onCancel` is provided, it is called in a similar way to `onListen` when the returned stream stops having listener. If it later gets a new listener, the `onListen` function is called again.

`onCancel`이 제공되면 리턴 된 stream이 listener를 가지고 있지 않을 때 `onListen`과 비슷한 방식으로 호출됩니다. 나중에 새로운 listener를 얻으면 `onListen` 함수가 다시 호출됩니다.

Use the callbacks, for example, for pausing the underlying subscription while having no subscribers to prevent losing events, or canceling the subscription when there are no listeners.

예를 들어 기본 subscription을 일시 중지하고 subscriptor가 없어도 이벤트가 손실되지 않도록 콜백을 사용하거나 listen기가 없을 때 subscription을 취소 할 수 있습니다.

#### asyncExpand()

Transforms each element into a sequence of asynchronous events.

각 element를 일련의 비동기 이벤트로 변환합니다.

Returns a new stream and for each event of this stream, do the following:

새 stream을 반환하고 이 stream의 각 이벤트에 대해 다음을 수행합니다.

- If the event is an error event or a done event, it is emitted directly by the returned stream.
- 이벤트가 오류 이벤트 또는 완료 이벤트 인 경우 리턴 된 stream에 의해 직접 생성됩니다.
- Otherwise it is an element. Then the `convert` function is called with the element as argument to produce a convert-stream for the element.
- 그렇지 않으면 element입니다. 그런 다음 element에 대한 변환 stream을 생성하기 위해 element를 인수로 사용하여 `convert` 함수가 호출됩니다.
- If that call throws, the error is emitted on the returned stream.
- 해당 호출이 throw되면 반환 된 stream에서 오류가 발생합니다.
- If the call returns `null`, no further action is taken for the elements.
- 호출이 `null`을 반환하면 element에 대해 더 이상의 조치가 취해지지 않습니다.
- Otherwise, this stream is paused and convert-stream is listened to. Every data and error event of the convert-stream is emitted on the returned stream in the order it is produced. When the convert-stream ends, this stream is resumed.
- 그렇지 않으면이 stream이 일시 중지되고 변환 stream이 listen됩니다. convert-stream의 모든 데이터 및 오류 이벤트는 생성 된 순서대로 반환 된 stream에서 생성됩니다. 변환 stream이 끝나면이 stream이 다시 시작됩니다.

The returned stream is a broadcast stream if this stream is.

반환 된 stream은이 stream이있는 경우 broadcast stream입니다.

#### asyncMap()

Creates a new stream with each data event of this stream asynchronously mapped to a new event.

이 stream의 각 데이터 이벤트를 비동기적으로 새 이벤트에 매핑하여 새 stream을 만듭니다.

This acts like [map](https://api.dart.dev/stable/2.4.0/dart-async/stream/map.html), except that `convert` may return a [Future](https://api.dart.dev/stable/2.4.0/dart-async/Future-class.html), and in that case, this stream waits for that future to complete before continuing with its result.

이것은 [map](https://api.dart.dev/stable/2.4.0/dart-async/stream/map.html)과 비슷하지만, `convert`가 [Future](https : // api.dart.dev/stable/2.4.0/dart-async/Future-class.html),이 경우이 stream은 해당 결과가 계속되기 전에 완료 될 때까지 기다립니다.

The returned stream is a broadcast stream if this stream is.

반환 된 stream은 이 stream이 있는 경우 broadcast stream입니다.

#### cast()

Adapt this stream to be a `stream<R>`.

이 stream을`stream<R>`으로 변경합니다.

This stream is wrapped as a `stream<R>` which checks at run-time that each data event emitted by this stream is also an instance of `R`.

이 stream은 실행시에, 이 stream에 의해 발행 된 각 데이터 이벤트가 `R`의 인스턴스이기도 한 것을 체크하는 `stream<R>`로서 랩됩니다.

#### contains()

Returns whether `needle` occurs in the elements provided by this stream.

이 stream에 의해 제공되는 element로 `needle`이 발생할지 어떨지를 돌려줍니다.

Compares each element of this stream to `needle` using [Object.==](https://api.dart.dev/stable/2.4.0/dart-core/Object/operator_equals.html). If an equal element is found, the returned future is completed with `true`. If this stream ends without finding a match, the future is completed with `false`.

[Object. ==](https://api.dart.dev/stable/2.4.0/dart-core/Object/operator_equals.html)를 사용하여이 stream의 각 element를 `needle`과 비교합니다. 동일한 원소가 발견되면 반환 된 future는 `true`로 완료됩니다. 이 stream이 일치하지 않고 끝나면 future는 `false`로 완료됩니다.

If this stream emits an error, or the call to [Object.==](https://api.dart.dev/stable/2.4.0/dart-core/Object/operator_equals.html) throws, the returned future is completed with that error, and processing stops.

이 stream에서 오류가 발생하거나 [Object. ==](https://api.dart.dev/stable/2.4.0/dart-core/Object/operator_equals.html) 호출이 throw되면 반환되는 future는 다음과 같습니다. 해당 오류로 완료되고 처리가 중지됩니다.

#### distinct()

Skips data events if they are equal to the previous data event.

이전 데이터 이벤트와 동일한 경우 데이터 이벤트를 건너 뜁니다.

The returned stream provides the same events as this stream, except that it never provides two consecutive data events that are equal. That is, errors are passed through to the returned stream, and data events are passed through if they are distinct from the most recently emitted data event.

돌려 주어지는 stream은,이 stream과 같은 이벤트를 제공합니다. 다만, 2 개의 연속한 데이터 이벤트는 결코 제공되지 않습니다. 즉, 오류가 반환 된 stream으로 전달되고 데이터 이벤트는 가장 최근에 방출 된 데이터 이벤트와 구별되는 경우 전달됩니다.

Equality is determined by the provided `equals` method. If that is omitted, the '==' operator on the last provided data element is used.

동등성은 제공된 `equals` 메쏘드에 의해 결정됩니다. 이것이 생략되면, 마지막으로 제공된 데이터 element의 `==`연산자가 사용됩니다.

If `equals` throws, the data event is replaced by an error event containing the thrown error. The behavior is equivalent to the original stream emitting the error event, and it doesn't change the what the most recently emitted data event is.

`equals`가 throw 되면 데이터 이벤트는 던져진 오류를 포함하는 오류 이벤트로 대체됩니다. 이 동작은 오류 이벤트를 내보내는 원본 stream과 동일하며 가장 최근에 방출 된 데이터 이벤트가 변경되지 않습니다.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the `equals` test.

반환 된 stream은 이 stream이있는 경우 broadcast stream입니다. broadcast stream이 두 번 이상 listen되면 각 subscription은 개별적으로 `equals`테스트를 수행합니다.

#### drain()

Discards all data on this stream, but signals when it is done or an error occurred.

이 stream의 모든 데이터를 삭제하지만 완료되거나 오류가 발생한 경우 신호를 보냅니다.

When subscribing using [drain](https://api.dart.dev/stable/2.4.0/dart-async/stream/drain.html), cancelOnError will be true. This means that the future will complete with the first error on this stream and then cancel the subscription. If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

[drain](https://api.dart.dev/stable/2.4.0/dart-async/stream/drain.html)을 사용하여 subscription하면 `cancelOnError`가 `true`가 됩니다. 이것은 future가이 stream의 첫 번째 오류로 완료되고 subscription을 취소한다는 것을 의미합니다. 이 stream에서 오류가 발생하거나`combine `호출이 throw되면 해당 오류로 반환 된 future가 완료되고 처리가 중지됩니다.

In case of a `done` event the future completes with the given `futureValue`.

`done` 이벤트의 경우 future는 주어진 `futureValue`로 완료됩니다.

#### elementAt()

Returns the value of the `index`th data event of this stream.

이 stream의 `index` 번째 데이터 이벤트의 값을 리턴합니다.

Stops listening to this stream after the `index`th data event has been received.

`index` 데이터 이벤트가 listen 된 후 이 stream을 listen하지 않습니다.

Internally the method cancels its subscription after these elements. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

내부적으로 메소드는 이러한 element 다음에 subscription을 취소합니다. 이것은 single-subscription (non-broadcast) stream이 닫히고 이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

If an error event occurs before the value is found, the future completes with this error.

값이 발견되기 전에 오류 이벤트가 발생하면 이 오류로 인해 future가 완료됩니다.

If a done event occurs before the value is found, the future completes with a [RangeError](https://api.dart.dev/stable/2.4.0/dart-core/RangeError-class.html).

값이 발견되기 전에 완료 이벤트가 발생하면 future는 [RangeError](https://api.dart.dev/stable/2.4.0/dart-core/RangeError-class.html)로 완료됩니다.

#### every()

Checks whether `test` accepts all elements provided by this stream.

`test`가 이 stream에 의해 제공되는 모든 element를 받아들이는지 어떤지를 판정합니다.

Calls `test` on each element of this stream. If the call returns `false`, the returned future is completed with `false` and processing stops.

이 stream의 각 element에 대해 `test`를 호출합니다. 호출이 `false`를 반환하면, 반환 된 future는`false`로 끝나고 처리는 중단합니다.

If this stream ends without finding an element that `test` rejects, the returned future is completed with `true`.

이 stream이 `test`가 거부하는 element를 찾지 않고 끝나면 리턴 된 future는`true`로 완료됩니다.

If this stream emits an error, or if the call to `test` throws, the returned future is completed with that error, and processing stops.

이 stream이 오류를 발생 시키거나`test`호출이 발생하면 반환 된 future가 해당 오류로 완료되고 처리가 중지됩니다.

#### expand()

Transforms each element of this stream into a sequence of elements.

이 stream의 각 element를 일련의 element로 변환합니다.

Returns a new stream where each element of this stream is replaced by zero or more data events. The event values are provided as an [Iterable](https://api.dart.dev/stable/2.4.0/dart-core/Iterable-class.html) by a call to `convert` with the element as argument, and the elements of that iterable is emitted in iteration order. If calling `convert` throws, or if the iteration of the returned values throws, the error is emitted on the returned stream and iteration ends for that element of this stream.

이 stream의 각 element가 0 개 이상의 데이터 이벤트로 치환 된 새로운 stream을 돌려줍니다. 이벤트 값은 element를 인수로 사용하여 `convert`를 호출하여 [Iterable](https://api.dart.dev/stable/2.4.0/dart-core/Iterable-class.html)으로 제공됩니다. 그 반복 가능한 element는 반복 순서로 배출됩니다. `convert`를 호출하는 것이 반환 값의 반복이 던지면 반환 된 stream에서 오류가 발생하고 이 stream의 해당 element에 대한 반복이 끝납니다.

Error events and the done event of this stream are forwarded directly to the returned stream.

오류 이벤트 및 이 stream의 `done` 이벤트는 반환 된 stream으로 직접 전달됩니다.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually call `convert` and expand the events.

반환 된 stream은 이 stream이 있는 경우 broadcast stream입니다. subscriptor stream이 두 번 이상 듣게 되면 각 subscription은 개별적으로 `convert`를 호출하고 이벤트를 확장합니다.

#### firstWhere()

Finds the first element of this stream matching `test`.

`test`와 일치하는 이 stream의 첫 번째 element를 찾습니다.

Returns a future that is completed with the first element of this stream that `test` returns `true` for.

`test`가 `true`를 리턴하는 이 stream의 첫 번째 element로 완료되는 future를 리턴합니다.

If no such element is found before this stream is done, and a `orElse` function is provided, the result of calling `orElse` becomes the value of the future. If `orElse` throws, the returned future is completed with that error.

이 stream이 끝나기 전에 그러한 element가 발견되지 않고 `orElse` 함수가 제공되면, `orElse`를 호출 한 결과는 future의 값이 됩니다. `orElse`가 throw되면 리턴 된 future가 그 에러로 완료됩니다.

If this stream emits an error before the first matching element, the returned future is completed with that error, and processing stops.

이 stream이 첫 번째 일치하는 element 앞에 오류를 내면 반환 된 future가 해당 오류로 완료되고 처리가 중지됩니다.

Stops listening to this stream after the first matching element or error has been received.

최초로 일치하는 element 또는 에러가 listen 된 후, 이 stream의 listen을 정지합니다.

Internally the method cancels its subscription after the first element that matches the predicate. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

내부적으로 메소드는 predicate와 일치하는 첫번째 element 다음에 subscription을 취소합니다. 이것은 single-subscription (non-broadcast) stream이 닫히고이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

If an error occurs, or if this stream ends without finding a match and with no `orElse` function provided, the returned future is completed with an error.

오류가 발생하거나이 stream이 일치하지 않고 종료되었거나 `orElse` 함수가 제공되지 않으면 반환 된 future가 오류와 함께 완료됩니다.

#### fold()

Combines a sequence of values by repeatedly applying `combine`.

`combine` 반복적으로 적용하여 값의 시퀀스를 결합합니다.

Similar to [Iterable.fold](https://api.dart.dev/stable/2.4.0/dart-async/stream/fold.html), this function maintains a value, starting with `initialValue` and updated for each element of this stream. For each element, the value is updated to the result of calling `combine` with the previous value and the element.

[Iterable.fold](https://api.dart.dev/stable/2.4.0/dart-async/stream/fold.html)와 마찬가지로 이 함수는 'initialValue'로 시작하여 각각에 대해 업데이트 된 값을 유지합니다. 이 stream의 element 각 element에 대해 값은 이전 값 및 element와`combine`를 호출 한 결과로 업데이트됩니다.

When this stream is done, the returned future is completed with the value at that time. For an empty stream, the future is completed with `initialValue`.

이 stream이 완료되면 반환 된 future가 그 시점의 값으로 완료됩니다. 빈 stream의 경우, future는`initialValue`로 완료됩니다.

If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

이 stream에서 오류가 발생하거나`combine`호출이 throw되면 해당 오류로 반환 된 future가 완료되고 처리가 중지됩니다.

#### forEach()

Executes `action` on each element of this stream.

이 stream의 각 element에 대해 `action`을 실행합니다.

Completes the returned [Future](https://api.dart.dev/stable/2.4.0/dart-async/Future-class.html) when all elements of this stream have been processed.

이 stream의 모든 element가 처리 될 때 반환 된 [Future](https://api.dart.dev/stable/2.4.0/dart-async/Future-class.html)를 완성합니다.

If this stream emits an error, or if the call to `action` throws, the returned future completes with that error, and processing stops.

이 stream이 오류를 발생 시키거나 `action`에 대한 호출이 발생하면 반환 된 future는 그 오류로 완료되고 처리가 중지됩니다.

#### handleError()

Creates a wrapper stream that intercepts some errors from this stream.

이 stream의 일부 오류를 가로 채기위한 래퍼 stream을 작성합니다.

If this stream sends an error that matches `test`, then it is intercepted by the `onError` function.

이 stream이 `test`와 일치하는 에러를 보내면, `onError` 함수에 의해 인터셉트됩니다.

The `onError` callback must be of type `void onError(error)` or `void onError(error, StackTrace stackTrace)`. The function type determines whether `onError` is invoked with a stack trace argument. The stack trace argument may be `null` if this stream received an error without a stack trace.

`onError` 콜백은 반드시 `void onError(error)` 또는 `void onError(error, StackTrace stackTrace)` 형이어야 합니다. 함수 타입은 `onError`가 스택 추적 인수로 호출되는지 여부를 결정합니다. 스택 추적없이 이 stream이 오류를 listen하면 스택 추적 인수가 `null`일 수 있습니다.

An asynchronous error `error` is matched by a test function if `test(error)` returns true. If `test` is omitted, every error is considered matching.

비동기 에러 `error`는 `test(error)`가 `true`를 반환하면 테스트 함수에 의해 매치됩니다. `test`가 생략되면, 모든 에러는 일치하는 것으로 간주됩니다.

If the error is intercepted, the `onError` function can decide what to do with it. It can throw if it wants to raise a new (or the same) error, or simply return to make this stream forget the error. If the received `error` value is thrown again by the `onError` function, it acts like a `rethrow` and it is emitted along with its original stack trace, not the stack trace of the `throw` inside `onError`.

에러가 인터셉트되면, `onError` 함수는 그것으로 무엇을 할 것인지를 결정할 수 있습니다. 새로운 (또는 같은) 에러를 발생시키고 싶을 경우, 또는 단순히이 stream이 에러를 잊어 버리는 경우에 리턴 할 수 있습니다. `onError` 함수에 의해 `error` 값이 다시 던져지면`rethrow`처럼 동작하고 `onError` 내부에 `throw`의 스택 트레이스가 아닌 원래 스택 트레이스와 함께 방출됩니다.

If you need to transform an error into a data event, use the more generic [stream.transform](https://api.dart.dev/stable/2.4.0/dart-async/stream/transform.html) to handle the event by writing a data event to the output sink.

오류를 데이터 이벤트로 변환해야하는 경우보다 일반적인 [stream.transform](https://api.dart.dev/stable/2.4.0/dart-async/stream/transform.html)을 사용하여 출력 싱크에 데이터 이벤트를 작성하여 이벤트를 처리하십시오.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the `test` and handle the error.

반환 된 stream은 이 stream이 있는 경우 broadcast stream입니다. broadcast stream이 두 번 이상 listen되면 각 subscription은 개별적으로 `test`를 수행하고 오류를 처리합니다.

#### join()

Combines the string representation of elements into a single string.

element의 문자열 표현을 단일 문자열로 결합합니다.

Each element is converted to a string using its [Object.toString](https://api.dart.dev/stable/2.4.0/dart-core/Object/toString.html) method. If `separator` is provided, it is inserted between element string representations.

각 element는 [Object.toString](https://api.dart.dev/stable/2.4.0/dart-core/Object/toString.html) 메서드를 사용하여 문자열로 변환됩니다. `separator`가 제공되면 element 문자열 표현 사이에 삽입됩니다.

The returned future is completed with the combined string when this stream is done.

이 stream이 완료되면 반환된 future는 결합된 문자열로 완료됩니다.

If this stream emits an error, or the call to [Object.toString](https://api.dart.dev/stable/2.4.0/dart-core/Object/toString.html) throws, the returned future is completed with that error, and processing stops.

이 stream에서 오류가 발생하거나 [Object.toString](https://api.dart.dev/stable/2.4.0/dart-core/Object/toString.html)을 호출하면 반환 된 future가 완료됩니다. 그 오류와 함께, 그리고 처리가 중지됩니다.

#### lastWhere()

Finds the last element in this stream matching `test`.

이 stream 내에서 `test`와 일치하는 마지막 element를 찾습니다.

If this stream emits an error, the returned future is completed with that error, and processing stops.

이 stream에서 오류가 발생하면 반환 된 오류가 해당 오류와 함께 완료되고 처리가 중지됩니다.

Otherwise as [firstWhere](https://api.dart.dev/stable/2.4.0/dart-async/stream/firstWhere.html), except that the last matching element is found instead of the first. That means that a non-error result cannot be provided before this stream is done.

그렇지 않으면 [firstWhere](https://api.dart.dev/stable/2.4.0/dart-async/stream/firstWhere.html)와 같지만 첫 번째 element 대신 마지막으로 일치하는 element가 발견됩니다. 즉, 이 stream이 완료되기 전에 오류가 아닌 결과를 제공 할 수 없습니다.

#### listen()

Adds a subscription to this stream.

이 stream에 subscription을 추가합니다.

Returns a [streamSubscription](https://api.dart.dev/stable/2.4.0/dart-async/streamSubscription-class.html) which handles events from this stream using the provided `onData`, `onError` and `onDone` handlers. The handlers can be changed on the subscription, but they start out as the provided functions.

제공된 `onData`, `onError` 및 `onDone`  핸들러를 사용하여 이 stream의 이벤트를 처리하는 [streamSubscription](https://api.dart.dev/stable/2.4.0/dart-async/streamSubscription-class.html)을 반환합니다. 핸들러는 subscription에서 변경할 수 있지만 제공된 기능으로 시작합니다.

On each data event from this stream, the subscriber's `onData` handler is called. If `onData` is `null`, nothing happens.

이 stream의 각 데이터 이벤트에서 가입자의 onData 처리기가 호출됩니다. `onData`가 `null`이라면 아무 일도 일어나지 않습니다.

On errors from this stream, the `onError` handler is called with the error object and possibly a stack trace.

이 stream의 오류가 발생하면 onError 핸들러가 오류 객체 및 스택 추적과 함께 호출됩니다.

The `onError` callback must be of type `void onError(error)` or `void onError(error, StackTrace stackTrace)`. If `onError` accepts two arguments it is called with the error object and the stack trace (which could be `null` if this stream itself received an error without stack trace). Otherwise it is called with just the error object. If `onError` is omitted, any errors on this stream are considered unhandled, and will be passed to the current [Zone](https://api.dart.dev/stable/2.4.0/dart-async/Zone-class.html)'s error handler. By default unhandled async errors are treated as if they were uncaught top-level errors.

`onError` 콜백은 `void onError(error)` 또는 `void onError(error, StackTrace stackTrace)`타입이어야합니다. `onError`가 두 개의 인수를 받아들이면 오류 객체와 스택 트레이스 (이 트레이스 자체가 스택 트레이스 없이 에러를 listen하면 `null`이 될 수 있다.)와 함께 호출된다. 그렇지 않으면 오류 오브젝트만으로 호출됩니다. `onError`가 생략되면 이 stream의 모든 오류는 처리되지 않은 것으로 간주되어 현재 [Zone](https://api.dart.dev/stable/2.4.0/dart-async/Zone-class.html)로 전달됩니다.)의 오류 처리기로 전달된다. 기본적으로 처리되지 않은 비동기 오류는 캐치되지 않은 최상위 오류 인 것처럼 처리됩니다.

If this stream closes and sends a done event, the `onDone` handler is called. If `onDone` is `null`, nothing happens.

이 stream이 닫히고 done 이벤트를 보내면 `onDone` 핸들러가 호출됩니다. `onDone`이 `null`이라면 아무 일도 일어나지 않습니다.

If `cancelOnError` is true, the subscription is automatically canceled when the first error event is delivered. The default is `false`.

`cancelOnError`가 참이면 첫 x 째 오류 이벤트가 전달되면 subscription이 자동으로 취소됩니다. 기본값은 `false`입니다.

While a subscription is paused, or when it has been canceled, the subscription doesn't receive events and none of the event handler functions are called.

subscription이 일시 중지되었거나 취소되었을 때 subscription은 이벤트를 받지 않으며 이벤트 처리기 기능이 호출되지 않습니다.

#### map()

Transforms each element of this stream into a new stream event.

이 stream의 각 element를 새로운 stream 이벤트로 변환합니다.

Creates a new stream that converts each element of this stream to a new value using the `convert` function, and emits the result.

이 stream의 각 element를`convert` 함수를 사용해 새로운 값으로 변환 해, 결과를내는 새로운 stream을 작성합니다.

For each data event, `o`, in this stream, the returned stream provides a data event with the value `convert(o)`. If `convert` throws, the returned stream reports it as an error event instead.

이 stream에서 각 데이터 이벤트 `o`에 대해 반환 된 stream은 `convert(o)`값을 가진 데이터 이벤트를 제공합니다. `convert`가 throw되면 반환 된 stream은 오류 이벤트로 보고합니다.

Error and done events are passed through unchanged to the returned stream.

오류 및 완료 이벤트는 변경되지 않고 리턴 된 stream으로 전달됩니다.

The returned stream is a broadcast stream if this stream is. The `convert` function is called once per data event per listener. If a broadcast stream is listened to more than once, each subscription will individually call `convert` on each data event.

반환 된 stream은 이 stream이 있는 경우 broadcast stream입니다. `convert` 함수는 listener 당 데이터 이벤트 당 한 번 호출됩니다. broadcast stream이 두 번 이상 listen되면 각 subscription은 각 데이터 이벤트에서 개별적으로 `convert`를 호출합니다.

Unlike [transform](https://api.dart.dev/stable/2.4.0/dart-async/stream/transform.html), this method does not treat the stream as chunks of a single value. Instead each event is converted independently of the previous and following events, which may not always be correct. For example, UTF-8 encoding, or decoding, will give wrong results if a surrogate pair, or a multibyte UTF-8 encoding, is split into separate events, and those events are attempted encoded or decoded independently.

[transform](https://api.dart.dev/stable/2.4.0/dart-async/stream/transform.html)과 달리 이 메서드는 stream을 단일 값의 청크로 처리하지 않습니다. 대신 각 이벤트는 이전 이벤트 및 다음 이벤트와 독립적으로 변환됩니다. 이는 항상 올바르지 않을 수 있습니다. 예를 들어 서로 게이트 쌍 또는 멀티 바이트 UTF-8 인코딩이 별도의 이벤트로 분리되어 있고 UTF-8 인코딩 또는 디코딩의 경우 잘못된 결과가 발생하며 이러한 이벤트는 독립적으로 인코딩되거나 디코딩됩니다.

#### pipe()

Pipes the events of this stream into `streamConsumer`.

이 stream의 이벤트를 `streamConsumer`에 pipe합니다.

All events of this stream are added to `streamConsumer` using [streamConsumer.addstream](https://api.dart.dev/stable/2.4.0/dart-async/streamConsumer/addstream.html). The `streamConsumer` is closed when this stream has been successfully added to it - when the future returned by `addstream` completes without an error.

이 stream의 모든 이벤트는 [streamConsumer.addstream](https://api.dart.dev/stable/2.4.0/dart-async/streamConsumer/addstream.html)을 사용하여 `streamConsumer`에 추가됩니다. `streamConsumer`는 이 stream이 성공적으로 추가되었을 때 닫힙니다.`addstream`에 의해 리턴 된 future가 오류없이 완료 될 때입니다.

Returns a future which completes when this stream has been consumed and the consumer has been closed.

이 stream이 소비되어 컨슈머 (consumer)가 클로즈되었을 때에 완료하는 장래를 돌려줍니다.

The returned future completes with the same result as the future returned by [streamConsumer.close](https://api.dart.dev/stable/2.4.0/dart-async/streamConsumer/close.html). If the call to [streamConsumer.addstream](https://api.dart.dev/stable/2.4.0/dart-async/streamConsumer/addstream.html) fails in some way, this method fails in the same way.

반환 된 future는 [streamConsumer.close](https://api.dart.dev/stable/2.4.0/dart-async/streamConsumer/close.html)에 의해 반환 된 future와 동일한 결과로 완료됩니다. [streamConsumer.addstream](https://api.dart.dev/stable/2.4.0/dart-async/streamConsumer/addstream.html)에 대한 호출이 어떤 식 으로든 실패하면이 방법은 같은 방식으로 실패합니다.

#### reduce()

Combines a sequence of values by repeatedly applying `combine`.

`combine`를 반복적으로 적용하여 값의 시퀀스를 결합합니다.

Similar to [Iterable.reduce](https://api.dart.dev/stable/2.4.0/dart-async/stream/reduce.html), this function maintains a value, starting with the first element of this stream and updated for each further element of this stream. For each element after the first, the value is updated to the result of calling `combine` with the previous value and the element.

[Iterable.reduce](https://api.dart.dev/stable/2.4.0/dart-async/stream/reduce.html)와 마찬가지로이 함수는 이 stream의 첫 번째 element부터 시작하여 값을 유지하고 이 stream의 각 element마다 갱신됩니다. 첫 번째 element 이후의 각 element에 대해 값은 이전 값 및 element와`combine`를 호출 한 결과로 업데이트 됩니다.

When this stream is done, the returned future is completed with the value at that time.

이 stream이 완료되면 반환 된 future가 그 시점의 값으로 완료됩니다.

If this stream is empty, the returned future is completed with an error. If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

이 stream이 비어 있으면 리턴 된 future가 오류와 함께 완료됩니다. 이 stream에서 오류가 발생하거나`combine`호출이 throw되면 해당 오류로 반환 된 future가 완료되고 처리가 중지됩니다.

#### singleWhere()

Finds the single element in this stream matching `test`.

이 stream 내의`test`와 일치하는 단일 element를 찾습니다.

Like [lastWhere](https://api.dart.dev/stable/2.4.0/dart-async/stream/lastWhere.html), except that it is an error if more than one matching element occurs in this stream.

[lastWhere](https://api.dart.dev/stable/2.4.0/dart-async/stream/lastWhere.html)와 유사하지만 이 stream에서 일치하는 element가 두 개 이상 발생하면 오류가 발생합니다.

#### skip()

Skips the first `count` data events from this stream.

이 stream으로부터 최초의 `count` 데이터 이벤트를 스킵합니다.

Returns a stream that emits the same events as this stream would if listened to at the same time, except that the first `count` data events are not emitted. The returned stream is done when this stream is.

최초의 `count` 데이터 이벤트가 출력되지 않는 것을 제외하면,이 stream과 같은 이벤트를 동시에 내보내는 stream을 리턴합니다. 리턴 된 stream은이 stream이있을 때 완료됩니다.

If this stream emits fewer than `count` data events before being done, the returned stream emits no data events.

이 stream이 완료되기 전에 `count` 데이터 이벤트보다 적은 수의 이벤트를 방출하면 리턴 된 stream은 데이터 이벤트를 방출하지 않습니다.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only counted from the time the returned stream is listened to.

반환 된 stream은 이 stream이 있는 경우 broadcast stream입니다. broadcast stream의 경우 이벤트는 반환 된 stream을 listen 한 시간부터 계산됩니다.

#### skipWhile()

Skip data events from this stream while they are matched by `test`.

`test`와 일치하는 동안 이 stream에서 데이터 이벤트를 건너 뜁니다.

Returns a stream that emits the same events as this stream, except that data events are not emitted until a data event fails `test`. The test fails when called with a data event if it returns a non-`true` value or if the call to `test` throws. If the call throws, the error is emitted as an error event on the returned stream instead of the data event, otherwise the event that made `test` return non-true is emitted as the first data event.

데이터 이벤트가 `test`에 실패 할 때까지 데이터 이벤트가 발행되지 않는 것을 제외하면, 이 stream과 같은 이벤트를 내보내는 stream을 리턴합니다. 테스트는`-true`가 아닌 값을 반환하거나`test`에 대한 호출이 발생하면 데이터 이벤트와 함께 호출 될 때 실패합니다. 호출이 throw되면 데이터 이벤트 대신 반환 된 stream에서 오류 이벤트로 오류가 발생합니다. 그렇지 않으면 `test`가 `true`가 아닌 것으로 반환 한 이벤트가 첫 번째 데이터 이벤트로 방출됩니다.

Error and done events are provided by the returned stream unmodified.

오류 및 완료 이벤트는 수정되지 않은 리턴 된 stream에 의해 제공됩니다.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only tested from the time the returned stream is listened to.

반환 된 stream은이 stream이있는 경우 broadcast stream입니다. broadcast stream의 경우 이벤트는 반환 된 stream을 listen 한 시점부터 만 테스트됩니다.

#### take()

Provides at most the first `count` data events of this stream.

이 stream의 최초의`count` 데이터 이벤트를 제공합니다.

Returns a stream that emits the same events that this stream would if listened to at the same time, until either this stream ends or it has emitted `count` data events, at which point the returned stream is done.

이 stream의 종료시, 또는 `count` 데이터 이벤트의 출력까지,이 stream이 동시에 listen하고 있는 경우와 같은 이벤트를 발행하는 stream을 돌려줍니다.

If this stream produces fewer than `count` data events before it's done, so will the returned stream.

이 stream이 완료되기 전에 `count` 데이터 이벤트보다 적은 수를 생성하면 반환 된 stream도 반환됩니다.

Starts listening to this stream when the returned stream is listened to and stops listening when the first `count` data events have been received.

최초의 `count` 데이터 이벤트가 listen되었을 때, 반환 된 stream을 listen하고 listen를 정지하면, 이 stream의 listen를 개시합니다.

This means that if this is a single-subscription (non-broadcast) streams it cannot be reused after the returned stream has been listened to.

즉, single-subscription (non-roadcast) stream 인 경우 반환 된 stream을 listen 한 후에 다시 사용할 수 없습니다.

If this is a broadcast stream, the returned stream is a broadcast stream. In that case, the events are only counted from the time the returned stream is listened to.

이것이 broadcast stream이면 리턴 된 stream은 broadcast stream입니다. 이 경우 이벤트는 반환 된 stream을 listen 한 시간부터 계산됩니다.

#### takeWhile()

Forwards data events while `test` is successful.

`test`가 성공한 동안 데이터 이벤트를 전달합니다.

Returns a stream that provides the same events as this stream until `test` fails for a data event. The returned stream is done when either this stream is done, or when this stream first emits a data event that fails `test`.

데이터 이벤트의 `test`가 실패 할 때까지이 stream과 같은 이벤트를 제공하는 stream을 리턴합니다. 돌려 주어지는 stream은,이 stream이 완료했을 때, 또는이 stream이 최초로`test`에 실패한 데이터 이벤트를 발행했을 때에 행해집니다.

The `test` call is considered failing if it returns a non-`true` value or if it throws. If the `test` call throws, the error is emitted as the last event on the returned streams.

`test` 호출은`true`가 아닌 값을 반환하거나 던지면 실패한 것으로 간주됩니다. `test` 호출이 던지면 반환 된 stream의 마지막 이벤트로 오류가 발생합니다.

Stops listening to this stream after the accepted elements.

수락 된 element 뒤에 이 stream을 listen하지 않습니다.

Internally the method cancels its subscription after these elements. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

내부적으로 메소드는 이러한 element 다음에 subscription을 취소합니다. 이것은 single-subscription (non-subscription) stream이 닫히고이 메서드를 호출 한 후에 다시 사용할 수 없음을 의미합니다.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only tested from the time the returned stream is listened to.

반환 된 stream은이 stream이있는 경우 broadcast stream입니다. broadcast stream의 경우 이벤트는 반환 된 stream을 listen 한 시점부터 만 테스트됩니다.

#### timeout()

Creates a new stream with the same events as this stream.

이 stream과 같은 이벤트를 가지는 새로운 stream을 작성합니다.

Whenever more than `timeLimit` passes between two events from this stream, the `onTimeout` function is called, which can emit further events on the returned stream.

이 stream에서 두 개의 이벤트 사이에`timeLimit`이상이 전달 될 때마다 `onTimeout` 함수가 호출됩니다.이 함수는 리턴 된 stream에서 더 많은 이벤트를 방출 할 수 있습니다.

The countdown doesn't start until the returned stream is listened to. The countdown is reset every time an event is forwarded from this stream, or when this stream is paused and resumed.

반환 된 stream을 listen 할 때까지 카운트 다운이 시작되지 않습니다. 카운트 다운은 이 stream에서 이벤트가 전달 될 때마다 또는이 stream이 일시 중지되었다가 다시 시작될 때마다 재설정됩니다.

The `onTimeout` function is called with one argument: an [EventSink](https://api.dart.dev/stable/2.4.0/dart-async/EventSink-class.html) that allows putting events into the returned stream. This `EventSink` is only valid during the call to `onTimeout`. Calling [EventSink.close](https://api.dart.dev/stable/2.4.0/dart-async/EventSink/close.html) on the sink passed to `onTimeout` closes the returned stream, and no further events are processed.

`onTimeout` 함수는 리턴 된 stream에 이벤트를 넣을 수 있는 [EventSink](https://api.dart.dev/stable/2.4.0/dart-async/EventSink-class.html) 인자를 가지고 호출됩니다. 이`EventSink`는 `onTimeout`을 호출하는 동안에 만 유효합니다. `onTimeout`에 전달 된 싱크의 [EventSink.close](https://api.dart.dev/stable/2.4.0/dart-async/EventSink/close.html)를 호출하면 리턴 된 stream이 닫히고 더 이상 이벤트가 처리되지 않습니다.

If `onTimeout` is omitted, a timeout will just put a [TimeoutException](https://api.dart.dev/stable/2.4.0/dart-async/TimeoutException-class.html) into the error channel of the returned stream. If the call to `onTimeout` throws, the error is emitted on the returned stream.

`onTimeout`이 생략되면, 타임 아웃은 리턴된 stream의 에러 채널에 [TimeoutException](https://api.dart.dev/stable/2.4.0/dart-async/TimeoutException-class.html)을 넣을 것입니다. onTimeout에 대한 호출이 발생하면 반환 된 stream에서 오류가 발생합니다.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will have its individually timer that starts counting on listen, and the subscriptions' timers can be paused individually.

반환 된 stream은 이 stream이있는 경우 broadcast stream입니다. broadcast stream을 여러 번 듣게 되면 각 subscription마다 개별적으로 타이머가 시작되고 listen 대기가 시작되고 subscription 타이머는 개별적으로 일시 중지 될 수 있습니다.

#### toList()

Collects all elements of this stream in a [List](https://api.dart.dev/stable/2.4.0/dart-core/List-class.html).

[List](https://api.dart.dev/stable/2.4.0/dart-core/List-class.html)에서 이 stream의 모든 element를 수집합니다.

Creates a `List<T>` and adds all elements of this stream to the list in the order they arrive. When this stream ends, the returned future is completed with that list.

`List<T>`를 작성해, 이 stream의 모든 element를 도착 순서에 리스트에 추가합니다. 이 stream이 끝나면 해당 목록에서 반환 된 future가 완료됩니다.

If this stream emits an error, the returned future is completed with that error, and processing stops.

이 stream에서 오류가 발생하면 반환 된 오류가 해당 오류와 함께 완료되고 처리가 중지됩니다.

#### toSet()

Collects the data of this stream in a [Set](https://api.dart.dev/stable/2.4.0/dart-core/Set-class.html).

[Set](https://api.dart.dev/stable/2.4.0/dart-core/Set-class.html)에서이 stream의 데이터를 수집합니다.

Creates a `Set<T>` and adds all elements of this stream to the set. in the order they arrive. When this stream ends, the returned future is completed with that set.

`Set<T>`를 작성해,이 stream의 모든 element를 세트에 추가합니다. 그들이 도착하는 순서대로. 이 stream이 끝나면 반환 된 future가 해당 집합으로 완료됩니다.

The returned set is the same type as returned by `new Set<T>()`. If another type of set is needed, either use [forEach](https://api.dart.dev/stable/2.4.0/dart-async/stream/forEach.html) to add each element to the set, or use `toList().then((list) => new SomeOtherSet.from(list))` to create the set.

리턴 된 세트는 `new Set<T>()`에 의해 리턴 된 것과 같은 타입입니다. 다른 유형의 세트가 필요한 경우 [forEach](https://api.dart.dev/stable/2.4.0/dart-async/stream/forEach.html)를 사용하여 각 element를 세트에 추가하거나 `toList().((list) => new SomeOtherSet.from(list))`를 호출하여 세트를 만듭니다.

If this stream emits an error, the returned future is completed with that error, and processing stops.

이 stream에서 오류가 발생하면 반환 된 오류가 해당 오류와 함께 완료되고 처리가 중지됩니다.

#### transform()

Applies  `streamTransformer` to this stream.

이 stream에 `streamTransformer`를 적용합니다.

Returns the transformed stream, that is, the result of `streamTransformer.bind(this)`. This method simply allows writing the call to `streamTransformer.bind` in a chained fashion, like

변환 된 stream, 즉 `streamTransformer.bind(this)`의 결과를 리턴합니다. 이 메소드는 단순히 `streamTransformer.bind`에 대한 호출을 체인 방식으로 작성하는 것을 허용합니다.

```dart
stream.map(mapping).transform(transformation).toList()
```

which can be more convenient than calling `bind` directly.

`bind`를 직접 호출하는 것보다 더 편리 할 수 있습니다.

The `streamTransformer` can return any stream. Whether the returned stream is a broadcast stream or not, and which elements it will contain, is entirely up to the transformation.

`streamTransformer`는 모든 stream을 반환 할 수 있습니다. 돌려 주어지는 stream이 broadcast stream인가 어떤가, 어느 element가 포함되는지는 전적으로 변환에 달려 있습니다.

This method should always be used for transformations which treat the entire stream as representing a single value which has perhaps been split into several parts for transport, like a file being read from disk or being fetched over a network. The transformation will then produce a new stream which transforms the stream's value incrementally (perhaps using [Converter.startChunkedConversion](https://api.dart.dev/stable/2.4.0/dart-convert/Converter/startChunkedConversion.html)). The resulting stream may again be chunks of the result, but does not have to correspond to specific events from the source string.

이 메소드는 파일 전체를 디스크에서 읽어들이거나 네트워크에서 가져 오는 것과 같이 전송을 위해 여러 부분으로 분할 된 단일 값을 나타내는 것으로 전체 stream을 처리하는 변환에 항상 사용해야합니다. 그런 다음 변환은 stream의 값을 점진적으로 변환하는 새 stream을 생성합니다 (아마도 [Converter.startChunkedConversion](https://api.dart.dev/stable/2.4.0/dart-convert/Converter/startChunkedConversion.html)을 사용하여). 결과 stream은 결과의 청크 일 수 있지만 소스 문자열의 특정 이벤트와 일치 할 필요는 없습니다.

#### where()

Creates a new stream from this stream that discards some elements.

이 stream으로부터, 일부의 element를 파기하는 새로운 stream을 작성합니다.

The new stream sends the same error and done events as this stream, but it only sends the data events that satisfy the `test`.

새 stream은이 stream과 동일한 오류 및 완료 이벤트를 전송하지만 `테스트`를 충족하는 데이터 이벤트 만 보냅니다.

If the `test` function throws, the data event is dropped and the error is emitted on the returned stream instead.

`test` 함수가 던져지면, 데이터 이벤트는 삭제되고 반환 된 stream에 에러가 발생합니다.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the `test`.

반환 된 stream은이 stream이 있는 경우 broadcast stream입니다. broadcast stream이 두 번 이상 listen되면 각 subscription은 개별적으로 `test`를 수행합니다.

### 클래스 메소드

#### castFrom()

Adapts `source` to be a `stream<T>`.

`source` 를 `stream<T>`로 변경합니다.

This allows `source` to be used at the new type, but at run-time it must satisfy the requirements of both the new type and its original type.

이것은 새로운 타입에서`source`를 사용할 수있게 하지만 실행시에는 새로운 타입과 원래 타입의 요구 사항을 모두 만족해야 합니다.

Data events created by the source stream must also be instances of `T`.

소스 stream에 의해 생성 된 데이터 이벤트는 또한 `T`의 인스턴스여야 합니다.


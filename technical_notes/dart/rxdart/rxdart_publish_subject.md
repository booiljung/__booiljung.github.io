# RxDart PublishSubject<T>

Exactly like a normal broadcast StreamController with one exception: `stream` returns an `Observable` instead of a `Stream`.

하나의 예외를 제외하고는 보통의 방송 StreamController와 정확히 같습니다 :`stream`은`Stream` 대신에`Observable`을 리턴합니다.

This Subject allows sending data, error and done events to the listener.

이 Subject는 데이터, 오류 및 완료 이벤트를 수신기로 보낼 수있게합니다.

PublishSubject is, by default, a broadcast (aka hot) controller, in order to fulfill the Rx Subject contract. This means the Subject's `stream` can be listened to multiple times.

PublishSubject는 Rx Subject 계약을 이행하기 위해 기본적으로 브로드 캐스트 (일명 핫) 컨트롤러입니다. 이것은 Subject의`stream`을 여러 번 듣게된다는 것을 의미합니다.

Example

```dart
final subject = new PublishSubject<int>();

// observer1 will receive all data and done events
subject.stream.listen(observer1);
subject.add(1);
subject.add(2);

// observer2 will only receive 3 and done event
subject.stream.listen(observe2);
subject.add(3);
subject.close();
```

### 인스턴스 메소드

#### add()

Sends a data `event`.

Listeners receive this event in a later microtask.

Note that a synchronous controller (created by passing true to the `sync` parameter of the `StreamController` constructor) delivers events immediately. Since this behavior violates the contract mentioned here, synchronous controllers should only be used as described in the documentation to ensure that the delivered events always *appear* as if they were delivered in a separate microtask.

#### addError()

Sends or enqueues an error event.

If `error` is `null`, it is replaced by a `NullThrownError`.

Listeners receive this event at a later microtask. This behavior can be overridden by using `sync` controllers. Note, however, that sync controllers have to satisfy the preconditions mentioned in the documentation of the constructors.

#### addStream()

Receives events from `source` and puts them into this controller's stream.

Returns a future which completes when the source stream is done.

Events must not be added directly to this controller using [add](https://pub.dev/documentation/rxdart/latest/rx/Subject/add.html), [addError](https://pub.dev/documentation/rxdart/latest/rx/Subject/addError.html), [close](https://pub.dev/documentation/rxdart/latest/rx/Subject/close.html) or [addStream](https://pub.dev/documentation/rxdart/latest/rx/Subject/addStream.html), until the returned future is complete.

Data and error events are forwarded to this controller's stream. A done event on the source will end the `addStream` operation and complete the returned future.

If `cancelOnError` is true, only the first error on `source` is forwarded to the controller's stream, and the `addStream` ends after this. If `cancelOnError` is false, all errors are forwarded and only a done event will end the `addStream`. If `cancelOnError` is omitted, it defaults to false.

#### any()

Checks whether `test` accepts any element provided by this stream.

Calls `test` on each element of this stream. If the call returns `true`, the returned future is completed with `true` and processing stops.

If this stream ends without finding an element that `test` accepts, the returned future is completed with `false`.

If this stream emits an error, or if the call to `test` throws, the returned future is completed with that error, and processing stops.

#### asBroadcastStream()

Returns a multi-subscription stream that produces the same events as this.

The returned stream will subscribe to this stream when its first subscriber is added, and will stay subscribed until this stream ends, or a callback cancels the subscription.

If onListen is provided, it is called with a subscription-like object that represents the underlying subscription to this stream. It is possible to pause, resume or cancel the subscription during the call to onListen. It is not possible to change the event handlers, including using StreamSubscription.asFuture.

If onCancel is provided, it is called in a similar way to onListen when the returned stream stops having listener. If it later gets a new listener, the onListen function is called again.

Use the callbacks, for example, for pausing the underlying subscription while having no subscribers to prevent losing events, or canceling the subscription when there are no listeners.

#### asyncExpand<S>()

Maps each emitted item to a new `Stream` using the given mapper, then subscribes to each new stream one after the next until all values are emitted.

asyncExpand is similar to flatMap, but ensures order by guaranteeing that all items from the created stream will be emitted before moving to the next created stream. This process continues until all created streams have completed.

This is functionally equivalent to `concatMap`, which exists as an alias for a more fluent Rx API.

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

#### buffer()

Creates an Observable where each item is a `List` containing the items from the source sequence.

This `List` is emitted every time `window` emits an event.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (i) => i)
  .buffer(new Stream.periodic(const Duration(milliseconds: 160), (i) => i))
  .listen(print); // prints [0, 1] [2, 3] [4, 5] ...
```

#### bufferCount()

Buffers a number of values from the source Observable by `count` then emits the buffer and clears it, and starts a new buffer each `startBufferEvery` values. If `startBufferEvery` is not provided, then new buffers are started immediately at the start of the source and when each buffer closes and is emitted.

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
  .listen(print); // prints 
```

#### bufferTest()

Creates an Observable where each item is a `List` containing the items from the source sequence, batched whenever test passes.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (int i) => i)
  .bufferTest((i) => i % 2 == 0)
  .listen(print); // prints [0], [1, 2] [3, 4] [5, 6] ...
```

#### bufferTime()

Creates an Observable where each item is a `List` containing the items from the source sequence, sampled on a time frame with `duration`.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (int i) => i)
  .bufferTime(const Duration(milliseconds: 220))
  .listen(print); // prints [0, 1] [2, 3] [4, 5] ...
```

#### cast<R>()

Adapt this stream to be a `Stream<R>`.

If this stream already has the desired type, its returned directly. Otherwise it is wrapped as a `Stream<R>` which checks at run-time that each data event emitted by this stream is also an instance of `R`.

#### close()

Closes the stream.

Listeners receive the done event at a later microtask. This behavior can be overridden by using `sync` controllers. Note, however, that sync controllers have to satisfy the preconditions mentioned in the documentation of the constructors.

#### concatMap<S>()

Maps each emitted item to a new `Stream` using the given mapper, then subscribes to each new stream one after the next until all values are emitted.

ConcatMap is similar to flatMap, but ensures order by guaranteeing that all items from the created stream will be emitted before moving to the next created stream. This process continues until all created streams have completed.

This is a simple alias for Dart Stream's `asyncExpand`, but is included to ensure a more consistent Rx API.

Example

```dart
Observable.range(4, 1)
  .concatMap((i) =>
    new Observable.timer(i, new Duration(minutes: i))
  .listen(print); // prints 4, 3, 2, 1
```

#### concatWith()

Returns an Observable that emits all items from the current Observable, then emits all items from the given observable, one after the next.

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

#### debounce()

Transforms a `Stream` so that will only emit items from the source sequence if a `window` has completed, without the source sequence emitting another item.

This `window` is created after the last debounced event was emitted. You can use the value of the last debounced event to determine the length of the next `window`.

A `window` is open until the first `window` event emits.

debounce filters out items emitted by the source [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) that are rapidly followed by another emitted item.

[Interactive marble diagram](http://rxmarbles.com/#debounce)

Example

```dart
new Observable.fromIterable([1, 2, 3, 4])
  .debounce((_) => TimerStream(true, const Duration(seconds: 1)))
  .listen(print); // prints 4
```

#### debounceTime()

Transforms a `Stream` so that will only emit items from the source sequence whenever the time span defined by `duration` passes, without the source sequence emitting another item.

This time span start after the last debounced event was emitted.

debounceTime filters out items emitted by the source [Observable](https://pub.dev/documentation/rxdart/latest/rx/Observable-class.html) that are rapidly followed by another emitted item.

[Interactive marble diagram](http://rxmarbles.com/#debounce)

Example

```dart
new Observable.fromIterable([1, 2, 3, 4])
  .debounceTime(const Duration(seconds: 1))
  .listen(print); // prints 4
```

#### defaultIfEmpty()

Emit items from the source Stream, or a single default item if the source Stream emits nothing.

Example

```dart
new Observable.empty().defaultIfEmpty(10).listen(print); // prints 10
```

#### delay()

The Delay operator modifies its source Observable by pausing for a particular increment of time (that you specify) before emitting each of the source Observable’s items. This has the effect of shifting the entire sequence of items emitted by the Observable forward in time by that specified increment.

[Interactive marble diagram](http://rxmarbles.com/#delay)

Example

```dart
new Observable.fromIterable([1, 2, 3, 4])
  .delay(new Duration(seconds: 1))
  .listen(print); // [after one second delay] prints 1, 2, 3, 4 immediately
```

#### dematerialize<S>()

Converts the onData, onDone, and onError [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects from a materialized stream into normal onData, onDone, and onError events.

When a stream has been materialized, it emits onData, onDone, and onError events as [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects. Dematerialize simply reverses this by transforming [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects back to a normal stream of events.

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

The returned stream provides the same events as this stream, except that it never provides two consecutive data events that are equal.

Equality is determined by the provided equals method. If that is omitted, the '==' operator on the last provided data element is used.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the equals test.

[Interactive marble diagram](http://rxmarbles.com/#distinctUntilChanged)

#### distinctUnique()

WARNING: More commonly known as distinct in other Rx implementations. Creates an Observable where data events are skipped if they have already been emitted before.

Equality is determined by the provided equals and hashCode methods. If these are omitted, the '==' operator and hashCode on the last provided data element are used.

The returned stream is a broadcast stream if this stream is. If a broadcast stream is listened to more than once, each subscription will individually perform the equals and hashCode tests.

[Interactive marble diagram](http://rxmarbles.com/#distinct)

#### doOnCancel()

Invokes the given callback function when the stream subscription is cancelled. Often called doOnUnsubscribe or doOnDispose in other implementations.

Example

```dart
final subscription = new Observable.timer(1, new Duration(minutes: 1))
  .doOnCancel(() => print("hi"));
  .listen(null);

subscription.cancel(); // prints "hi"
```

#### doOnData()

Invokes the given callback function when the stream emits an item. In other implementations, this is called doOnNext.

Example

```dart
new Observable.fromIterable([1, 2, 3])
  .doOnData(print)
  .listen(null); // prints 1, 2, 3
```

#### doOnDone()

Invokes the given callback function when the stream finishes emitting items. In other implementations, this is called doOnComplete(d).

Example

```dart
new Observable.fromIterable([1, 2, 3])
  .doOnDone(() => print("all set"))
  .listen(null); // prints "all set"
```

#### doOnEach()

Invokes the given callback function when the stream emits data, emits an error, or emits done. The callback receives a [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) object.

The [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) object contains the [Kind](https://pub.dev/documentation/rxdart/latest/rx/Kind-class.html) of event (OnData, onDone, or OnError), and the item or error that was emitted. In the case of onDone, no data is emitted as part of the [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html).

Example

```dart
new Observable.just(1)
  .doOnEach(print)
  .listen(null); // prints Notification{kind: OnData, value: 1, errorAndStackTrace: null}, Notification{kind: OnDone, value: null, errorAndStackTrace: null}
```

#### doOnError()

Invokes the given callback function when the stream emits an error.

Example

```dart
new Observable.error(new Exception())
  .doOnError((error, stacktrace) => print("oh no"))
  .listen(null); // prints "Oh no"
```

#### doOnListen()

Invokes the given callback function when the stream is first listened to.

Example

```dart
new Observable.just(1)
  .doOnListen(() => print("Is someone there?"))
  .listen(null); // prints "Is someone there?"
```

#### doOnPause()

Invokes the given callback function when the stream subscription is paused.

Example

```dart
final subscription = new Observable.just(1)
  .doOnPause(() => print("Gimme a minute please"))
  .listen(null);

subscription.pause(); // prints "Gimme a minute please"
```

#### doOnResume()

Invokes the given callback function when the stream subscription resumes receiving items.

Example

```dart
final subscription = new Observable.just(1)
  .doOnResume(() => print("Let's do this!"))
  .listen(null);

subscription.pause();
subscription.resume(); "Let's do this!"
```

#### drain<S>()

Discards all data on this stream, but signals when it is done or an error occurred.

When subscribing using [drain](https://pub.dev/documentation/rxdart/latest/rx/Observable/drain.html), cancelOnError will be true. This means that the future will complete with the first error on this stream and then cancel the subscription. If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

In case of a `done` event the future completes with the given `futureValue`.

#### elementAt()

Returns the value of the `index`th data event of this stream.

Stops listening to this stream after the `index`th data event has been received.

Internally the method cancels its subscription after these elements. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

If an error event occurs before the value is found, the future completes with this error.

If a done event occurs before the value is found, the future completes with a `RangeError`.

#### every()

Checks whether `test` accepts all elements provided by this stream.

Calls `test` on each element of this stream. If the call returns `false`, the returned future is completed with `false` and processing stops.

If this stream ends without finding an element that `test` rejects, the returned future is completed with `true`.

If this stream emits an error, or if the call to `test` throws, the returned future is completed with that error, and processing stops.

#### exhaustMap<S>()

Converts items from the source stream into a new Stream using a given mapper. It ignores all items from the source stream until the new stream completes.

Useful when you have a noisy source Stream and only want to respond once the previous async operation is finished.

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

#### firstWhere()

Finds the first element of this stream matching `test`.

Returns a future that is completed with the first element of this stream that `test` returns `true` for.

If no such element is found before this stream is done, and a `orElse` function is provided, the result of calling `orElse` becomes the value of the future. If `orElse` throws, the returned future is completed with that error.

If this stream emits an error before the first matching element, the returned future is completed with that error, and processing stops.

Stops listening to this stream after the first matching element or error has been received.

Internally the method cancels its subscription after the first element that matches the predicate. This means that single-subscription (non-broadcast) streams are closed and cannot be reused after a call to this method.

If an error occurs, or if this stream ends without finding a match and with no `orElse` function provided, the returned future is completed with an error.

#### flatMap<S>()

Converts each emitted item into a new Stream using the given mapper function. The newly created Stream will be be listened to and begin emitting items downstream.

The items emitted by each of the new Streams are emitted downstream in the same order they arrive. In other words, the sequences are merged together.

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

#### forEach()

Executes `action` on each element of this stream.

Completes the returned `Future` when all elements of this stream have been processed.

If this stream emits an error, or if the call to `action` throws, the returned future completes with that error, and processing stops.

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

#### ignoreElements()

Creates an Observable where all emitted items are ignored, only the error / completed notifications are passed

Example

```
new Observable.merge(
	new Observable.just(1),
	new Observable.error(new Exception())
)
.listen(print, onError: print); // prints Exception
```

#### interval()

Creates an Observable that emits each item in the Stream after a given duration.

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

#### lastWhere()

Finds the last element in this stream matching `test`.

If this stream emits an error, the returned future is completed with that error, and processing stops.

Otherwise as [firstWhere](https://pub.dev/documentation/rxdart/latest/rx/Observable/firstWhere.html), except that the last matching element is found instead of the first. That means that a non-error result cannot be provided before this stream is done.

#### listen()

Adds a subscription to this stream. Returns a `StreamSubscription` which handles events from the stream using the provided `onData`, `onError` and `onDone` handlers.

The handlers can be changed on the subscription, but they start out as the provided functions.

On each data event from this stream, the subscriber's `onData` handler is called. If `onData` is `null`, nothing happens.

On errors from this stream, the `onError` handler is called with the error object and possibly a stack trace.

The `onError` callback must be of type `void onError(error)` or `void onError(error, StackTrace stackTrace)`. If `onError` accepts two arguments it is called with the error object and the stack trace (which could be `null` if the stream itself received an error without stack trace). Otherwise it is called with just the error object. If `onError` is omitted, any errors on the stream are considered unhandled, and will be passed to the current `Zone`'s error handler. By default unhandled async errors are treated as if they were uncaught top-level errors.

If this stream closes and sends a done event, the `onDone` handler is called. If `onDone` is `null`, nothing happens.

If `cancelOnError` is true, the subscription is automatically cancelled when the first error event is delivered. The default is `false`.

While a subscription is paused, or when it has been cancelled, the subscription doesn't receive events and none of the event handler functions are called.

Example

```dart
new Observable.just(1).listen(print); // prints 1
```

#### map<S>()

Maps values from a source sequence through a function and emits the returned values.

The returned sequence completes when the source sequence completes. The returned sequence throws an error if the source sequence throws an error.

#### mapTo<S>()

Emits the given constant value on the output Observable every time the source Observable emits a value.

Example

```dart
Observable.fromIterable([1, 2, 3, 4])
  .mapTo(true)
  .listen(print); // prints tr
```

#### materialize()

Converts the onData, on Done, and onError events into [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) objects that are passed into the downstream onData listener.

The [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html) object contains the [Kind](https://pub.dev/documentation/rxdart/latest/rx/Kind-class.html) of event (OnData, onDone, or OnError), and the item or error that was emitted. In the case of onDone, no data is emitted as part of the [Notification](https://pub.dev/documentation/rxdart/latest/rx/Notification-class.html).

Example:     new Observable.just(1)         .materialize()         .listen((i) => print(i)); // Prints onData & onDone Notification

```dart
new Observable<int>.error(new Exception())
    .materialize()
    .listen((i) => print(i)); // Prints onError Notification
```

#### max()

Converts a Stream into a Future that completes with the largest item emitted by the Stream.

This is similar to finding the max value in a list, but the values are asynchronous.

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

Example

```dart
new Observable.timer(1, new Duration(seconds: 10))
    .mergeWith([new Observable.just(2)])
    .listen(print); // prints 2, 1
```

#### min()

Converts a Stream into a Future that completes with the smallest item emitted by the Stream.

This is similar to finding the min value in a list, but the values are asynchronous!

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

Examples

```dart
new Observable.fromIterable([1, "hi"])
  .ofType(new TypeToken<String>)
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

#### onAdd()

An extension point for sub-classes. Perform any side-effect / state
management you need to here, rather than overriding the `add` method
directly.

#### onAddError()

An extension point for sub-classes. Perform any side-effect / state
management you need to here, rather than overriding the `add` method
directly.

#### onErrorResume()

Intercepts error events and switches to a recovery stream created by the provided `recoveryFn`.

The onErrorResume operator intercepts an onError notification from the source Observable. Instead of passing the error through to any listeners, it replaces it with another Stream of items created by the `recoveryFn`.

The `recoveryFn` receives the emitted error and returns a Stream. You can perform logic in the `recoveryFn` to return different Streams based on the type of error that was emitted.

If you do not need to perform logic based on the type of error that was emitted, please consider using [onErrorResumeNext](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorResumeNext.html) or [onErrorReturn](https://pub.dev/documentation/rxdart/latest/rx/Observable/onErrorReturn.html).

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

Example

```dart
new Observable.error(new Exception())
  .onErrorReturnWith((e) => e is Exception ? 1 : 0)
  .listen(print); // prints 1
```

#### pairwise()

Emits the n-th and n-1th events as a pair..

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

#### publish()

Convert the current Observable into a [ConnectableObservable](https://pub.dev/documentation/rxdart/latest/rx/ConnectableObservable-class.html) that can be listened to multiple times. It will not begin emitting items from the original Observable until the `connect` method is invoked.

This is useful for converting a single-subscription stream into a broadcast Stream.

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

#### reduce

Combines a sequence of values by repeatedly applying `combine`.

Similar to [Iterable.reduce](https://pub.dev/documentation/rxdart/latest/rx/Observable/reduce.html), this function maintains a value, starting with the first element of this stream and updated for each further element of this stream. For each element after the first, the value is updated to the result of calling `combine` with the previous value and the element.

When this stream is done, the returned future is completed with the value at that time.

If this stream is empty, the returned future is completed with an error. If this stream emits an error, or the call to `combine` throws, the returned future is completed with that error, and processing is stopped.

#### sample()

Emits the most recently emitted item (if any) emitted by the source `Stream` since the previous emission from the `sampleStream`.

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .sample(new TimerStream(1, const Duration(seconds: 1)))
  .listen(print); // prints 3
```

#### sampleTime()

Emits the most recently emitted item (if any) emitted by the source `Stream` since the previous emission within the recurring time span, defined by `duration`

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .sampleTime(const Duration(seconds: 1))
  .listen(print); // prints 3
```

#### scan<S>()

Applies an accumulator function over an observable sequence and returns each intermediate result. The optional seed value is used as the initial accumulator value.

Example

```dart
new Observable.fromIterable([1, 2, 3])
   .scan((acc, curr, i) => acc + curr, 0)
   .listen(print); // prints 1, 3, 6
```

#### share()

Convert the current Observable into a new Observable that can be listened to multiple times. It will automatically begin emitting items when first listened to, and shut down when no listeners remain.

This is useful for converting a single-subscription stream into a broadcast Stream.

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

#### skip()

Skips the first count data events from this stream.

The returned stream is a broadcast stream if this stream is. For a broadcast stream, the events are only counted from the time the returned stream is listened to.

#### skipUntil<S>()

Starts emitting items only after the given stream emits an item.

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

#### startWith()

Prepends a value to the source Observable.

Example

```dart
new Observable.just(2).startWith(1).listen(print); // prints 1, 2
```

#### startWithMany()

Prepends a sequence of values to the source Observable.

Example

```dart
new Observable.just(3).startWithMany([1, 2])
  .listen(print); // prints 1, 2, 3
```

#### switchIfEmpty()

When the original observable emits no items, this operator subscribes to the given fallback stream and emits items from that observable instead.

This can be particularly useful when consuming data from multiple sources. For example, when using the Repository Pattern. Assuming you have some data you need to load, you might want to start with the fastest access point and keep falling back to the slowest point. For example, first query an in-memory database, then a database on the file system, then a network call if the data isn't on the local machine.

This can be achieved quite simply with switchIfEmpty!

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

#### takeUntil<S>()

Returns the values from the source observable sequence until the other observable sequence produces a value.

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

#### throttle()

Emits only the first item emitted by the source `Stream` while `window` is open.

if `trailing` is true, then the last item is emitted instead

You can use the value of the last throttled event to determine the length of the next `window`.

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .throttle((_) => TimerStream(true, const Duration(seconds: 1)))
```

#### throttleTime()

Emits only the first item emitted by the source `Stream` within a time span of `duration`.

if `trailing` is true, then the last item is emitted instead

Example

```dart
new Stream.fromIterable([1, 2, 3])
  .throttleTime(const Duration(seconds: 1))
```

#### timeInterval()

Records the time interval between consecutive values in an observable sequence.

Example

```dart
new Observable.just(1)
  .interval(new Duration(seconds: 1))
  .timeInterval()
  .listen(print); // prints TimeInterval{interval: 0:00:01
```

#### timeout()

The Timeout operator allows you to abort an Observable with an onError
termination if that Observable fails to emit any items during a specified
duration.  You may optionally provide a callback function to execute on
timeout.

#### timestamp()

Wraps each item emitted by the source Observable in a [Timestamped](https://pub.dev/documentation/rxdart/latest/rx/Timestamped-class.html) object that includes the emitted item and the time when the item was emitted.

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

```dart
stream.map(mapping).transform(transformation).toList()
```

which can be more convenient than calling `bind` directly.

The `streamTransformer` can return any stream. Whether the returned stream is a broadcast stream or not, and which elements it will contain, is entirely up to the transformation.

This method should always be used for transformations which treat the entire stream as representing a single value which has perhaps been split into several parts for transport, like a file being read from disk or being fetched over a network. The transformation will then produce a new stream which transforms the stream's value incrementally (perhaps using `Converter.startChunkedConversion`). The resulting stream may again be chunks of the result, but does not have to correspond to specific events from the source string.

#### where()

Filters the elements of an observable sequence based on the test.

#### window()

Creates an Observable where each item is a `Stream` containing the items from the source sequence.

This `List` is emitted every time `window` emits an event.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (i) => i)
  .window(new Stream.periodic(const Duration(milliseconds: 160), (i) => i))
  .asyncMap((stream) => stream.toList())
  .listen(print); // prints [0, 1] [2, 3] [4, 5] ...
```

#### windowCount()

Buffers a number of values from the source Observable by `count` then emits the buffer as a `Stream` and clears it, and starts a new buffer each `startBufferEvery` values. If `startBufferEvery` is not provided, then new buffers are started immediately at the start of the source and when each buffer closes and is emitted.

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
  .listen(print); // prints [1, 2, 3], 
```

#### windowTest()

Creates an Observable where each item is a `Stream` containing the items from the source sequence, batched whenever test passes.

Example

```dart
new Observable.periodic(const Duration(milliseconds: 100), (int i) => i)
  .windowTest((i) => i % 2 == 0)
  .asyncMap((stream) => stream.toList())
  .listen(print); // prints [0], [1, 2] [3, 4] [5, 6] ...
```

#### windowTime()

Creates an Observable where each item is a `Stream` containing the items from the source sequence, sampled on a time frame with `duration`.

Example

```dart
new Observable.periodic(
    const Duration(milliseconds: 100),
    (int i) => i
)
.windowTime(const Duration(milliseconds: 220))
.doOnData((_) => print('next window'))
.flatMap((s) => s)
.listen(print); // prints next window 0, 1, next window 2, 3, ...
```

#### withLatestFrom<S, R>()

Creates an Observable that emits when the source stream emits, combining the latest values from the two streams using the provided function.

If the latestFromStream has not emitted any values, this stream will not emit either.

[Interactive marble diagram](http://rxmarbles.com/#withLatestFrom)

Example

```dart
new Observable.fromIterable([1, 2]).withLatestFrom(
  new Observable.fromIterable([2, 3]), (a, b) => a + b)
  .listen(print); // prints 4 (due to the async nature of streams)
```

#### zipWith<S, R>()

Returns an Observable that combines the current stream together with another stream using a given zipper function.

Example

```dart
new Observable.just(1)
    .zipWith(new Observable.just(2), (one, two) => one + two)
    .listen(print); // prints 3
```
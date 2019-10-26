# ReplaySubject<T>

A special StreamController that captures all of the items that have been added to the controller, and emits those as the first items to any new listener.

This subject allows sending data, error and done events to the listener. As items are added to the subject, the ReplaySubject will store them. When the stream is listened to, those recorded items will be emitted to the listener. After that, any new events will be appropriately sent to the listeners. It is possible to cap the number of stored events by setting a maxSize value.

ReplaySubject is, by default, a broadcast (aka hot) controller, in order to fulfill the Rx Subject contract. This means the Subject's `stream` can be listened to multiple times.

컨트롤러에 추가 된 모든 항목을 캡처하고 새 리스너의 첫 번째 항목으로 해당 항목을 내보내는 특수 StreamController입니다.

이 주제는 데이터, 오류 및 완료 이벤트를 청취자에게 전송할  수있게 합니다. 제목에 항목이 추가되면 ReplaySubject가 항목을 저장합니다. 스트림을 청취하면 녹음 된 항목이 청취자에게 방출됩니다. 그 후 새로운 이벤트가 청취자에게 적절하게 전송됩니다. maxSize 값을 설정하여 저장된 이벤트의 수를 제한 할 수 있습니다.

ReplaySubject는 기본적으로 Rx Subject 계약을 이행하기 위해 브로드 캐스트 (일명 핫) 컨트롤러입니다. 이것은 Subject의`stream`을 여러 번 듣게된다는 것을 의미합니다.

### Example

```dart
final subject = new ReplaySubject<int>();

subject.add(1);
subject.add(2);
subject.add(3);

subject.stream.listen(print); // prints 1, 2, 3
subject.stream.listen(print); // prints 1, 2, 3
subject.stream.listen(print); // prints 1, 2, 3
```

### Example with maxSize

```dart
final subject = new ReplaySubject<int>(maxSize: 2);

subject.add(1);
subject.add(2);
subject.add(3);

subject.stream.listen(print); // prints 2, 3
subject.stream.listen(print); // prints 2, 3
subject.stream.listen(print); // prints 2, 3
```
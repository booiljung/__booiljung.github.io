# ReactiveX (RxDart)

많은 상태 관리와 중복 코드로 인해, 2005년쯤 C++로 ReactiveX와 유사한 Mocha를 구현해본 적이 있습니다. 당시 ReactiveX의 존재를 몰랐고, 201O 몇년쯤에 존재를 알았습니다.  Mocha와 Rx의 차이점은 Mocha는 3D 렌더링을 위해 구현하였으므로 시스템 자원에 크게 개의치 않고 무자비하게 CPU 자원을 끌어다 썼다는 점, 당시 C++이 람다 함수를 지원하지 않아, 함수가 멀리 떨어저 구현 될 수 밖에 없어, 패턴이 발생하면 함수 객체를 구현하여 재사용했다는 정도일 것입니다. 요즘 언어들은 람다함수를 지원하여 Rx 구현과 사용에 개선이 되었습니다. Mocha를 직접 만들어 Rx의 유용한점을 잘 알고 있기에 이를 정리해 봅니다.

## Future

JavaScirpt에서 Promise는 Dart에서 Future에 해당합니다. Future는 데이터를 비동기적으로 전달하기 위한 개체입니다. Rx에서 Future를 언급하는 이유는 Rx도 비동기에 기반하기 때문입니다.

TODO: 예제



## Observable

`Observable`은 `Stream`을 확장합니다. 이 라이브러리에 포함 된 모든 `Streams` 및 `StreamTransformer`를 유창한 API로 결합합니다.

### Dart Stream vs Observables

Observable 클래스는 Dart 생태계와 유창하게 통합되기 위해 Dart Stream 클래스를 확장합니다. 이것은 몇 가지 장점을 제공합니다:

- Observables는 Dart Stream을 입력으로 기대하는 모든 API에서 작동합니다.
- 핵심 Stream API에서 많은 메소드와 속성을 상속받습니다.
- 언어 수준 구문으로 스트림을 생성 할 수 있습니다.

전반적으로 우리는 Observable 스펙을 가능한 한 가깝게 따르려고하지만, 절충안을 작성해야하는 경우 Dart 생태계에 우선 순위를 부여합니다. 따라서 Dart 's Stream 클래스와 표준 Rx Observable 사이에는 몇 가지 중요한 차이점이 있습니다.

첫째, Dart의 Cold Observables는 단일 구독입니다. 다시 말해서 Observables가 hot (일명 broadcast) stream이 아닌 한 Observables를 한 번만 들을 수 있습니다. cold stream을 두 번 청취하려고 하면 StateError가 발생합니다. stream을 여러 번 청취해야하는 경우 stream의 새 인스턴스를 반환하는 팩토리 함수를 만들면 됩니다.

둘째로, first와 last와 같은 내부에 포함 된 많은 메소드는 Single이나 Observable을 반환하지 않지만 dart  Future를 반환해야합니다. 운좋게도 Dart Futures는 작업하기 쉽고 필요할 경우 myFuture.asStream() 메서드를 사용하여 쉽게 스트림으로 다시 변환 할 수 있습니다.

셋째, 오류가 발생하면 Dart의 stream이 기본적으로 닫히지 않습니다. Rx에서 오류는 Observable이 운영자에 의해 인터셉트되지 않는 한 종료 되도록 합니다. dart는 오류가 발생할 때 닫히는 stream을 만드는 메커니즘을 가지고 있지만, 대부분의 stream은 이 동작을 나타내지 않습니다.

넷째, Dart stream은 기본적으로 비동기이지만 Observables는 다른 Scheduler에서 작업을 예약하지 않는 한 기본적으로 동기식입니다. Dart로 동기식 stream을 만들 수 있지만 기본값이 다르다는 점에 유의하십시오.

마지막으로 Dart broadcast stream (Hot Observables와 유사)을 사용할 때 onListen은 broadcast stream을 처음 청취 할 때만 호출됩니다.


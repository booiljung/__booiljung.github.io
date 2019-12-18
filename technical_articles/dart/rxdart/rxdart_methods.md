# RxDart Method List

| Methods            | Observable | PublishSubject | BehaviorSubject | ReplaySubject |
| ------------------ | :--------: | :------------: | :-------------: | :-----------: |
| concat             |     O      |                |                 |               |
| concatEager        |     O      |                |                 |               |
| defer              |     O      |                |                 |               |
| empty              |     O      |                |                 |               |
| error              |     O      |                |                 |               |
| eventTransformed   |     O      |                |                 |               |
| fromFuture         |     O      |                |                 |               |
| fromIterable       |     O      |                |                 |               |
| just               |     O      |                |                 |               |
| merge              |     O      |                |                 |               |
| never              |     O      |                |                 |               |
| periodic           |     O      |                |                 |               |
| race               |     O      |                |                 |               |
| repeat             |     O      |                |                 |               |
| retry              |     O      |                |                 |               |
| retryWhen          |     O      |                |                 |               |
| seeded             |            |                |        O        |               |
| switchLatest       |     O      |                |                 |               |
| timer              |     O      |                |                 |               |
| controller         |            |       O        |        O        |       O       |
| done               |            |       O        |        O        |       O       |
| first              |     O      |       O        |        O        |       O       |
| hasListener        |            |       O        |        O        |       O       |
| hasValue           |            |                |        O        |               |
| isBroadcast        |     O      |       O        |        O        |       O       |
| isClosed           |            |       O        |        O        |       O       |
| isEmpty            |     O      |       O        |        O        |       O       |
| isPaused           |            |       O        |        O        |       O       |
| last               |     O      |       O        |        O        |       O       |
| length             |     O      |       O        |        O        |       O       |
| onCancel           |            |       O        |        O        |       O       |
| onListen           |            |       O        |        O        |       O       |
| onPause            |            |       O        |        O        |       O       |
| single             |     O      |       O        |        O        |       O       |
| sink               |            |       O        |        O        |       O       |
| stream             |            |       O        |        O        |       O       |
| value              |            |                |        O        |       O       |
| add                |            |       O        |        O        |       O       |
| addError           |            |       O        |        O        |       O       |
| addStream          |            |       O        |        O        |       O       |
| any                |     O      |       O        |        O        |       O       |
| asBroadcastStream  |     O      |       O        |        O        |       O       |
| asyncExpand        |     O      |       O        |        O        |       O       |
| asyncMap           |     O      |       O        |        O        |       O       |
| buffer             |     O      |       O        |        O        |       O       |
| bufferCount        |     O      |       O        |        O        |       O       |
| bufferTest         |     O      |       O        |        O        |       O       |
| bufferTime         |     O      |       O        |        O        |       O       |
| cast               |     O      |       O        |        O        |       O       |
| close              |            |       O        |        O        |       O       |
| concatMap          |     O      |       O        |        O        |       O       |
| concatWith         |     O      |       O        |        O        |       O       |
| contains           |     O      |       O        |        O        |       O       |
| debounce           |     O      |       O        |        O        |       O       |
| debounceTime       |     O      |       O        |        O        |       O       |
| defaultIfEmpty     |     O      |       O        |        O        |       O       |
| delay              |     O      |       O        |        O        |       O       |
| dematerialize      |     O      |       O        |        O        |       O       |
| distinct           |     O      |       O        |        O        |       O       |
| distinctUnique     |     O      |       O        |        O        |       O       |
| doOneCancel        |     O      |       O        |        O        |       O       |
| doOnData           |     O      |       O        |        O        |       O       |
| doOnDone           |     O      |       O        |        O        |       O       |
| doOnEach           |     O      |       O        |        O        |       O       |
| doOnError          |     O      |       O        |        O        |       O       |
| doOnListen         |     O      |       O        |        O        |       O       |
| doOnPause          |     O      |       O        |        O        |       O       |
| doOnResume         |     O      |       O        |        O        |       O       |
| drain              |     O      |       O        |        O        |       O       |
| elementAt          |     O      |       O        |        O        |       O       |
| every              |     O      |       O        |        O        |       O       |
| exhaustMap         |     O      |       O        |        O        |       O       |
| expand             |     O      |       O        |        O        |       O       |
| firstWhere         |     O      |       O        |        O        |       O       |
| flatMap            |     O      |       O        |        O        |       O       |
| flatMapIterable    |     O      |       O        |        O        |       O       |
| fold               |     O      |       O        |        O        |       O       |
| forEach            |     O      |       O        |        O        |       O       |
| groupBy            |     O      |       O        |        O        |       O       |
| handleError        |     O      |       O        |        O        |       O       |
| ignoreElements     |     O      |       O        |        O        |       O       |
| interval           |     O      |       O        |        O        |       O       |
| join               |     O      |       O        |        O        |       O       |
| lastWhere          |     O      |       O        |        O        |       O       |
| listen             |     O      |       O        |        O        |       O       |
| map                |     O      |       O        |        O        |       O       |
| mapTo              |     O      |       O        |        O        |       O       |
| materialize        |     O      |       O        |        O        |       O       |
| max                |     O      |       O        |        O        |       O       |
| mergeWith          |     O      |       O        |        O        |       O       |
| min                |     O      |       O        |        O        |       O       |
| ofType             |     O      |       O        |        O        |       O       |
| onAdd              |            |       O        |        O        |       O       |
| onAddError         |            |       O        |        O        |       O       |
| onErrorResume      |     O      |       O        |        O        |       O       |
| onErrorResumeNext  |     O      |       O        |        O        |       O       |
| onErrorReturn      |     O      |       O        |        O        |       O       |
| onErrorReturnWith  |     O      |       O        |        O        |       O       |
| pairwise           |     O      |       O        |        O        |       O       |
| pipe               |     O      |       O        |        O        |       O       |
| publish            |     O      |       O        |        O        |       O       |
| publishReplay      |     O      |       O        |        O        |       O       |
| publishValue       |     O      |       O        |        O        |       O       |
| publishValueSeeded |     O      |       O        |        O        |       O       |
| reduce             |     O      |       O        |        O        |       O       |
| sample             |     O      |       O        |        O        |       O       |
| sampleTime         |     O      |       O        |        O        |       O       |
| scan               |     O      |       O        |        O        |       O       |
| share              |     O      |       O        |        O        |       O       |
| shareReplay        |     O      |       O        |        O        |       O       |
| shareValue         |     O      |       O        |        O        |       O       |
| shareValueSeeded   |     O      |       O        |        O        |       O       |
| singleWhere        |     O      |       O        |        O        |       O       |
| skip               |     O      |       O        |        O        |       O       |
| skipUntil          |     O      |       O        |        O        |       O       |
| skipWhile          |     O      |       O        |        O        |       O       |
| startWith          |     O      |       O        |        O        |       O       |
| startWithMany      |     O      |       O        |        O        |       O       |
| switchIfEmpty      |     O      |       O        |        O        |       O       |
| switchMap          |     O      |       O        |        O        |       O       |
| take               |     O      |       O        |        O        |       O       |
| takeUntil          |     O      |       O        |        O        |       O       |
| takeWhile          |     O      |       O        |        O        | isBroadcastO  |
| throttle           |     O      |       O        |        O        |       O       |
| throttleTime       |     O      |       O        |        O        |       O       |
| timeInterval       |     O      |       O        |        O        |       O       |
| timeout            |     O      |       O        |        O        |       O       |
| timestamp          |     O      |       O        |        O        |       O       |
| toList             |     O      |       O        |        O        |       O       |
| toSet              |     O      |       O        |        O        |       O       |
| transform<S>       |     O      |       O        |        O        |       O       |
| where              |     O      |       O        |        O        |       O       |
| window             |     O      |       O        |        O        |       O       |
| windowCount        |     O      |       O        |        O        |       O       |
| windowTest         |     O      |       O        |        O        |       O       |
| windowTime         |     O      |       O        |        O        |       O       |
| withLastestFrom    |     O      |       O        |        O        |       O       |
| zipWith            |     O      |       O        |        O        |       O       |


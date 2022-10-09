[Up](../index.md)

# TypeScript

https://joshua1988.github.io/ts/guide/basic-types.html#array 를 참고하여 요약

타입스크립트란?

- 자바스크립트에 타입을 부여한 언어.
- 자바스크립트이 수퍼셋.

장점:

- 빌드 타임에 오류 체크
- 코드 가이드 및 자동 완성

## 타입

### 기본 타입

타입스크립트의 기본 타입은 12가지가 있다.

#### string:

문자열

```ts
let str: string = 'hi';
```

#### number:

숫자

```ts
let num: number = 10;
```

#### boolean:

진위

```ts
let isLoggedIn: boolean = false;
```

### 객체 타입

#### array:

```ts
let arr: number[] = [1,2,3];
```

제네릭 지원:

```ts
let arr: Array<number> = [1,2,3];
```

#### tuple:

각 요소의 타입이 지정되어 있는 고정 길이 배열

```ts
let arr: [string, number] = ['hi', 10];
```

정의하지 않은 타입 및 인덱스로 접근하면 오류

```ts
arr[1].concat('!'); // Error, 'number' does not have 'concat'
arr[5] = 'hello'; // Error, Property '5' does not exist on type '[string, number]'.
```

#### enum:

상수의 집합

```ts
enum Avengers { Capt, IronMan, Thor }
let hero: Avengers = Avengers.Capt;
```

인덱스 번호로도 접근 가능

```ts
enum Avengers { Capt, IronMan, Thor }
let hero: Avengers = Avengers[0];
```

인덱스 변경도 가능

```ts
enum Avengers { Capt = 2, IronMan, Thor }
let hero: Avengers = Avengers[2]; // Capt
let hero: Avengers = Avengers[4]; // Thor
```

#### any:

모든 타입 허용

```ts
let str: any = 'hi';
let num: any = 10;
let arr: any = ['a', 2, true];
```

#### void:

```ts
let unuseful: void = undefined;
function notuse(): void {
  console.log('sth');
}
```

#### never:

함수의 끝에 도달하지 않음

```ts
// 이 함수는 절대 함수의 끝까지 실행되지 않는다는 의미
function neverEnd(): never {
  while (true) {
  }
}
```

## 함수

3가지 타입을 정의할 수 있습니다.

- 함수의 매개변수 타입
- 함수의 반환 타입
- 함수의 구조 타입

### 함수에서 타입

타입 스크립트의 경우 매개변수와 반환값에 타입을 지정해야 한다:

```ts
function sum(a: number, b: number): number {
  return a + b;
}
```

반환값이 없는 경우`void`를 반환 타입으로 지정해야 한다:

```ts
function noresult(): void {
  console.log('noresult called');
}
```

### 함수의 인자

타입스크립트에서는 함수의 인자를 모두 필수 값으로 간주. 함수의 매개변수에는 `undefined`나 `null`이라도 인자로 넘겨야 한다. 컴파일러는 정의된 매개변수 값이 넘어 왔는지 체크한다:

```ts
function sum(a: number, b: number): number {
  return a + b;
}
sum(10, 20); // 결과: 30
sum(10, 20, 30); // error, too many parameters
sum(10); // error, too few parameters
```

정의된 매개변수 만큼 인자를 넘기지 않아도 되는 자바스크립트와 다릅니다. 인자를 선택적으로 전달하고 싶다면 `?`를 지정할 수 있다:

```ts
function sum(a: number, b?: number): number {
  return a + b;
}
sum(10, 20); // 30
sum(10, 20, 30); // error, too many parameters
sum(10); // 10
```

매개변수 초기화는 ES6와 같다:

```ts
function sum(a: number, b = 100): number {
  return a + b;
}
sum(10, undefined); // 110
sum(10, 20, 30); // error, too many parameters
sum(10); // 110
```

### 매개변수에 REST 문법 적용

ES6에서 Spread 문법은 배열로된 하나의 인자를 받는다.

```js
function sum(x, y, z) {
  return x + y + z;
}
const numbers = [1, 2, 3];
sum(...numbers); // expected output: 6
sum.apply(null, numbers); // expected output: 6
```

기존에 두 배열을 합치려면, `push`, `splice`, `concat`를 사용해야 했다. Spread 문법을 사용하면 두 배열을 간단히 합치 거나 복사가 가능하다:

```js
var parts = [ 'shoulders', 'knees' ]; 
var lyrics = [ 'head', ...parts, 'and', 'toes' ];
// lyrcis = [ "head", "shoulders", "knees", "and", "toes" ]
```

ES6에서 Rest문법은 Spread와 비슷하지만 여러 인자들을 하나의 배열로 반환합니다. 함수의 지역인자 arguments와 Rest의 다른 점은 arguments는 모든 인자를 포함하며 유사배열입니다.

```js
function sum(...theArgs) {
  return theArgs.reduce((previous, current) => {
    return previous + current;
  });
}
sum(1, 2, 3); // expected output: 6
sum(1, 2, 3, 4); // expected output: 10
```

타입스크립트에서 Rest 문법은 아래와 같이 사용할 수 있습니다:

```ts
function sum(a: number, ...nums: number[]): number {
  const totalOfNums = 0;
  for (let key in nums) {
    totalOfNums += nums[key];
  }
  return a + totalOfNums;
}
```

### `this`

자바스크립트와 다르게 `this`의 잘못된 사용을 탐지 할 수 있다. 타입스크립트에서 `this`가 가리키는 것을 명시하려면 아래와 같이 사용한다:

```ts
function 함수명(this: 타입) {
  // ...
}
```

예를 들면:

```ts
interface Vue {
  el: string;
  count: number;
  init(this: Vue): () => {};
}

let vm: Vue = {
  el: '#app',
  count: 10,
  init: function(this: Vue) {
    return () => {
      return this.count;
    }
  }
}

let getCount = vm.init();
let count = getCount();
console.log(count); // expected output: 10
```

위의 코드를 타입스크립트로 컴파일 했을 때 만일 `--noImplicitThis` 옵션이 있더라도 에러가 발생하지 않는다.

### 콜백에서의 `this`

일반적인 상황에서의 `this`와 다르게 콜백으로 함수가 전달되었을 때의 `this`를 구분해 주어야 하며 강제를 해야 합니다:

```ts
interface UIElement {
  // onclick 매개변수 `this: void`는 `this`를 사용할 수 없습니다.
  addClickListener(onclick: (this: void, e: Event) => void): void;
}

class UIHandler {
    info: string;
    onClick(this: Handler, e: Event) {
        // onclick 매개변수 `this`는 `void`인데 `this`를 사용하여 오류입니다.
        this.info = e.message;
    }
}
let handler = new UIHandler();
uiElement.addClickListener(handler.onClick); // error!
```

`UIElement` 인터페이스에 따라 `Handler`를 구현하면 아래와 같습니다.

```ts
class UIHandler {
    info: string;
    onClick(this: void, e: Event) {
        // onclick 매개변수 `this: void`는 `this`를 사용할 수 없습니다.
        console.log('clicked!');
    }
}
let handler = new UIHandler();
uiElement.addClickListener(handler.onClick);
```

## 인터페이스

인터페이스는 규격이며 다음과 같은 규격을 정의 할 수 있습니다.

- 객체의 스펙 (시그니처, 속성 타입)
- 함수의 스펙 (시그니쳐, 매개변수 타입, 반환 타입)
- 배열과 객체에 대한 접근 방식
- 클래스

### 간단한 인터페이스의 예:

자바스크립트에서는 아래처럼:

```js
let person = { name: 'Tom', age: 12 };

function logAge(obj: { age: number }) {
  console.log(obj.age); // 12
}
logAge(person); // 12
```

`logAge`는 반드시 `age` 속성을 포함해야 합니다. 그렇지 않으면 런타임에 오류가 발생할 것입니다.

타입스크립트에서는 인터페이스를 제공하여 컴파일시 오류를 검증할 수 있습니다:

```ts
interface personAge {
  age: number;
}

function logAge(obj: personAge) {
  console.log(obj.age);
}

let person = { name: 'Tom', age: 12 };
logAge(person);
```

### 인터페이스에서 옵션 속성

인터페이스를 사용할때 모든 속성에 대해 값을 주지 않아도 됩니다:

```ts
interface Meat {
  name: string;
  salt?: number;  
}

let myMeat = {
  name: 'Beef'
};
function eatMeat(meat: Meat) {
  console.log(meat.name); // Beef
}
eatMeat(myMeat);
```

`Meat` 인터페이스의 속성 `salt`는 옵션 속성이므로 인자로 넘긴 `meat`에는 `salt` 속성을 주지 않아도 오류가 발생하지 않습니다.

옵션 속성은 인터페이스에 정의되어 있지 않은 속성에 대해서 인지시켜줄 수 있습니다.

```ts
interface Meat {
  name: string;
  salt?: number;  
}

let myMeat = {
  name: 'Beef'
};
function eatMeat(meat: Meat) {
  console.log(meat.weight); // `weight`는 `Meat`의 속성이 아니므로 오류가 발생합니다.
}
eatMeat(myMeat);
```

이 특성은 컴파일시 오류를 검증할 수 있게 합니다.

### 읽기 전용 속성

읽기 전용 속성은 인터페이스로 객체를 처음 생성할 때만 값을 할당하고 그 이후에는 변경할 수 없는 속성을 의미합니다. 문법은 다음과 같이 `readonly` 속성을 앞에 붙입니다.

```ts
interface Meat {
  readonly age: number;
}

let myMeat = {
  age: 4  
};

myMeat.age = 3; // `age`를 변경하려고 하면 오류가 발생합니다.
```

### 읽기 전용 배열

 `ReadonlyArray<T>` 은 읽기 전용 배열이며 선언시에만 초기자를 줄 수 있습니다:

```ts
let arr: ReadonlyArray<number> = [ 1,2,3 ];
arr.splice(0,1); // 배열 내용을 변경할 수 없으므로 오류입니다.
arr.push(4); // 배열 내용을 변경할 수 없으므로 오류입니다.
arr[0] = 100; // 배열 내용을 변경할 수 없으므로 오류입니다.
```

### 타입 체크

타입스크립트는 인터페이스를 통한 객체 선언에서 엄밀한 속성 검사를 합니다:

```ts
interface Meat {
  age?: number;
}

function eatMeat(meat: Meat) {
  // ..
}

eatMeat({ aze: 20 }); // `aze`는 `Meat`의 속성이 아니므로 오류가 발생 합니다.
```

타입 추론을 무시하려면 `as` 를 사용합니다.

```ts
let myMeat = {
    aze: 20
};
eatMeat(myMeat as Meat);
```

인터페이스 정의하지 않은 속성들을 추가로 사용하려면 아래와 같습니다.

```ts
interface Meat {
  age?: number;
  [프로퍼티이름: 타입]: any;
}
```

### 함수 타입

[Up](../index.md)

# TypeScript

타입스크립트란?

- 자바스크립트에 타입을 부여한 언어.
- 자바스크립트의 수퍼셋.

장점:

- 빌드 타임에 오류 체크
- 코드 가이드 및 자동 완성

## 타입:

기본 타입은 12가지

- Boolean
- Number
- String
- Object
- Array
- Tuple
- Enum
- Any
- Void
- Null
- Undefined
- Never

#### string 문자열:

```ts
let str: string = 'hi';
```

#### number 숫자:

```ts
let num: number = 10;
```

#### boolean 진위:

```ts
let isLoggedIn: boolean = false;
```

#### array 배열:

```ts
let arr: number[] = [1,2,3];
```

제네릭:

```ts
let arr: Array<number> = [1,2,3];
```

#### tuple 튜블

각 요소의 타입이 지정되어 있는 고정 길이 배열:

```ts
let arr: [string, number] = ['hi', 10];
```

정의하지 않은 타입 및 인덱스로 접근하면 오류:

```ts
arr[1].concat('!'); // Error, 'number' does not have 'concat'
arr[5] = 'hello'; // Error, Property '5' does not exist on type '[string, number]'.
```

#### enum 열거형:

```ts
enum Avengers { Capt, IronMan, Thor }
let hero: Avengers = Avengers.Capt;
```

인덱스 번호로 접근:

```ts
enum Avengers { Capt, IronMan, Thor }
let hero: Avengers = Avengers[0];
```

인덱스 변경:

```ts
enum Avengers { Capt = 2, IronMan, Thor }
let hero: Avengers = Avengers[2]; // Capt
let hero: Avengers = Avengers[4]; // Thor
```

#### any 모든 타입 허용:

```ts
let str: any = 'hi';
let num: any = 10;
let arr: any = ['a', 2, true];
```

#### void:

- `undefined`와 `null`만 할당.

- 함수 반환값 없음.

```ts
let unuseful: void = undefined;
function notuse(): void {
  console.log('sth');
}
```

#### never 함수의 끝에 도달하지 않음

```ts
// 이 함수는 절대 함수의 끝까지 실행되지 않는다는 의미
function neverEnd(): never {
  while (true) {
  }
}
```

## 함수:

3가지 타입:

- 함수의 매개변수 타입
- 함수의 반환 타입
- 함수의 구조 타입

### 함수에서 타입

매개변수와 반환값의 타입 지정:

```ts
function sum(a: number, b: number): number {
  return a + b;
}
```

반환값이 없는 경우`void`를 반환 타입으로 지정:

```ts
function noresult(): void {
  console.log('noresult called');
}
```

### 함수의 인자

함수의 인자를 모두 필수 값으로 간주.

함수의 매개변수에는 `undefined`나 `null`이라도 인자로 전달 필수.

컴파일러는 정의된 매개변수 값이 넘어 왔는지 체크.

```ts
function sum(a: number, b: number): number {
  return a + b;
}
sum(10, 20); // 결과: 30
sum(10, 20, 30); // error, too many parameters
sum(10); // error, too few parameters
```

인자를 선택적으로 전달하고 싶다면 `?`를 지정:

```ts
function sum(a: number, b?: number): number {
  return a + b;
}
sum(10, 20); // 30
sum(10, 20, 30); // error, too many parameters
sum(10); // 10
```

매개변수 초기화는 ES6와 동일:

```ts
function sum(a: number, b = '100'): number {
  return a + b;
}
sum(10, undefined); // 110
sum(10, 20, 30); // error, too many parameters
sum(10); // 110
```

### REST 문법이 적용된 매개변수

```js
function sum(x, y, z) {
  return x + y + z;
}
const numbers = [1, 2, 3];
sum(...numbers); // expected output: 6
sum.apply(null, numbers); // expected output: 6
```

Spread 문법으로 두 배열을 합치거나 복사:

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

타입스크립트에서 Rest 문법:

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

타입스크립트에서 `this`:

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

위의 코드를 컴파일 했을 때 만일 `--noImplicitThis` 옵션이 있더라도 에러 없음.

### 콜백에서의 `this`

일반적인 상황에서의 `this`와 다르게 콜백으로 함수가 전달되었을 때의 `this`를 구분하여 강제해야 함:

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

- 객체의 스펙 (시그니처, 속성 타입)
- 함수의 스펙 (시그니쳐, 매개변수 타입, 반환 타입)
- 배열과 객체에 대한 접근 방식
- 클래스

### 간단한 인터페이스의 예:

간단한 인터페이스의 예:

```js
function printLabel(labeledObj: { label: string }) {
  console.log(labeledObj.label);
}
 
let myObj = { size: 10, label: "Size 10 Object" };
printLabel(myObj);
```

타입스크립트에서는 인터페이스를 제공하여 컴파일시 오류 검증:

```ts
interface LabeledValue {
  label: string;
}
 
function printLabel(labeledObj: LabeledValue) {
  console.log(labeledObj.label);
}
 
let myObj = { size: 10, label: "Size 10 Object" };
printLabel(myObj);
```

### 인터페이스에서 옵션 속성

초기값 주지 않아도 되는 인터페이스 속성:

```ts
interface SquareConfig {
  color?: string;
  width?: number;
}
 
function createSquare(config: SquareConfig): { color: string; area: number } {
  let newSquare = { color: "white", area: 100 };
  if (config.color) {
    newSquare.color = config.color;
  }
  if (config.width) {
    newSquare.area = config.width * config.width;
  }
  return newSquare;
}
 
let mySquare = createSquare({ color: "black" });
```

lint  또는 컴파일러가 오류 탐지:

```ts
interface SquareConfig {
  color?: string;
  width?: number;
}
 
function createSquare(config: SquareConfig): { color: string; area: number } {
  let newSquare = { color: "white", area: 100 };
  if (config.clor) {
Property 'clor' does not exist on type 'SquareConfig'. Did you mean 'color'?

    // Error: Property 'clor' does not exist on type 'SquareConfig'
    newSquare.color = config.clor;
Property 'clor' does not exist on type 'SquareConfig'. Did you mean 'color'?

  }
  if (config.width) {
    newSquare.area = config.width * config.width;
  }
  return newSquare;
}
 
let mySquare = createSquare({ color: "black" });
```

### 읽기 전용 속성

`readonly`를 사용하여 객체 생성시 값을 할당하고 변경할 수 없는 속성.

```ts
interface Point {
  readonly x: number;
  readonly y: number;
}

let p1: Point = { x: 10, y: 20 };
p1.x = 5; // error!

let a: number[] = [1, 2, 3, 4];
let ro: ReadonlyArray<number> = a;
 
ro[0] = 12; // error!
ro.push(5); // error!
ro.length = 100; // error!
a = ro; // error!
```

### 읽기 전용 배열

 `ReadonlyArray<T>` 은 읽기 전용 배열이며 선언시에만 초기자를 줄 수 있음:

```ts
let arr: ReadonlyArray<number> = [ 1,2,3 ];
arr.splice(0,1); // 배열 내용을 변경할 수 없으므로 오류.
arr.push(4); // 배열 내용을 변경할 수 없으므로 오류.
arr[0] = 100; // 배열 내용을 변경할 수 없으므로 오류.
```

imutable을 mutable에 할당은 허용되지 않음:

```typescript
let a: number[] = [1, 2, 3, 4];
let ro: ReadonlyArray<number> = a;
 
a = ro as number[];
```

`readonly`를 사용할지 `const`를 사용할지 기억하는 가장 쉬운 방법은 변수에 사용할지 프로퍼티에 사용할지에 따름. 변수는 const를 사용하는 반면 프로퍼티는 읽기 전용을 사용.

### 타입 체크

타입스크립트는 인터페이스를 통한 객체 선언에서 엄밀한 속성 검사:

```ts
interface SquareConfig {
  color?: string;
  width?: number;
}
 
function createSquare(config: SquareConfig): { color: string; area: number } {
  return {
    color: config.color || "red",
    area: config.width ? config.width * config.width : 20,
  };
}
 
let mySquare = createSquare({ colour: "red", width: 100 }); // colour 오류
```

타입 추론을 무시하려면 `as` 사용.

```ts
let mySquare = createSquare({ width: 100, opacity: 0.5 } as SquareConfig);
```

인터페이스 정의하지 않은 속성들을 추가로 사용:

```ts
interface SquareConfig {
  color?: string;
  width?: number;
  [propName: string]: any;
}

let squareOptions = { colour: "red" };
let mySquare = createSquare(squareOptions); // 오류
```

### 함수 타입

인터페이스로 함수 타입 정의:

```typescript
interface SearchFunc {
  (source: string, subString: string): boolean;
}
```

사용의 예:

```typescript
let mySearch: SearchFunc;
 
mySearch = function (source: string, subString: string): boolean {
  let result = source.search(subString);
  return result > -1;
};
```

타입만 체크하므로 동일:

```typescript
let mySearch: SearchFunc;
 
mySearch = function (src: string, sub: string): boolean {
  let result = src.search(sub);
  return result > -1;
};
```

오류:

```typescript
let mySearch: SearchFunc;
 
mySearch = function (src, sub) {
  let result = src.search(sub);
  return "string"; // string을 리턴하므로 오류
};
```

### 인덱서블 타입 Indexable Type:

`[]` 연산자를 사용할 수 있는 타입. 숫자를 인덱스로 주고 문자열을 반환:

```typescript
interface StringArray {
  [index: number]: string;
}
 
let myArray: StringArray;
myArray = ["Bob", "Fred"];
 
let myStr: string = myArray[0];
```

인덱스로는  자바스크립트의 레거시로 `string`, `number`, `symbol`, `template strings` 만 사용가능. 인덱스로 number를 사용하면 자바스크립트는 내부적으로 문자열로 변환하여 사용.

```typescript
interface Animal {
  name: string;
}
 
interface Dog extends Animal {
  breed: string;
}
 
interface NotOkay {
  [x: number]: Animal; // number 인덱스는 Animal타입으로 Dog 타입으로 할당 불가능.
  [x: string]: Dog;
}
```

```typescript
interface NumberDictionary {
  [index: string]: number; 
  length: number; // ok, length is a number
  name: string; // error, the type of 'name' is not a subtype of the indexer
}
```

```typescript
interface NumberOrStringDictionary {
  [index: string]: number | string; 
  length: number; // ok, length is a number
  name: string; // ok, name is a string
}
```

읽기 전용 인덱스:

```typescript
interface ReadonlyStringArray {
  readonly [index: number]: string;
}
 
let myArray: ReadonlyStringArray = ["Alice", "Bob"];
myArray[2] = "Mallory"; // error!
```

### Indexable Types with Template Strings

```typescript
interface HeadersResponse {
  "content-type": string,
  date: string,
  "content-length": string 
  // Permit any property starting with 'data-'.
  [headerName: 'x-${string}']: string;
}
 
function handleResponse(r: HeadersResponse) {
  // Handle known, and x- prefixed
  const type = r["content-type"]
  const poweredBy = r["x-powered-by"]
  const origin = r.origin // // Unknown keys without the prefix raise errors
}
```

### 클래스 타입

인터페이스 구현, 생성자는 `constructor`:

```ts
interface ClockInterface {
  currentTime: Date;
}
 
class Clock implements ClockInterface {
  currentTime: Date = new Date();
  constructor(h: number, m: number) {}
}
```

setTime 메소드 구현:

```typescript
interface ClockInterface {
  currentTime: Date;
  setTime(d: Date): void;
}
 
class Clock implements ClockInterface {
  currentTime: Date = new Date();
  setTime(d: Date) {
    this.currentTime = d;
  }
  constructor(h: number, m: number) {}
}
```

Difference between the static and instance sides of classes:

```typescript
interface ClockConstructor {
  new (hour: number, minute: number);
}
 
class Clock implements ClockConstructor {
  currentTime: Date;
  constructor(h: number, m: number) {} // new를 구현하지 않았으므로 오류
}
```

```typescript
interface ClockConstructor {
  new (hour: number, minute: number): ClockInterface;
}
 
interface ClockInterface {
  tick(): void;
}
 
function createClock(
  ctor: ClockConstructor,
  hour: number,
  minute: number
): ClockInterface {
  return new ctor(hour, minute);
}
 
class DigitalClock implements ClockInterface {
  constructor(h: number, m: number) {}
  tick() {
    console.log("beep beep");
  }
}
 
class AnalogClock implements ClockInterface {
  constructor(h: number, m: number) {}
  tick() {
    console.log("tick tock");
  }
}
 
let digital = createClock(DigitalClock, 12, 17);
let analog = createClock(AnalogClock, 7, 32);
```

```typescript
interface ClockConstructor {
  new (hour: number, minute: number): ClockInterface;
}
 
interface ClockInterface {
  tick(): void;
}
 
const Clock: ClockConstructor = class Clock implements ClockInterface {
  constructor(h: number, m: number) {}
  tick() {
    console.log("beep beep");
  }
};
 
let clock = new Clock(12, 17);
clock.tick();
```

#### 인터페이스 확장:

```typescript
interface Shape {
  color: string;
}
 
interface Square extends Shape {
  sideLength: number;
}
 
let square = {} as Square;
square.color = "blue";
square.sideLength = 10;
```

인터페이스 다중 상속:

```typescript
interface Shape {
  color: string;
}
 
interface PenStroke {
  penWidth: number;
}
 
interface Square extends Shape, PenStroke {
  sideLength: number;
}
 
let square = {} as Square;
square.color = "blue";
square.sideLength = 10;
square.penWidth = 5.0;
```

### 하이브리드 타입

예시: 함수 타입이면서 객체 타입인 인터페이스:

```typescript
interface Counter {
  (start: number): string;
  interval: number;
  reset(): void;
}
 
function getCounter(): Counter {
  let counter = function (start: number) {} as Counter;
  counter.interval = 123;
  counter.reset = function () {};
  return counter;
}
 
let c = getCounter();
c(10);
c.reset();
c.interval = 5.0;
```

클래스를 확장한 인터페이스:

```typescript
class Control {
  private state: any;
}
 
interface SelectableControl extends Control {
  select(): void;
}
 
class Button extends Control implements SelectableControl {
  select() {}
}
 
class TextBox extends Control {
  select() {}
}
 
class ImageControl implements SelectableControl {
  // Class 'ImageControl' incorrectly implements interface 'SelectableControl'.
  // Types have separate declarations of a private property 'state'.
  private state: any;
  select() {}
}
```

## 참조

- https://joshua1988.github.io/ts/guide/basic-types.html#array 를 참고하여 요약

[Up](../index.md)

# TypeScript

## 타입

### 기본 타입

타입스크립트의 기본 타입은 12가지가 있습니다.

| 타입        | 설명                                                       |      |
| ----------- | ---------------------------------------------------------- | ---- |
| `boolean`   | 진위                                                       |      |
| `number`    | 숫자                                                       |      |
| `string`    | 문자열                                                     |      |
| `object`    |                                                            |      |
| `Array`     | 배열                                                       |      |
| `Tuple`     | 튜플                                                       |      |
| `enum`      | 상수의 집합                                                |      |
| `any`       | 모든 타입                                                  |      |
| `void`      | `undefined` 또는 `null`만 할당, 함수의 경우 반환 값이 없음 |      |
| `null`      |                                                            |      |
| `undefined` |                                                            |      |
| `never`     | 함수의 끝에 절대 도달하지 않음                             |      |

#### `Boolean`:

타입이 진위 값인 경우에는 아래와 같이 선언합니다.

```ts
let isTrue: boolean = false;
```

#### `number`:

타입이 숫자이면 아래와 같이 선언합니다.

```ts
let age: number = 10;
```

#### `string`:

타입이 문자열이면 아래와 같이 선언합니다.

```ts
let said: string = 'hello';
```

#### `Array`:

타입이 배열이면 아래와 같이 선언합니다.

```ts
let ages: number[] = [11, 14, 27];
```

또는 아래와 같이 제네릭을 사용할 수 있습니다.

```ts
let ages: Array<number> = [11, 14, 27];
```

#### `Tuple`:

길이가 고정되고 각 요소의 타입이 지정되어 있는 배열 타입입니다.

```ts
let saidHello: [string, number] = ['hello', 8];
```

만약 정의하지 않은 타입, 인덱스로 접근할 경우 오류가 납니다.

```ts
saidHello[1].concat('!'); // Error, 'number' does not have 'concat'
saidHello[5] = 'hello'; // Error, Property '5' does not exist on type '[string, number]'.
```

#### `enum`:

상수값의 집합을 의미합니다.

```ts
enum Jobs { Engineer, Teacher, Student, Soldier, Nurse }
let work: Jobs = Jobs.Student;
```

인덱스로 접근할 수 있습니다.

```ts
let work: Jobs = Jobs[0]; // Engineer
```

인덱스를 사용자 편의로 변경하여 사용할 수도 있습니다.

```ts
let work: Jobs = Jobs[2]; // Student
let hero: Jobs = Jobs[4]; // Nurse
```

#### `any`:

자바스크립트로 구현되어 있는 웹 서비스 코드에 타입스크립트를 점진적으로 적용할 때 활용하기에 적합합니다. 모든 타입에 대해서 허용합니다.

```ts
let said: any = 'hello';
let age: any = 10;
let properties: any = [ 'a', 2, true ];
```

#### `void`:

`undefined`와 `null`만 할당 가능 합니다.

`void`가 함수 반환 타입에 지젇되면 반환 값을 설정할 수 없습니다:

```ts
let v: void = undefined;
```

#### `never`:

함수의 끝에 도달하지 않습니다:

```ts
function neverFinish(): never {
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

자바스크립트의 함수는 타입을 지정하지 않습니다:

```js
function sum(a, b) {
  return a + b;
}
```

타입 스크립트의 경우 매개변수와 반환값에 타입을 지정해야 합니다:

```ts
function sum(a: number, b: number): number {
  return a + b;
}
```

반환값이 없는 경우`void`를 반환 타입으로 지정해야 합니다:

```ts
function noresult(): void {
  console.log('noresult called');
}
```

### 함수의 인자

타입스크립트에서는 함수의 인자를 모두 필수 값으로 간주합니다. 함수의 매개변수에는 `undefined`나 `null`이라도 인자로 넘겨야 하며, 컴파일러는 정의된 매개변수 값이 넘어 왔는지 체크합니다:

```ts
function sum(a: number, b: number): number {
  return a + b;
}
sum(10, 20); // 결과: 30
sum(10, 20, 30); // error, too many parameters
sum(10); // error, too few parameters
```

정의된 매개변수 만큼 인자를 넘기지 않아도 되는 자바스크립트와 다릅니다. 인자를 선택적으로 전달하고 싶다면 `?`를 지정할 수 있습니다:

```ts
function sum(a: number, b?: number): number {
  return a + b;
}
sum(10, 20); // 30
sum(10, 20, 30); // error, too many parameters
sum(10); // 10
```

매개변수 초기화는 ES6와 동일합니다:

```ts
function sum(a: number, b = 100): number {
  return a + b;
}
sum(10, undefined); // 110
sum(10, 20, 30); // error, too many parameters
sum(10); // 110
```

### 매개변수에 REST 문법 적용

ES6에서 Spread 문법은 배열로된 하나의 인자를 받습니다.

```js
function sum(x, y, z) {
  return x + y + z;
}

const numbers = [1, 2, 3];
sum(...numbers); // expected output: 6
sum.apply(null, numbers); // expected output: 6
```

기존에 두 배열을 합치려면, `push`, `splice`, `concat`를 사용해야 했습니다. Spread 문법을 사용하면 두 배열을 간단히 합치 거나 복사가 가능합니다:

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

자바스크립트와 다르게 `this`의 잘못된 사용을 탐지 할 수 있습니다. 타입스크립트에서 `this`가 가리키는 것을 명시하려면 아래와 같이 사용합니다:

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

위의 코드를 타입스크립트로 컴파일 했을 때 만일 `--noImplicitThis` 옵션이 있더라도 에러가 발생하지 않습니다.

### 콜백에서의 `this`

일반적인 상황에서의 `this`와 다르게 콜백으로 함수가 전달되었을 때의 `this`를 구분해 주어야 하며 강제를 해야 합니다:

```ts
interface UIElement {
  // 아래 함수의 `this: void` 코드는 함수에 `this` 타입을 선언할 필요가 없다는 의미입니다.
  addClickListener(onclick: (this: void, e: Event) => void): void;
}

class UIHandler {
    info: string;
    onClick(this: Handler, e: Event) {
        // `UIElement`에 `this`는 `void`인데 `this`를 사용했기 때문에 오류입니다.
        this.info = e.message;
    }
}
let handler = new UIHandler();
uiElement.addClickListener(handler.onClick); // error!
```

만약 `UIElement` 인터페이스의 스펙에 맞춰 `Handler`를 구현하려면 아래와 같이 변경합니다:

```ts
class UIHandler {
    info: string;
    onClick(this: void, e: Event) {
        // `this`의 타입이 `void` 이기 때문에 `this`를 사용할 수 없습니다.
        console.log('clicked!');
    }
}
let handler = new UIHandler();
uiElement.addClickListener(handler.onClick);
```


ReactJS

# 컴포넌트의 `constructor()` 알아보기

- 컴포넌트의 생성자로 컴포넌트가 메모리에 할당되고 나서 호출된다.
- `props`를 인자로 전달 받으며 `props`는 태그를 통해서 기술되고 전달 된다.

```jsx
<SomeComponent some_prop="some value"/>
```

`MyComponent.js`에 `constructor()` 함수를 재정의 한다:

```js
import React, { Component } from 'react';

class MyComponent extends Component {
    constructor(props) {
        super(props);
        this.state = {};
        console.log('MyComponent.constructor() called.');
    }
    render () {
        return (
            <h2>[This is MyComponent]</h2>
        )
    }
}

export default MyComponent;
```

- `MyComponent`에 `state` 개체를 추가 하였다.

표출되는 코드에 영향을 주지 않으므로 저장하여도 웹브라우저 표출 내용은 바뀌지 않을 것이다.

웹브라우저에서 `F12`를 눌러 개발자 도구에서 `Console`탭을 누르면 다음과 같이 표출 되어 있을 것이다.

```
MyComponent.constuctuor() called.
```

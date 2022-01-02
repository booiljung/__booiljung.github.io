ReactJS

# 컴포넌트의 `componentDidMount()` 알아보기

- `componentDidMount()`는 `render()` 함수가 return 되는 html 형식의 코드를 DOM에 그려준 다음에 호출된다.
- DOM에 그려진 이후 실행되어야 하는 이벤트 처리, 초기화 등에 사용된다.

`MyComponent`에 `componentDidMount()` 함수를 추가하자:

```js
import React, { Component } from 'react';

class MyComponent extends Component {
    constructor(props) {
        super(props);
        this.state = {};
        console.log('MyComponent.constructor() called.');
    }
    static getDerivedStateFromProps(props, state) {
        console.log('MyComponent.static getDerivedStateFromProps(props, state) called:' + props.my_props);
        return { my_state:props.my_props};
    }
    render () {
        return (
            <h2>[This is MyComponent]</h2>
        )
    }
    componentDidMount() {
        console.log('MyComponent.componentDidMount() called: ' + this.state.my_state);
    }
}

export default MyComponent;
```

개발자 도구의 콘솔을 보면 로그가 표시 되어 있다:

```
MyComponent.constructor() called.
MyComponent.static getDerivedStateFromProps(props, state) called: from App.js
MyComponent.componentDidMount() called: from App.js
```

- `App.js`에서 `MyComponent` 태그에 `<MyComponent my_props='from App.js'/>`로 주입한  `from App.js`가 `MyComponent.getDerivedStateFromProps()`를 거쳐 `MyComponent.componentDidMount()`에 전달 하고 있는 것을 확인 할 수 있다.




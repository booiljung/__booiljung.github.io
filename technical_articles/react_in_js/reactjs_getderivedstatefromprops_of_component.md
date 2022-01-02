ReactJS

# 컴포넌트의 `    getDerivedStateFromProps()` 알아보기

- `constructor()`함수 다음에 실행되며 컴포넌트가 새로운 props를 받게 되면 state를 변경해 준다.

`App.js`의 `App` 태그에 프로퍼티 `my_prop`를 추가 한다:

```js
import logo from './logo.svg';
import './App.css';
import MyComponent from './MyComponent';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <MyComponent my_props='from App.js'/>
      </header>
    </div>
  );
}

export default App;
```

- `MyComponent` 태그에 `my_props='from App.js'`를 추가 하였다. `App.js`에서 `MyComponent` 태그에 프로퍼티를 전달한다.

`MyComponent.js`에 `static getDerivedStateFromProps()` 함수를 재정의 한다:

```js
import React, { Component } from 'react';

class MyComponent extends Component {
    constructor(props) {
        super(props);
        this.state = {};
        console.log('MyComponent.constructor() called.');
    }
    static getDerivedStateFromProps(props, state) {
        console.log('MyComponent.static getDerivedStateFromProps(props, state) called: ' + props.my_props);
        return {};
    }
    render () {
        return (
            <h2>[This is MyComponent]</h2>
        )
    }
}

export default MyComponent;
```

개발자 도구의 콘솔을 보면 로그가 표시 되어 있다:

```
MyComponent.constructor() called.
MyComponent.static getDerivedStateFromProps(props, state) called: from App.js
```

- `App.js`에서 `MyComponent` 태그에 `<MyComponent my_props='from App.js'/>`로 주입한 props `from App.js`가 `MyComponent`에 전달 하고 있는 것을 확인 할 수 있다.


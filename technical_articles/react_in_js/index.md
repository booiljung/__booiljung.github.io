# React in JavaScript

##  앱 생성하고 실행하기

앱 생성:

```sh
npx create-react-app app001
```

`src/index.js`:

```js
import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import App from './App';
import reportWebVitals from './reportWebVitals';

ReactDOM.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
  document.getElementById('root')
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
```

여기서 `React`는 최상위 컴포넌트로 `App` 컴포넌트를 렌더링 하고 있다.

`src/index.css`

```css
body {
  margin: 0;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen',
    'Ubuntu', 'Cantarell', 'Fira Sans', 'Droid Sans', 'Helvetica Neue',
    sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
}

code {
  font-family: source-code-pro, Menlo, Monaco, Consolas, 'Courier New',
    monospace;
}
```

`src/App.js`:

```js
import logo from './logo.svg';
import './App.css';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.js</code> and save to reload.
        </p>
        <a
          className="App-link"
          href="https://reactjs.org"
          target="_blank"
          rel="noopener noreferrer"
        >
          Learn React
        </a>
      </header>
    </div>
  );
}

export default App;
```

실행해보자:

```sh
npm start
```

새 웹브라우저 탭이 생성되고 표출된다:

![image-20220101223105956](index.assets/image-20220101223105956.png)

`App.js`에서 텍스트 두줄을 제거하도록 아래와 같이 코드를 제거하여 수정한다:

```js
import logo from './logo.svg';
import './App.css';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
      </header>
    </div>
  );
}

export default App;
```

저장하면 웹페이지가 다음과 같이 변경된다:

![image-20220101224754884](index.assets/image-20220101224754884.png)

## 컴포넌트 추가하고 삽입하기

새 컴포넌트 파일 `MyComponent.js`를 생성하고 다음 내용을 채우자:

```js
import React, { Component } from 'react';

class MyComponent extends Component {
    render () {
        return (
            <h2>[This is MyComponent]</h2>
        )
    }
}

export default MyComponent;
```

- `import React, { Component } from 'react';`로 `Component` 클래스 참조시 `React.`을 생략할 수 있도록 하였다. `{ Component }`내용이 빠진다면 클래스 상속시 아래처럼 기술해야 한다.

  - ```js
    class MyComponent extends React.Component {
        render () {
            return (
                <h2>[This is MyComponent]</h2>
            )
        }
    }
    ```

- `Component` 클래스를 상속 받아 `MyComponent`라는 클래스를 정의하고 render() 함수를 오버라이드하여 `<h2>[This is my Component]</h2>`를 반환하였다.

- `render()`함수는 html 태그 형식을 반환하며 DOM이 변경되어야 할 시점에 호출된다.

-  `export default MyComponent`를 통해 외부에 `MyComponent`의 존재를 알렸다.

`App.js` 파일의 내용을 아래처럼 변경 한다.

```js
import logo from './logo.svg';
import './App.css';
import MyComponent from './MyComponent';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <MyComponent/>
      </header>
    </div>
  );
}

export default App;
```

저장하면 웹 페이지가 아래와 같이 변경된다:

![image-20220101224910125](index.assets/image-20220101224910125.png)

- 로고 아래에 [This is MyComponent]가 표출 되었다.

## `constructor()` 함수

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

## `    static getDerivedStateFromProps()`함수

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
        console.log('MyComponent.static getDerivedStateFromProps(props, state) called:' + props.my_props);
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

- `static getDerivedStateFromProps()`함수는 `constructor()`함수 다음에 실행되며 컴포넌트가 새로은 props를 받게 되면 state를 변경해 준다.

개발자 도구의 콘솔을 보면 로그가 표시 되어 있다:

```
MyComponent.constructor() called.
MyComponent.static getDerivedStateFromProps(props, state) called:from App.js
```

- `App.js`에서 `MyComponent` 태그에 `<MyComponent my_props='from App.js'/>`로 주입한 props `from App.js`가 `MyComponent`에 전달 하고 있는 것을 확인 할 수 있다.
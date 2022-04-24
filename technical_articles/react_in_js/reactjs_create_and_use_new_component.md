ReactJS

# 컴포넌트 추가하고 삽입하기

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

- `export default MyComponent`를 통해 외부에 `MyComponent`의 존재를 알렸다.

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

![image-20220101224910125](reactjs_add_component.assets/image-20220101224910125.png)

- 로고 아래에 [This is MyComponent]가 표출 되었다.
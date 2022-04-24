ReactJS

# props의 자료형

컴포넌트의 `props`에 자료형을 선언해 두면 부모 컴포넌트에서 주입하는 `props`의 타입을 확인하여 경고를 할 수 있게 한다. `MyType.js` 파일을 추가 하자:

```js
import React, { Component } from 'react';
import datatype from 'prop-types';

class MyType extends Component {
	render() {
        let { boolean, number, string, array, object_json, func } = this.props
        return (
            <div>
                <p>Boolean props: {boolean.toString()}</p>
                <p>Number props: {number}</p>
                <p>String props: {string}</p>
                <p>Array props: {array.toString()}</p>
                <p>ObjectJson props: {JSON.stringify(json)}</p>
                <p>Function props: {func}</p>
            </div>
        )
	}
}

MyType.propTypes = {
	boolean: datatype.bool,
	number: datatype.number,
	string: datatype.number,
	array: datatype.array,
	json: datatype.object,
	func: datatype.func,
}

export default MyType;
```

`App.js`에 새로 추가된 `MyType`를 사용해 보자:

```js
import logo from './logo.svg';
import './App.css';
import MyComponent from './MyComponent';
import MyType from './MyType';
import react from 'react';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <MyComponent my_props='from App.js'/>
        <MyType
          bool={1===1}
          num={1234}
          str="this string"
          arr={[3, 1, 4]}
          json={{lang:'js', lib:'react', editor:'vscode'}}
          func={console.log("func: function")}/>
      </header>
    </div>
  );
}

export default App;
```


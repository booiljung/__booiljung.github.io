ReactJS

# JQuery button event

jquery를 설치한다:

```sh
npm install jquery
```

`MyComponent.js`를 변경한다:

```js
import $ from 'jquery';
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

    prompt_input(e) {
        let input = $('#inputId');
        let value = input.val();
        alert(value);
    }

    render () {
        return (
            <div>            
                <h2>[This is MyComponent]</h2>
                <input id="inputId" name="inputName" value="inputValue"></input>
                <button id="buttonId" onClick={e => this.prompt_input(e)}>Prompt</button>
            </div>
        )
    }
    componentDidMount() {
        console.log('MyComponent.componentDidMount() called: ' + this.state.my_state);
    }
}

export default MyComponent;
```

`MyComponent` 클래스에 `prompt_input()` 함수가 추가 되었다. 

```js
    prompt_input(e) {
        let input = $('#inputId');
        let value = input.val();
        alert(value);
    }
```

- `$('#inputId')`은 `inputId`를 가진 개체를 찾는다.
- `input.val()`은 개체의 현재 값을 반환 한다.
- `alert(value)`은 화면에 표시한다.
- 이 3개 라인은 `inputId`에 입력된 현재 값을 읽어서 웹 브라우저에 `alert()`으로 표시한다.


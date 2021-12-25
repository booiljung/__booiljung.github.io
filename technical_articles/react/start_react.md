# React 시작하기

- Focal fossa에서 테스트 함

#### 설치

```sh
sudo apt install npm
```

#### 애플리케이션 생성

주의: 이전에 `npm install -g create-react-app`로 이미 설치되어 있다면, `npm uninstall -g create-react-app`이나 `yarn global remove create-react-app`로 제거.

새 애플리케이션 생성

```sh
npx create-react-app my-app
```

타입스크립트 지원:

```
npx create-react-app my-app --template typescript
```

yarn 패키지 매니저 선택 (추천):

```sh
npx create-react-app my-app --use-yarn
```

npm + typescript 로 생성

```sh
npx create-react-app my-app --use-npm --template typescript
```

#### 애플리케이션 시작

```sh
npm start
```

#### 단위 테스트

```sh
npm test
```

#### 프러덕션 빌드

```sh
npm run build
```

#### 폴더 구조

```
my-app/
  README.md
  node_modules/
  package.json
  public/
    index.html
    favicon.ico
  src/
    App.css
    App.js
    App.test.js
    index.css
    index.js
    logo.svg
```

- `public/index.html`: 페이지 템플릿
- `src/index.tsx`: js 진입점

- `src/` 개발자가 작성하는 소스코드

#### ./vscode/launch.json

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Chrome",
      "type": "chrome",
      "request": "launch",
      "url": "http://localhost:3000",
      "webRoot": "${workspaceFolder}/src",
      "sourceMapPathOverrides": {
        "webpack:///src/*": "${webRoot}/*"
      }
    }
  ]
}
```

#### 자동 코드 포매팅

```
sudo npm install --save husky lint-staged prettier
```

`package.json`에 추가:

```json
"husky": {
  "hooks": {
    "pre-commit": "lint-staged"
  }
}
```

```json
"lint-staged": {
  "src/**/*.{js,jsx,ts,tsx,json,css,scss,md}": [
    "prettier --write"
  ]
},
```

#### Styleguidist

```sh
npm install --save react-styleguidist
```

`package.json`에 추가:

```json
"scripts": {
  "styleguide": "styleguidist server",
  "styleguide:build": "styleguidist build"
},
```

스타일가이드 실행:

```sh
npm run styleguide
```

프로젝트 루트 폴더에 styleguide.config.js 추가:

```js
module.exports = {
  components: 'src/components/**/*.js'
};
```

#### Source map explorer

```sh
npm install --save source-map-explorer
```

`package.json`에 추가:

```json
"scripts": {
  "analyze": "source-map-explorer 'build/static/js/*.js'",
}
```

프러덕션 빌드하고 분석:

```sh
npm run build
npm run analyze
```

#### 개발시 HTTPS 사용

```sh
HTTPS=true npm start
```

사설 인증서:

```sh
HTTPS=true SSL_CRT_FILE=cert.crt SSL_KEY_FILE=cert.key npm start
```

## 컴포넌트

#### 버튼

`button.css`:

```css
.Button {
  padding: 20px;
}
```

`button.tsx`:

```tsx
import React, { Component } from 'react';
import './button.css';

class Button extends Component {
  render() {
    // You can use them as regular CSS styles
    return <div className="Button" />;
  }
}
```





## 참조

- https://create-react-app.dev/docs/getting-started

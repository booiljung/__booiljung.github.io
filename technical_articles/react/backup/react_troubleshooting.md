# React 문제 해결

## 문제 해결

#### uv_cwd

```
Error: ENOENT: no such file or directory, uv_cwd
```

폴더를 나갔다가 다시 들어오기

```sh
cd ..
cd my-app/
```









#### [styled-component](https://styled-components.com/)

```sh
npm install --save styled-components react-router-dom dotenv
npm install --save-dev @types/styled-components babel-plugin-styled-components @types/react-router-dom
npm install -D gh-pages
```





#### 자동 코드 포매팅

```sh
npm install --save husky lint-staged prettier
npm audit fix
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
}
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
}
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











## 참조

- https://create-react-app.dev/docs/getting-started

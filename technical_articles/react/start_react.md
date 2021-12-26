# React 시작하기

2021년 12월

#### npm

설치:

```sh
sudo apt install npm
```

버전 확인:

```
npm --version
```

여기서 문제:

```sh
Error: Cannot find module 'semver'
```

문제 해결 (https://askubuntu.com/questions/1152570/npm-cant-find-module-semver-error-in-ubuntu-19-04)

```sh
sudo rm -rf /usr/local/bin/npm /usr/local/share/man/man1/node* ~/.npm
sudo rm -rf /usr/local/lib/node*
sudo rm -rf /usr/local/bin/node*
sudo rm -rf /usr/local/include/node*

sudo apt purge nodejs npm -y
sudo apt autoremove -y
```

Download the latest `tar.xz` NodeJS file from https://nodejs.org/en/download/

```sh
tar -xf node-v#.#.#-linux-x64.tar.xz
sudo mv node-v#.#.#-linux-x64/bin/* /usr/local/bin/
sudo mv node-v#.#.#-linux-x64/lib/node_modules/ /usr/local/lib/
cat 'export PATH="/usr/local/bin/:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

업데이트:

```sh
sudo npm install -g npm
```

#### 애플리케이션 생성

주의: 이전에 `npm install -g create-react-app`로 이미 설치되어 있다면, `npm uninstall -g create-react-app`이나 `yarn global remove create-react-app`로 제거.

새 애플리케이션 생성:

```sh
npx create-react-app my-app
```

- 프로젝트 이름은 소문자로 시작 한다.
- webpack, babel, dev server 패키지가 기본으로 설치 된다.

새 애플리케이션 생성 with 타입스크립트 지원:

```
npx create-react-app my-app --template typescript
```

새 애플리케이션 생성 with yarn:

```sh
npx create-react-app my-app --use-yarn
```

새 애플리케이션 생성 with npm + typescript:

```sh
npx create-react-app my-app --use-npm --template typescript
```

새 애플리케이션 생성 with yarn + typescript:

```shell
npx create-react-app my-app --use-yarn --template typescript
```

#### 새 애플리케이션의 폴더 구조

```
my-app/
  node_modules/
  public/
    favicon.ico
    index.html
    logo192.png
    logo512.png
    manifest.json
    robot.txt
  src/
    App.css
    App.test.tsx
    App.tsx
    index.css
    index.tsx
    logo.svg
    reat-app-env.d.ts
    reportWebVitals.ts
    setupTests.ts
  .gitignore
  package-lock.json
  package.json
  README.md
  tsconfig.json
```

- `public/index.html`: 페이지 템플릿
- `src/` 개발자가 작성하는 소스코드
- `src/index.tsx`: 진입점

#### `src/index.tsx` 내용

```tsx
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

기본으로 `App` 컴포넌트를 실행하고 있다.

#### 애플리케이션 시작

```sh
npm start
```

HTTPS 시작

```sh
HTTPS=true npm start
```

사설 인증서 HTTPS로 시작:

```sh
HTTPS=true SSL_CRT_FILE=cert.crt SSL_KEY_FILE=cert.key npm start
```

애플리케이션 중지:

```sh
npm stop
```

애플리케이션 재시작:

```sh
npm restart
```

#### 단위 테스트

```sh
npm test
```

#### 프러덕션 빌드

```sh
npm run build
```

#### [패키지](https://www.zerocho.com/category/NodeJS/post/58285e4840a6d700184ebd87)

package.json 생성:

```sh
npm init
```

패키지 설치:

```sh
npm install 패키지@버전
```

dependencies에 추가 (npm5 부터 기본 옵션):

```sh
npm install 패키지@버전 --save 또는 -S
```

devDependencies에 추가:

```sh
npm install 패키지@버전 --save-dev 또는 -D
```

글로벌 패키지에 추가:

```sh
npm install 패키지@버전 -g
```

패키지 업데이트:

```sh
npm update
```

중복 패키지 제거:

```sh
npm dedupe
```

패키지에 대한 설명:

```sh
npm docs
```

`node_modules` 위치 보기:

```sh
npm root
```

오래된 패키지 확인:

```sh
npm outdated
```

패키지 목록:

```sh
npm ls
npm ls 패키지
```

패키지 자세히 보기:

```sh
npm ll
```

패키지 버그 리포트:

```sh
npm bugs
```

#### 외부 모듈의 보안 취약점 점검

모듈 설치 후 취약점 점검:

```sh
npm audit
```

취약점 수정:

```sh
npm audit fix
```

프로텍션 없이 취약점 수정:

```shell
npm audit fix --force
```

#### 설정

캐시 보기:

```sh
npm cache
```

캐시 삭제 - 문제가 있을때 해결 수단일 수도 있음:

```sh
npm cache clean
```

npm 다시 설치:

```sh
npm rebuild
```

현재 설정 목록 보기:

```sh
npm config list
```

특정 설정 변경:

```sh
npm set 이름 [값
```

특정 설정 조회:

```sh
npm get 이름
```

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

## 참조

- https://create-react-app.dev/docs/getting-started

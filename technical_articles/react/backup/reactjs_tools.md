ReactJS

# 도구

새 애플리케이션 생성:

```sh
npx create-react-app my-app
```

- 프로젝트 이름은 소문자로 시작 한다.
- webpack, babel, dev server 패키지가 기본으로 설치 된다.

#### 애플리케이션 실행 시작

```sh
npm start
```

HTTPS로 시작:

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

#### 
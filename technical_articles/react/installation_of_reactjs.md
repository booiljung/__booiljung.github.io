2023년 1월 24일 작성

# ReactJS 설치

## 로컬에 설치 방법

ReactJS를 로컬에 설치하는 방법은 NodeJS 개발환경을 사용하므로 [NodeJS 설치](../nodejs/installation_of_nodejs.md) 를 참조한다.

## 도커로 개발 및 배포하는 방법

참조: [프론트엔드 개발자를 위한 Docker로 React 개발 및 배포하기](https://velog.io/@oneook/Docker%EB%A1%9C-React-%EA%B0%9C%EB%B0%9C-%EB%B0%8F-%EB%B0%B0%ED%8F%AC%ED%95%98%EA%B8%B0) 는 Sanjeev Thiyagarajan님의 유튜브 강좌를 oneook님이 macos에서 진행한 것이다. 이 글은 우분투를 기준으로 설명한다.

## Dockerfile 작성

```
# 가져올 이미지를 정의
FROM node:18.13
# 경로 설정하기
WORKDIR /app
# package.json 워킹 디렉토리에 복사 (.은 설정한 워킹 디렉토리를 뜻함)
COPY package.json .
# 명령어 실행 (의존성 설치)
RUN npm install
# 현재 디렉토리의 모든 파일을 도커 컨테이너의 워킹 디렉토리에 복사한다.
COPY . .

# 각각의 명령어들은 한줄 한줄씩 캐싱되어 실행된다.
# package.json의 내용은 자주 바뀌진 않을 거지만
# 소스 코드는 자주 바뀌는데
# npm install과 COPY . . 를 동시에 수행하면
# 소스 코드가 조금 달라질때도 항상 npm install을 수행해서 리소스가 낭비된다.

# 3000번 포트 노출
EXPOSE 3000

# npm start 스크립트 실행
CMD ["npm", "start"]
```

작성되었으면 이미지를 빌드한다.

```
$ docker build . -t <이미지:태그>
```

이미지를 확인 한다.

```
$ docker image ls
```

실행한다.

```
$ docker run -d --name <컨테이너이름> <이미지:태그>
```


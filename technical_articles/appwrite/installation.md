# Installation of Appwrite

## 백엔드 설치

개발을 위해 다음 환경에 Appwrite를 설치한다.

- Unbutu 20.04 LTS Desktop
- AMD64, RAM 32G

도커 버전

```
$ docker -v
Docker version 20.10.17, build 100c701
```

도커 컴포즈 버전

```
$ docker-compose -v
docker-compose version 1.29.2, build 5becea4c
```

Appwrite 도커 컴포즈 파일을 https://github.com/appwrite/appwrite/blob/master/docker-compose.yml 에서 다운로드 받아 설치 할 수 있다.

`.env`파일을 설정한다.

`_APP_FUNCTIONS_RUNTIMES`에 런타임을 지정한다. 

지원 런타임은 https://github.com/appwrite/functions-starter 를 참조한다.

도커 컴포즈로 설치한다.

```
$ docker-compose up -d
```

로그 확인

```
$ docker-compose logs -f
```

설치 후 http://localhost 에 로그인하여 root 계정을 생성하자.




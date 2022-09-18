# Appwrite

Apppwrite는 오픈소스 서버리스 플랫폼으로 https://appwrite.io/ 에서 제공 한다.

> Appwrite is a self-hosted backend-as-a-service platform that provides  developers with all the core APIs required to build any application.

Appwrite는 다음을 제공한다.

- 오픈소스 https://github.com/appwrite/appwrite
- 백엔드서버
- redis를 통한 메모리 캐시
- 유저 (관리자), 감독
- 계정 (일반 사용자), 팀, 아바타
- 데이터베이스, 파일저장소
- 펑션, 지역화, Webhook
- MySQL 지원. (MySQL은 JOIN 성능이 나쁜 편에 속한다. 어서 PGSQL을 지원하기를 바란다.)

## 이 문서의 목적

Appwrite는 문서가 부족하며 공식 문서는 불친절하며 따라하여도 제대로 동작하지 않는다.

Appwrite를 익혀가며 실제 동작하는 방식을 적어 간다.

## 이 문서의 한계

이 페이지는 Appwrite 사용법이 아니다.  Appwrite를 배우면서 튜토리얼을 기록하며 다음 제약에서 사용할 것이다.

- Ubuntu 20.04가 설치된 로컬 컴퓨터에서 Docker-Compose로 설치 운영할 것이다.
- 클라이언트 앱은 Flutter로 개발 될 것이다.
- 서버 앱은 Flutter와 동일한 Dart 또는 Python으로 개발 될 것이다.
- 에디터는 VSCODE를 사용할 것이다.
- 서버는 Bash 터미널에서 CLI로 개발 될 것이다.

## 차례

- [설치](installation.md)
- [프로젝트](project.md)
- [Functions](functions.md)


[Up](index.md)

# 도커로 PostgreSQL과 Redmine 설치하기

이미지 다운로드

```sh
sudo docker pull postgres:12.3
sudo docker pull redmine:4.1.1
```

데이터베이스 경로 생성

```sh
sudo mkdir -p /var/lib/redmine/data
```

네트워크 생성

```sh
sudo docker network create booil-redmine-network --driver=bridge
```

PostgreSQL 컨테이너 시작

```sh
sudo docker run -d postgres:12.3 \
--name booil-redmine-postgres \
--volume /var/lib/redmine/data:/var/lib/postgresql/data \
-e POSTGRES_PASSWORD=secret \
-e POSTGRES_USER=redmine 
```

Redmine 컨테이너 시작

```sh
sudo docker run -d redmine:4.1.1 \
--name booil-redmine \
--volume /var/lib/redmine/data:/usr/src/redmine/files \
--link booil-redmine-postgres:postgres \
-e REDMINE_DB_POSTGRES=booil-redmine-postgres \
-e REDMINE_DB_USERNAME=redmine \
-e REDMINE_DB_PASSWORD=secret \
--port 8081:3000
```

모든 컨테이너 보기

```sh
sudo docker ps -a
```

컨테이너 정지
```sh
sudo docker stop booil-redmine
sudo docker stop booil-redmine-postgres
```

컨테이너 제거

```sh
sudo docker rm booil-redmine
sudo docker rm booil-redmine-postgres
```

네트워크 제거

```sh
sudo docker network rm booil-redmine-network
```

`docker-compose.yml`

```yaml
version: '3.3'
services:
  server:
    image: ubuntu:18.04
    container_name: redmine-server
  data:
    image: postgres:12.3
    container_name: redmine-data
    restart: always
    depends_on:
      - server
    volumes:
      - /var/lib/redmine/data:/var/lib/postgresql/data
    networks:
      - redmine
    environment:
      - POSTGRES_PASSWORD=secret
      - POSTGRES_USER=redmine
  web:
    image: redmine:4.1.1
    container_name: redmine-web
    restart: always
    depends_on:
      - data
      - server
    volumes:
      - /var/lib/redmine/files:/usr/src/redmine/files
    networks:
      - redmine
    ports:
      - "8081:3000"
    environment:
      - REDMINE_DB_POSTGRES=booil-redmine-postgres
      - REDMINE_DB_USERNAME=redmine
      - REDMINE_DB_PASSWORD=secret
networks:
  redmine:
    name: redmine
    driver: bridge

```

## 참조

- [dockerhub: redmine](https://hub.docker.com/_/redmine)

- [dockerhub: postgresql](https://hub.docker.com/_/postgres)




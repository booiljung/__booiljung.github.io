## Ubuntu 18.04에 Docker-Compose로 ownCloud 설치

root 계정으로 적당한 폴더를 만든다.

```
sudo passwd root
...
su
...

mkdir /srv/owncloud
```

예제 `docker-compose.yaml`을 가져 와서

```
wget https://raw.githubusercontent.com/owncloud/docs/master/modules/admin_manual/examples/installation/docker/docker-compose.yml
```

편집 한다.

```
version: '2.1'

services:
  owncloud:
    image: owncloud/server:10.6
    restart: always
    ports:
      - 80:8080
    depends_on:
      - db
      - redis
    environment:
      - OWNCLOUD_DOMAIN=localhost:80
      - OWNCLOUD_DB_TYPE=mysql
      - OWNCLOUD_DB_NAME=owncloud
      - OWNCLOUD_DB_USERNAME=owncloud
      - OWNCLOUD_DB_PASSWORD=file@sm2020
      - OWNCLOUD_DB_HOST=db
      - OWNCLOUD_ADMIN_USERNAME=admin
      - OWNCLOUD_ADMIN_PASSWORD=file@sm2020
      - OWNCLOUD_MYSQL_UTF8MB4=true
      - OWNCLOUD_REDIS_ENABLED=true
      - OWNCLOUD_REDIS_HOST=redis
    healthcheck:
      test: ["CMD", "/usr/bin/healthcheck"]
      interval: 30s
      timeout: 10s
      retries: 5
    volumes:
      - /srv/owncloud/files:/mnt/data

  db:
    image: webhippie/mariadb:latest
    restart: always
    environment:
      - MARIADB_ROOT_PASSWORD=owncloud
      - MARIADB_USERNAME=owncloud
      - MARIADB_PASSWORD=file@sm2020
      - MARIADB_DATABASE=owncloud
      - MARIADB_MAX_ALLOWED_PACKET=128M
      - MARIADB_INNODB_LOG_FILE_SIZE=64M
    healthcheck:
      test: ["CMD", "/usr/bin/healthcheck"]
      interval: 30s
      timeout: 10s
      retries: 5
    volumes:
      - /srv/owncloud/mysql:/var/lib/mysql
      - /srv/owncloud/backup:/var/lib/backup

  redis:
    image: webhippie/redis:latest
    restart: always
    environment:
      - REDIS_DATABASES=1
    healthcheck:
      test: ["CMD", "/usr/bin/healthcheck"]
      interval: 30s
      timeout: 10s
      retries: 5
    volumes:
      - /srv/owncloud/redis:/var/lib/redis
```

컨테이너를 생성하고 올린다.

```
docker-compose up -d
```



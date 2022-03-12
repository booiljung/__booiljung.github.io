# sqlalchemy

2022년 3월 12일

sqlalchemy를 스터디 합니다.

## 환경

- ubuntu 20.04 lts
- docker, docker-compose
- postgresql
- python-3.9.10

## docker-ce 설치

레거시 버전 제거:

```sh
sudo apt remove -y \
	docker \
	docker-engine \
	docker.io \
	containerd runc
```

의존성 패키지 설치:

```sh
sudo apt install -y \
	apt-transport-https \
	ca-certificates \
	curl \
	gnupg \
	gnupg-agent \
	software-properties-common
```

도커 gpg 설치 및 패키지 저장소 등록:

```sh
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

echo \
	"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
	$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

docker 설치:

```sh
sudo apt-get update

sudo apt-get install \
	docker-ce \
	docker-ce-cli \
	containerd.io
```

docker 설치 테스트:

```sh
sudo docker run hello-world
```

## docker-compose 1.29.2설치

```sh
sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

## docker-compose로 postgresql과 pgadmin 설치

`docker-compose.yml` 파일:

```yaml
version: '1.0'

volumes:
    postgres:
    pgadmin:

services:
  postgres:
    container_name: firstalchemy-pgadmin-devel
    image: "postgres:13.3"
    environment:
      POSTGRES_DB: "firstalchemy"
      POSTGRES_USER: "postgres"
      POSTGRES_PASSWORD: "1234"
      PGDATA: "/data/postgres"
    volumes:
      - postgres:/data/postgres
      - ./docker_postgres_init.sql:/docker-entrypoint-initdb.d/docker_postgres_init.sql
    ports:
      - "15432:5432"
    restart: unless-stopped
    
  pgadmin:
    container_name: first-alchemy-pgadmin-devel
    image: "dpage/pgadmin4:5.4"
    environment:
      PGADMIN_DEFAULT_EMAIL: hong@gil.dong
      PGADMIN_DEFAULT_PASSWORD: 1234
      PGADMIN_CONFIG_SERVER_MODE: "False"
      PGADMIN_CONFIG_MASTER_PASSWORD_REQUIRED: "False"
    volumes:
      - pgadmin:/data/pgadmin
      - ./docker_pgadmin_servers.json:/pgadmin4/servers.json
    ports:
      - "15433:80"
    entrypoint:
      - "/bin/sh"
      - "-c"
      - "/bin/echo 'firstalchemy:5432:*:postgres:1234' > /tmp/pgpassfile && chmod 600 /tmp/pgpassfile && /entrypoint.sh"
    restart: unless-stopped
```

- `restart: unless-stopped`를 설정되어 부팅 후에도 자동으로 올라온다.

`docker_pgadmin_servers.json` 파일:

```json
{
    "Servers": {
      "1": {
        "Name": "firstalchemy",
        "Group": "Servers",
        "Port": 5432,
        "Username": "postgres",
        "Host": "postgres",
        "SSLMode": "prefer",
        "MaintenanceDB": "firstalchemy",
        "PassFile": "/tmp/pgpassfile"
      }
    }
  }

```

빈 `docker_postgres_init.sql` 파일:

```sql

```

docker-compose로 postgresql과 pgadmin 올리기:

```sh
sudo docker-compose up -d
```

postgresql과 pgadmin이 올라온 상태 보기:

```sh
sudo docker ps
```

docker-compose로 postgresql과 pgadmin 내리기:

```sh
sudo docker-compose down
```

pgadmin 웹 URL:

```
http://localhost:15433
```

## sqlalchemy 개발 환경 구성

*시행 착오로 얻음*

python에서 postgresql을 사용하려면 psycopg2 패키지를 필요로 하며, psycopg2는 libpq-dev과 postgresql-common을 필요로 합니다.

```sh
sudo apt install -y libpq-dev postgresql-common
```

requirements.txt 파일:

```
wheel
psycopg2==2.9.3
sqlalchemy
```

패키지 설치:

```sh
pip install -r requirements.txt
```

## sqlalchemy 초급

*참조: https://docs.sqlalchemy.org/en/14/core/engines.html*

버전 체크:

```python
import sqlalchemy
print(sqlalchemy.__version__)
```

기본 엔진 생성: 

```python
from sqlalchemy import create_engine
engine = create_engine('postgresql://postgres:1234@localhost:15432/firstalchemy')
```

기본 엔진 URL을 자세히 설명:

```python
from sqlalchemy import create_engine
engine = create_engine('<데이터베이스 드라이버>://<데이터베이스 사용자 이름>:<이 사용자의 비밀번호>@<호스트>:<포트>/<데이터베이스 이름>')
```

psycopg2 드라이버: 

```python
engine = create_engine('postgresql+psycopg2://postgres:1234@localhost:15432/firstalchemy')
```

pg8000 드라이버:

```python
engine = create_engine('postgresql+pg8000://postgres:1234@localhost:15432/firstalchemy')
```


# Docker 에서 UDP 서버 테스트

UDP가 도커로 보내어 지고 있는지 확인하는 도구:

`docker-compose` 파일

```yaml
version: '3'

services:
  udp:
    container_name: udp-listener
    image: mendhak/udp-listener
    environment:
      - UDPPORT=14550
    ports:
      - "0.0.0.0:14550:14550"
      - "0.0.0.0:14550:14550/udp"
```

그리고:

```sh
docker compose up
```

# 참조

- https://hub.docker.com/r/mendhak/udp-listener
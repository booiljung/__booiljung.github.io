# DevContainer에서 UDP bind

3시간을 보낸 끝에 해결하였다.

Dockerfile 구성:

```dockerfile
EXPOSE 14550 14550/udp
EXPOSE 14560 14560/udp
```

docker-compose 구성:

```yaml
    ports:
      - 14550:14550/udp
      - 14560:14560/udp
```

.devcontainer/devcontainer.json 구성:

```json
"appPort": [ "14550:14550", "14550:14550/udp", "14560:14560", "14560:14560/udp" ]
```

## 참조

- https://community.home-assistant.io/t/receiving-udp/234042/4
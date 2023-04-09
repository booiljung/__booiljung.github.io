# DevContainer에 docker-comopose 적용

`./devcontainer/devcontainer.json`에 `docker-compose.yml` 경로와 서비스를 지정:

```
{
	"name": "<devcontainer name>",
	"dockerComposeFile": [
		"docker-compose.yml"
	],
	"service": "<service name>",
	"workspaceFolder": "/workspaces/${localWorkspaceFolderBasename}",
}
```

docker-compose 파일:

```
version: '3'
services:
  <service name>:
    container_name: <container name>
    build:
      context: .
      dockerfile: Dockerfile
    volumes:
      - ../..:/workspaces:cached
      - /var/run/docker.sock:/var/run/docker-host.sock
    command: /bin/bash -c "while sleep 1000; do :; done"    
```

Dockerfile:

```dockerfile
FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-20.04

...

```
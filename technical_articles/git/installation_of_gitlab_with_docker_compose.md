# Docker-Compose로 Ubuntu에 GitLab 설치

## 주의사항

- 백업 파일 버전과 gitlab 파일 버전이 일치해야 한다.
- 백업 파일에서 마이그레이션은 지원하지 않으니 주의!
- dockerhub에 특정 버전 태그가 목록에 없지만 실제로 있을 수도 있다.

- [dockerhub gitlagb ce](https://hub.docker.com/r/gitlab/gitlab-ce/tags?page=1&ordering=last_updated)

## 준비

- 우분투 18.04 LTS
- 도커 사용자

root 계정으로 쟉업한다.

```
sudo passwd root
...
su
```

```
mkdir /srv/gitlab
cd /srv/gitlab/
```

다음 파일을 생성하고 편집

```
sudo gedit docker-compose.yaml
```

다음 내용. 버전, 호스트 이름, URL을 설정하고 저장 한다.

```
web:
  #image: 'gitlab/gitlab-ce:latest'
  image: 'gitlab/gitlab-ce:13.7.1.ce.0' # 백업된 파일 버전에 맞는 버전을 선택한다.
  restart: always
  hostname: 'git.sm'
  environment:
    GITLAB_OMNIBUS_CONFIG: |
      external_url 'http://git.sm'
      # Add any other gitlab.rb configuration here, each on its own line
  ports:
    - '80:80'
    - '443:443'
    - '22:22'
  volumes:
    - '/srv/gitlab/config:/etc/gitlab'
    - '/srv/gitlab/logs:/var/log/gitlab'
    - '/srv/gitlab/data:/var/opt/gitlab'
    - '/srv/gitlab/backups:/var/opt/gitlab/backups' # 이 볼륨은 백업 폴더를 링크 하도록 한다.
```

작성하였으면 docker-compose로 올린다.

```
docker-compose up -d
```

gitlab 업그레이드

```
docker-compose pull # 업그레이드
docker-compose up -d # 컨테이너 올리기
```

컨테이너 내리기

```
docker-compose down
```

컨테이너 제거

```
docker-compose rm
```

gitlab 제거 후 다시 올려도 데이터 보존 및 동작 확인

```
docker-compose up -d
```

상태 확인

```
docker exe -it <name of container> gitlab-ctl status
```

각 프로세스들이 run 상태이면 정상이다.

## 백업과 복원

### 백업

```
gitlab-backup create
```

### 백업 복원

주의: 백업 파일 버전과 gitlab 파일 버전이 일치해야 한다.

컨테이너 보기

```sh
docker ps
```

데이터 베이스에 연결되는 프로세스를 중단시킨다.

```
docker exec -it <name of container> gitlab-ctl stop unicorn
docker exec -it <name of container> gitlab-ctl stop puma
docker exec -it <name of container> gitlab-ctl stop sidekiq
```

해당 프로세스들이 중단 되었는지 확인 한다.

```
docker exec -it <name of container> gitlab-ctl status
```

백업 파일은 지정된 backups 폴더에 있어야 한다. 여기서 백업 파일 이름은 `해시_년_월_일_버.어.전_gitlab_backup.tar`인데 마지막 `_gitlab_backup.tar`는 제외하고 `해시_년_월_일_버.어.전`으로 지정한다.

```
docker exec -it <name of container> gitlab-backup restore BACKUP=11493107454_2018_04_25_10.6.4-ce
```

`gitlab-secrets.json`을 `config` 폴더에 리스토어 한다.

그리고, 컨테이너를 다시 시작한다.

```
docker restart <name of container>
```

잘 동작 하고 있는지 체크한다.

```
docker exec -it <name of container> gitlab-rake gitlab:check SANITIZE=true
```

## 이슈

현재 이 메일이 동작하지 않음

## 참조

https://www.44bits.io/ko/post/almost-perfect-development-environment-with-docker-and-docker-compose
https://docs.gitlab.com/12.10/omnibus/docker/#install-gitlab-using-docker-compose



# Appwrite 속을 살펴보기

Appwrite 도커 컨테이너는 아래와 같다.

| 이미지                  | 컨테이너                     | 포트                         | 진입점 |
| ----------------------- | ---------------------------- | ---------------------------- | ------ |
| appwrite/appwrite:1.0.1 | appwrite-worker-functions    | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-executor            | 80/tcp, 443/tcp              |        |
| traefik:2.7             | appwrite-traefik             | 80/tcp, 443/tcp              |        |
| appwrite/appwrite:1.0.1 | appwrite-usage-timeseries    | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite                     | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-usage-database      | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-builds       | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-deletes      | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-certificates | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-webhooks     | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-databases    | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-audits       | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-realtime            | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-maintenance         | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-schedule            | 80/tcp                       |        |
| appwrite/appwrite:1.0.1 | appwrite-worker-mails        | 80/tcp                       |        |
| appwrite/telegraf:1.4.0 | appwrite-telegraf            | 8092/udp, 8125/udp, 8094/tcp |        |
| mariadb:10.7            | appwrite-mariadb             | 3306/tcp                     |        |
| redis:7.0.4-alpine      | appwrite-redis               | 6379/tcp                     |        |
| appwrite/influxdb:1.5.0 | appwrite-influxdb            | 8086/tcp                     |        |


​                                                                                           

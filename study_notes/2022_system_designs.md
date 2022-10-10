# 요즘 설계 중인 시스템

개발을 더 빠르게, 초급도 빨리 배워서 개발 할 수 있는 환경을 구성 중이다.

| 서비스                        | 기능                                                         |           |
| ----------------------------- | ------------------------------------------------------------ | --------- |
| traefik proxy                 | 리버스 프록시 (HTTP 라우터) 서비스, 압축 등                  |           |
| kong, **apisix**              | api gateway, 인증, 모니터링, 로깅, 서버리스, 보안, 중재 등   |           |
| supabase                      | serverless, pg_graphql - 회사 생존 어려울 듯                 | 후순위    |
| appwrite, openwhisk, openfaas | x2 개발 퍼포먼스 향상이 크지 않음. 복잡한 쿼리 불가.         | 탈락      |
| **hasura**                    | serverless + graphql engine 서비스. aws, ms 같은 대기업들이 언급. | x5 기대중 |
| **open-runtimes**             | 함수 실행 환경 제공 서비스                                   |           |
| mailhog                       | 개발용 메일 서버 클라이언트 서비스                           |           |
| **minio**                     | aws s3 (simple storage service) 호환 오픈소스                |           |
| nhost-auth                    | 사용자 인증 api 제공 서비스.                                 |           |
| nhost-storage                 | minio 에 대한 api 제공 서비스.                               |           |
| **elasticsearch**             | 분산 검색 및 분석 엔진 (문서 빅데이터)                       |           |
| **prometheus**                | 서버 모니터링 서비스                                         |           |
| **influxdb**                  | 시계열 (time series) 빅데이터 DBMS                           |           |
| **telegraf**                  | 시계열 (time series) 데이터 수집 에이전트                    |           |
| **grafana**                   | 시계열 (time series) 데이터 시각화 서비스                    |           |
| mqtt                          | 장비간 메시지 송수신 서비스                                  |           |
| hadoop                        | 빅데이터 분산 처리 프레임워크                                |           |
| spark                         | 빅데이터 분석 프레임워크 및 언어                             |           |
| redis                         | 서버 측 메모리 캐시                                          |           |
| coturn                        | p2p 데이터 스트리밍                                          |           |

**hasura:**

- postgresql만 완벽하게 지원.
- sqlserver, bigquery 부분 지원.
- mysql 지원은 개발 중.

- 지원 프레임워크: java spring, python flask, fastapi, ts-zeit, ts-express, js-zeit, ts-express, ruby, kotlin

**open-runtimes:**

- 지원 언어: nodejs, python, ruby, php, dart, deno, swift, .net, c++, java
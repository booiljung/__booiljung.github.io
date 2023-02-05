# 그냥 Postgres를 모든 곳에 사용하세요.

페이스북 PostgreSQL 그룹에 [그냥 Postgres를 모든 곳에 사용하세요](https://news.hada.io/topic?id=8018&fbclid=IwAR2AKG0v5wBj1LjdzeTSo6ZS4r0qXNMzt0EKIPOu7fu99qMXmjcrjDOHJzM)라는 대담한 글이 링크 되었다. 작성자를 보니 다른 분도 아니고 레진 창업 멤버인 xguru다.

내용을 옮기자면 아래와 같다.

- Postgres는 (수백만명의 사용자까지) 수많은 백엔드 기술을 대체 가능
   → Kafka, RabbitMQ, Mongo, Redis,..
- 캐시에 Redis 대신 UNLOGGED 테이블에 TEXT 를 JSON 형으로 사용
  - 스토어드 프로시저로 데이터에 대한 만료기간을 설정
- 메시지큐(Kafka) : SKIP LOCKED
- 데이터 웨어하우스는 Postgres+TimescaleDB
- Mongo 대신 JSONB를 저장하고 검색 및 인덱싱
- pg_cron 으로 메일 전송 같은 CRON 데몬으로 사용
- Geospacial 쿼리에 사용
- Elastic 대신 Fulltext 검색에 사용
- DB내에서 JSON을 생성해서 서버사이드 코드 없이 API에 바로 전달하기
- GraphQL 어댑터로 GraphQL도 지원

이 내용을 검증해 보자.
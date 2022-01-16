# PostGIS

참조: https://postgis.net/docs/manual-3.0/postgis-ko_KR.html



## PostGIS란?

- PostgerSQL에 설치되어 공간 데이터 처리 지원
- 다양한 벡터 타입 지원
  - GeometryCollection
  - MultiPolygon
  - MutiLineString
  - MultiPoint
  - Polygon
  - LineString
  - Point
- raster 타입 지원
- 공간 쿼리 Function 지원
  - ST_Union
  - ST_Intersects
  - ST_Intersection
  - ST_DWithin 등
- 공간 인덱스 지원
  - GIST
- GeoSerer, QGIS, uDig 등 오픈소스 소프트웨어와 연동 지원

## 설치

- 생략

## 초기 설정

PostgreSQL 설치과 완료되면 postgres 유저가 생성된다.

PostgreSQL의 설정 파일과 데이터를 관리할 폴더를 생성하고 폴더의 소유자를 postgres로 변경한다:

```sh
sudo mkdir /pg_data
sudo chown postgres:postgres /pg_data
```

postgres 유저로 로그인 한다:

```sh
sudo su - postgres
```

PostgreSQL을 초기화 해준다. `initdb`를 실행할 때,  `-D data_dir` 옵션을 설정해야 원하는 폴더에 설정 파일과 데이터를 넣을 수 있다

```sh
/usr/pgsql-9.5/bin/initdb -D /pg_data -E UTF8 --locale=C
```




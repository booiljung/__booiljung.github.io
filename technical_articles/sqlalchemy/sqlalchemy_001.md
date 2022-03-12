## sqlalchemy 초급 001

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


[Up](../index.md)

# FastAPI

## 설치

```sh
sudo pip install --upgrade pip
sudo pip install fastapi
sudo pip install "uvicorn[standard]"
```

### 001.py

```python
from typing import Optional
from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/items/{item_id}")
def read_item(item_id: int, q: Optional[str] = None):
    return {"item_id": item_id, "q": q}
```

실행:

```sh
uvicorn 001:app --reload
```

Swagger:

```sh
http://127.0.0.1:8000/docs
```

ReDoc:

```
http://127.0.0.1:8000/redoc
```

vscode 001.rest:

```rest
@scheme = http
@hostname = 127.0.0.1
@port = 8000
@baseUrl = {{scheme}}://{{hostname}}:{{port}}

### @name getHelloWorld

GET {{baseUrl}}/ HTTP/1.1
Content-Type: application/json

### @name getItems

@item_id=112
@q=myquery
GET {{baseUrl}}/items/
	112
	?q=myquery HTTP/1.1
Content-Type: application/json
```

### 002.py

```python
from typing import Optional
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

@app.get("/")
def read_root():
	return {"Hello": "World"}

@app.get("/items/{item_id}")
def read_item(item_id: int, q: Optional[str] = None):
	return {"item_id": item_id, "q": q}

class Item(BaseModel):
	name: str
	price: float
	is_offer: Optional[bool] = None

@app.put("/items/{item_id}")
def update_item(item_id: int, item: Item):
	return {"item_name": item.name, "item_id": item_id}
```

vscode 002.rest:

```rest
@scheme = http
@hostname = 127.0.0.1
@port = 8000
@baseUrl = {{scheme}}://{{hostname}}:{{port}}

### @name putItem

@item_id=113
PUT {{baseUrl}}/items/{{item_id}} HTTP/1.1
Content-Type: application/json

{
    "name": "myname",
    "price": 12.50,
    "is_offer": true
}
```


# 다트 웹앱
## 다트 웹앱 시작하기

### 다트 설치

##### 저장소 등록

```
sudo apt-get update
sudo apt-get install apt-transport-https
sudo sh -c 'curl https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add -'
sudo sh -c 'curl https://storage.googleapis.com/download.dartlang.org/linux/debian/dart_stable.list > /etc/apt/sources.list.d/dart_stable.list'
```

##### 설치

```
sudo apt-get update
sudo apt-get install dart
```

### CLI 도구 설치

```
pub global activate webdev
pub global activate stagehand
```

### 웹앱 생성

```
mkdir quickstart
cd quickstart
stagehand web-simple
pub get
```

### 웹앱 실행

```
webdev serve
```




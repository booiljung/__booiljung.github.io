[Up](./index.md)

# NodeJS 설치

이 설치 안내서는 2023년 1월 24일에 업데이트 되었습니다.

## NVM

### 설치

```
sudo apt install build-essential libssl-dev
```

https://github.com/nvm-sh/nvm/releases 에서 설치하고자 하는 nvm 버전을 확인 합니다.

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.x.y/install.sh | bash
```

### 사용법

로컬에 설치된 버전 확인

```
nvm ls
```

릴리즈 된 전체 버전 확인

```
nvm ls-remote
```

릴리즈 된 전체 lts 버전 확인

```
nvm ls-remote --lts
```

새로운 버전 설치

```
nvm install <버전>
```

사용할 버전 지정

```
nvm use <버전>
```

## nodejs PPA에서 설치

웹 브라우저로 nodejs.org 에 접속합니다. 홈 화면에서 최신 버전과 LTS  버전에서 선택할 수 있습니다.

설치하고자 하는 버전을 확인 하고 설치 스크립트를 다운로드 합니다.

```
curl -sL https://deb.nodesource.com/setup_x.y -o ~/Downloads/nodesource_setup.sh
```

설치 스크립트 실행

```
sudo bash ~/Downloads/nodesource_setup.sh
```

설치

```
sudo apt insall nodejs
```

버전 확인 

```
node -v
```

환경 변수 설정

~/.profile이나 ~/.bashrc 에 다음을 설정 합니다.

```shell
export PATH=$PATH:~/node/bin
```

~/.profile 에 설정하였다면 다시 로그인 하거나

 ```shell
source ~/.profile
 ```

로 실행 하거나, 셸에서 직접  `export ...`로 입력하고 엔터를 눌러도 됩니다.

설치 확인

Node Package Manager의 버전을 확인하여 제대로 설치 되었는지 확인 할 수 있습니다.

```shell
npm -v
```



# SSH Key란

SSH는 암호화된 원격 접속 프로토콜이며, 서버에 접속 할때 비밀번호 대신 키를 제출하는 방식으로 비밀번호보다 높은 수준의 보안을 적용할 수 있고, 로그인 없이 자동으로 서버에 접속할 수 있게 한다.

SSH Key는 개인키(Private Key)와 공유키(Public Key)로 구성된다.

| 키의 분류 |              개인키               |           공유키            |
| :-------: | :-------------------------------: | :-------------------------: |
|   영어    |            Private Key            |         Public Key          |
| 키의 위치 | SSH 클라이언트가 설치된 로컬 머신 | SSH 서버가 설치된 원격 머신 |
| 키 파일명 |             `id_rsa`              |        `id_rsa.pub`         |

SSH Key를 발급하기 위해서는 `ssh-keygen`을 사용하여 rsa 타입으로 4096 비트 길이의 키를 생성한다:

```sh
ssh-keygen -t rsa -b 4096
```

```sh
Generating public/private rsa key pair.
```

다음은 Passphrase를 묻는데 키를 암호화는 비밀번호다:

```
Enter passphrase (empty for no passphrase)
```

키 생성시 저장할 디렉토리를 지정하지 않으면 기본 디렉토리인 `~/.ssh`에 저장 된다.

서버에 업로드 한다:

```sh
scp $HOME/.ssh/id_rsa.pub username@host:id_rsa.pub
```

서버에서 다운로드한 키를 autorized_keys에 추가한다:

```sh
cat $HOME/.id_rsa.pub >> $HOME/.ssh/authorized_keys
```










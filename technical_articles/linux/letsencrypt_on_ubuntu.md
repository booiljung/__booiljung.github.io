# Ubuntu에서 Let's Encrypt 인증서 자동으로 받기

## 테스트 환경

- Ubuntu 18.04.3 LTS

## 테스트를 위한 도메인 네임 받기

- https://my.freenom.com/clientarea.php 에 가입하고 도메인 이름을 만든다.
- 관리자 메뉴에서 등록한 도메인에 대해 기본 네임 서버를 등록 한다.
- DNS Management에서 A 레코드를 등록한다.
- www.booiljungtest01.tk를 공인IP 주소로 연결 한다.

## 네트워크 장비 확인

- 방화벽이 허용 되는지 확인 한다.
- 포트포워딩이 되는지 확인 한다.

## 인증서 설치할 환경에 nginx 설치

```
sudo apt install nginx
sudo service nginx status
```

`/etc/nginx/site-enabled/default` 를 변경하여 `registry.booiljungtest1.tk`, `mavlink.booiljungtest1.tk`, `p2p.booiljungtest1.tk` 서버를 추가하고
`/var/www/`에 root 파일들을 추가한다.

기본 서버의 server_name은 www.booiljungtest1.tk로 변경한다.

웹브라우저로 접속하여 확인해 본다.

## 인증서 받기

- https://letsencrypt.org/getting-started/ 를 확인.
- https://certbot.eff.org/instructions 에서 옵션을 서택하고 안내를 받는다. 

### 설치 예시 (ubuntu 18.04 2021년 1월)

snap 설치 및 업그레이드

```
sudo apt install snap
sudo snap install core; sudo snap refresh core
```

구버전 데비안 패키지 certbot이 설치 되어 있다면

```
dpkg -l certbot
sudp apt remove certbot
```

certbot 설치

```
sudo snap install --classic certbot
```

설치. 질의에 답을 한다.

```
sudo certbot --nginx
```

성공하면 다음이 표시 된다.

```
Requesting a certificate for www.booiljungtest1.tk
Performing the following challenges:
http-01 challenge for registry.booiljungtest1.tk
Waiting for verification...
Cleaning up challenges
Deploying Certificate to VirtualHost /etc/nginx/sites-enabled/default
Redirecting all traffic on port 80 to ssl in /etc/nginx/sites-enabled/default
```

인증서 (재)발급 테스트.

```
sudo certbot renew --dry-run
```

(재)발급 결과

```
Saving debug log to /var/log/letsencrypt/letsencrypt.log

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Processing /etc/letsencrypt/renewal/www.booiljungtest1.tk.conf
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Cert not due for renewal, but simulating renewal for dry run
Plugins selected: Authenticator nginx, Installer nginx
Account registered.
Simulating renewal of an existing certificate for www.booiljungtest1.tk
Performing the following challenges:
http-01 challenge for www.booiljungtest1.tk
Waiting for verification...
Cleaning up challenges

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
new certificate deployed with reload of nginx server; fullchain is
/etc/letsencrypt/live/www.booiljungtest1.tk/fullchain.pem
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Congratulations, all simulated renewals succeeded: 
  /etc/letsencrypt/live/www.booiljungtest1.tk/fullchain.pem (success)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
```

certbot 이 nginx/site-enabled/default 를 변경하여

```
server {
    if ($host = www.booiljungtest1.tk) {
        return 301 https://$host$request_uri;
    } # managed by Certbot

    listen 80 ;
    listen [::]:80 ;
    server_name www.booiljungtest1.tk;
    return 404; # managed by Certbot
}
```

추가한다.

인증서는

`/etc/letsencrypt/archive/*`

에 있다고 한다.


## 참조

https://happist.com/573990/%EC%B5%9C%EC%8B%A0-lets-encrypt-ssl-%EC%9D%B8%EC%A6%9D%EC%84%9C-%EB%B0%9C%EA%B8%89-%EB%B0%A9%EB%B2%95-3%EA%B0%80%EC%A7%80-%EC%A0%95%EB%A6%AC

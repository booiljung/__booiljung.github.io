# 도메인 네임별에 따라 전달하기

URL에 따라 서버에 전달하는 Reverse Proxy 예제는 많다. 그러나 도메인 네임에 따라 Reverse Proxy하는 예제나 문서는 찾을 수 없었다.

## 실험

1. 네게의 서버를 만들었다.
  - proxy : `192.168.0.150`
  - proxy1 : `192.168.0.151`
  - proxy2 : `192.168.0.152`
  - proxy3 : `192.168.0.153`
2. 네임서버에 proxy1.booil, proxy2.booil, proxy3.booil의 서버 IP주소 `192.168.0.150`를 반환하도록 하였다. 
3. proxy.me에 nginx를 설치하였다.
  - `/etc/nginx/sites-available/default`를 편집하여 다음을 추가 하였다.
```
server {
	listen 80;
	listen [::]:80;

	server_name proxy1.booil;

	location / {
		proxy_pass http://192.168.0.151:80;
	}
}


server {
	listen 80;
	listen [::]:80;

	server_name proxy2.booil;

	location / {
		proxy_pass http://192.168.0.152:80;
	}
}

server {
	listen 80;
	listen [::]:80;

	server_name proxy3.booil;

	location / {
		proxy_pass http://192.168.0.153:80;
	}
}

```
  - `/var/www/html/index.html`을 편집하였다.
```
Welcome to nginx proxy!

If you see this page, the nginx web server is successfully installed and working. Further configuration is required.

For online documentation and support please refer to nginx.org.
Commercial support is available at nginx.com.

Thank you for using nginx.
```

4. proxy1, proxy2, proxy1 각각의 서버에 nginx를 설치하고 
  - `/var/www/html/index.html`을 편집하였다.
```
Welcome to nginx proxy1, proxy2, proxy3!

If you see this page, the nginx web server is successfully installed and working. Further configuration is required.

For online documentation and support please refer to nginx.org.
Commercial support is available at nginx.com.

Thank you for using nginx.
```

5. 개발용 PC에서 proxy1.me, proxy2.me, proxy3.me에 접속하니 `192.168.0.151`, `192.168.0.152`, `192.168.0.153`에 의도대로 접속 된다.
6. 해당하는 도메인 네임이 없으면 proxy.booil에 접속된다.






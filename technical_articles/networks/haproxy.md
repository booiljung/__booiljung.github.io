# HAProxy 구성

## SSL 없이 라우팅

p2p.booiljungtest2.ml와 p2p.booiljungtest2.ml를 라우팅 하는 예제

```
global
	log /dev/log	local0
	log /dev/log	local1 notice
	chroot /var/lib/haproxy
	stats socket /run/haproxy/admin.sock mode 660 level admin expose-fd listeners
	stats timeout 30s
	user haproxy
	group haproxy
	daemon

	# Default SSL material locations
	ca-base /etc/ssl/certs
	crt-base /etc/ssl/private

	# Default ciphers to use on SSL-enabled listening sockets.
	# For more information, see ciphers(1SSL). This list is from:
	#  https://hynek.me/articles/hardening-your-web-servers-ssl-ciphers/
	# An alternative list with additional directives can be obtained from
	#  https://mozilla.github.io/server-side-tls/ssl-config-generator/?server=haproxy
	ssl-default-bind-ciphers ECDH+AESGCM:DH+AESGCM:ECDH+AES256:DH+AES256:ECDH+AES128:DH+AES:RSA+AESGCM:RSA+AES:!aNULL:!MD5:!DSS
	ssl-default-bind-options no-sslv3

defaults
	log	global
	mode	http
	option	httplog
	option	dontlognull
        timeout connect 5000
        timeout client  50000
        timeout server  50000
	errorfile 400 /etc/haproxy/errors/400.http
	errorfile 403 /etc/haproxy/errors/403.http
	errorfile 408 /etc/haproxy/errors/408.http
	errorfile 500 /etc/haproxy/errors/500.http
	errorfile 502 /etc/haproxy/errors/502.http
	errorfile 503 /etc/haproxy/errors/503.http
	errorfile 504 /etc/haproxy/errors/504.http

#frontend LOADBALANCER-01
#	bind *:80
#	mode http
#	default_backend WEBSERVERS-01
#
#backend WEBSERVERS-01
#        balance roundrobin
#        server web1 192.168.0.151:80 maxconn 4096
#        server web2 192.168.0.152:80 maxconn 4096      

frontend http-in
        bind *:80

        acl host_p2p hdr(host) -i p2p.booiljungtest2.ml
        acl host_registry hdr(host) -i registry.booiljungtest2.ml
        
        use_backend p2p_cluster if host_p2p
        use_backend registry_cluster if host_registry

backend p2p_cluster
        balance leastconn
        option httpclose
        option forwardfor
        cookie JSESSIONID prefix
        server node1 192.168.0.151:80 cookie A check

backend registry_cluster
        balance leastconn
        option httpclose
        option forwardfor
        cookie JSESSIONID prefix
        server node1 192.168.0.152 cookie A check        
```

## 참조

- https://techexpert.tips/ko/%ED%95%98%ED%94%84%EB%A1%9D%EC%8B%9C/%EC%9A%B0%EB%B6%84%ED%88%AC-%EB%A6%AC%EB%88%85%EC%8A%A4%EC%97%90-haproxy-%EC%84%A4%EC%B9%98/

- https://m.blog.naver.com/PostView.nhn?blogId=happy_jhyo&logNo=221185157239&proxyReferer=https:%2F%2Fwww.google.com%2F

- https://seanmcgary.com/posts/haproxy---route-by-domain-name

- https://blog.naver.com/ncloud24/221621051483


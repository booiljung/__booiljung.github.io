# Ubuntu에 bind9 네임서버 구성

인터넷을 뒤져가며 여러 예제들을 보고 시도 하였으나 실패하였습니다.
매뉴얼은 500페이지가 넘어 엄두가 나지 않습니다.

## Ubuntu 18.04 LTS

### bind9

#### 설치

```
sudo apt install bind9
```

설치하면 자동으로 시작

#### 구성

`/etc/bind/named.conf.options` 변경

```
options {
	directory "/var/cache/bind";

	forwarders {
		168.126.63.1;
		168.126.63.2;
		8.8.8.8;
		8.8.8.4;
	};

	dnssec-validation auto;
	auth-nxdomain no;    # conform to RFC1035
	listen-on-v6 { any; };
	allow-query { any; };
	recursion yes;
};
```

`/etc/bind/named.conf.local` 변경

```
zone "booil.kr" {
	type master;
	file "/etc/bind/db.booil.kr";
	allow-update { any; };
	allow-transfer { any; };
	notify no;
};

zone "0.168.192.in-addr.arpa" {
	type master;
	file "/etc/bind/db.booil.kr.rev";
	allow-update { any; };
	allow-transfer { any; };
};
```

`/etc/bind/db.booil.kr` 생성
 
```
$TTL		604800
@		IN	SOA	ns.booil.kr. user.booil.kr. (
			      2		; Serial
			 604800		; Refresh
			  86400		; Retry
			2419200		; Expire
			 604800 )	; Negative Cache TTL
booil.kr.	IN	NS	ns.booil.kr.
booil.kr.	IN	A	192.168.0.253

; ordered by ALPHBET
docker		IN	A	192.168.0.251
file		IN	A	192.168.0.252
gateway		IN	A	192.168.0.1
git		IN	A	192.168.0.104
ns		IN	A	192.168.0.253

```

`/etc/bind/db.sm.rev` 생성

```
$TTL	604800
@	IN	SOA	ns.booil.kr. user.booil.kr. (
			     42		; Serial
			 604800		; Refresh
			  86400		; Retry
			2419200		; Expire
			 604800 )	; Negative Cache TTL

; ordered by		ALPHBET
251	IN	PTR	docker.booil.kr.
252	IN	PTR	file.booil.kr.
1	IN	PTR	gateway.booil.kr.
104	IN	PTR	git.booil.kr.
253	IN	PTR	ns.booil.kr.
```

#### 변경사항 업데이트

(필수) 네임 데몬 리로드

```
sudo service bind9 reload
```

(선택) 네임 데몬 재시작

```
sudo service bind9 restart
```

(선택) 네임 데몬 상태 보기

```
sudo service bind9 status
```

(선택) 네트워크 재시작

```
sudo service networking restart
```

(필수) 캐시 삭제
```
rndc flush
```

(필수) 캐시 리로드
```
rndc reload
```

(필수) 캐시 덤프
```
rndc dumpddb -cache
```

(선택) 테스트

```
$ nslookup
> git.smsoft.co.kr
```

## 라우터 연동

- 라우터 DNS 항목에 네임 서버 IP 적용
- 고정 IP 사용자는 DNS 항목에 네임 서버 IP 적용

## 참조

https://qastack.kr/ubuntu/330148/how-do-i-do-a-complete-bind9-dns-server-configuration-with-a-hostname



# minikube start

[minikube start](https://minikube.sigs.k8s.io/docs/start/)를 따라하며 메모

Debian 배포판에 설치:

```sh
$ curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube_latest_amd64.deb
$ sudo dpkg -i minikube_latest_amd64.deb
```

의존성 설치:

```sh
$ minikube start
```

단축 명령:

```sh
$ alias minikube kubectl="minikube kubectl --"
```

웹 대시보드:

```sh
$ minikube dashboard
```

## Service를 통한 애플리케이션 배포

`kicbase/echo-server:1.0`를 배포하고 8080포트로 서비스:

```sh
$ kubectl create deployment hello-minikube --image=kicbase/echo-server:1.0
$ kubectl expose deployment hello-minikube --type=NodePort --port=8080
```

서비스 목록 보기:

```shell
$ kubectl get services
```

지정하여 서비스 목록 보기:

```sh
$ kubectl get services hello-minikube
```

브라우저를 통해 서비스 동작:

```sh
$ minikube service hello-minikube
```

포트 포워딩:

```sh
$ kubectl port-forward service/hello-minikube 7080:8080
```

## LoadBalancer를 통한 애플리케이션 배포

잘 되지 않음

## Ingress를 통한 애플리케이션 배포

ingress 애드온 설치:

```sh
$ minikube addons enable ingress
```

다음 코드를 `ingress-example.yml`에 저장:

```yaml
kind: Pod
apiVersion: v1
metadata:
  name: foo-app
  labels:
    app: foo
spec:
  containers:
    - name: foo-app
      image: 'kicbase/echo-server:1.0'
---
kind: Service
apiVersion: v1
metadata:
  name: foo-service
spec:
  selector:
    app: foo
  ports:
    - port: 8080
---
kind: Pod
apiVersion: v1
metadata:
  name: bar-app
  labels:
    app: bar
spec:
  containers:
    - name: bar-app
      image: 'kicbase/echo-server:1.0'
---
kind: Service
apiVersion: v1
metadata:
  name: bar-service
spec:
  selector:
    app: bar
  ports:
    - port: 8080
---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: example-ingress
spec:
  rules:
    - http:
        paths:
          - pathType: Prefix
            path: /foo
            backend:
              service:
                name: foo-service
                port:
                  number: 8080
          - pathType: Prefix
            path: /bar
            backend:
              service:
                name: bar-service
                port:
                  number: 8080
---
```

`ingress-example.yml` 적용:

```sh
$ kubectl apply -f ingress-example.yaml
```

Ingress 정보 보기:

```sh
$ kubectl get ingress
NAME              CLASS   HOSTS   ADDRESS      PORTS   AGE
example-ingress   nginx   *       172.17.0.2   80      38s
```

curl로 테스트:

```sh
$ curl <ip_from_above>/foo
Request served by foo-app
...
$ curl <ip_from_above>/bar
Request served by bar-app
...
```













## 클러스터 관리

중지:

```sh
$ minikube pause
```

재개:

```sh
$ minikube unpause
```

중단 :

```sh
$ minikube stop
```

메모리 제한:

```sh
$ minikube config set memory 9001
```

이전 k8s 릴리즈 버전으로 두번째 클러스터 시작:

```sh
$ minikube start -p aged --kubernetes-version=v1.16.1
```

모든 클러스턱 제거:

```sh
$ minikube delete --all
```


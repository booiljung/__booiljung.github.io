# Minikube

Minikube는 학습과 개발을 위한 로컬 컴퓨터용 k8s 이며 `minikube start`로 시작 할 수 있습니다.

데비안 패키지로 설치 방법:

```sh
$ curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube_latest_amd64.deb
$ sudo dpkg -i minikube_latest_amd64.deb
```

클러스터 시작:

```sh
$ minikube start
W0317 00:17:10.453963   41968 main.go:291] Unable to resolve the current Docker CLI context "default": context "default" does not exist
😄  minikube v1.29.0 on Ubuntu 20.04
✨  Automatically selected the docker driver. Other choices: qemu2, virtualbox, none, ssh
📌  Using Docker driver with root privileges
👍  Starting control plane node minikube in cluster minikube
🚜  Pulling base image ...
💾  Downloading Kubernetes v1.26.1 preload ...
    > preloaded-images-k8s-v18-v1...:  397.05 MiB / 397.05 MiB  100.00% 25.42 M
    > gcr.io/k8s-minikube/kicbase...:  407.19 MiB / 407.19 MiB  100.00% 9.50 Mi
🔥  Creating docker container (CPUs=2, Memory=2200MB) ...- E0317 00:18:06.257310   41968 network_create.go:102] failed to find free subnet for docker network minikube after 20 attempts: no free private network subnets found with given parameters (start: "192.168.49.0", step: 9, tries: 20)

❗  Unable to create dedicated network, this might result in cluster IP change after restart: un-retryable: no free private network subnets found with given parameters (start: "192.168.49.0", step: 9, tries: 20)
🐳  Preparing Kubernetes v1.26.1 on Docker 20.10.23 ...- E0317 00:18:18.764710   41968 start.go:131] Unable to get host IP: network inspect: docker network inspect minikube --format "{"Name": "{{.Name}}","Driver": "{{.Driver}}","Subnet": "{{range .IPAM.Config}}{{.Subnet}}{{end}}","Gateway": "{{range .IPAM.Config}}{{.Gateway}}{{end}}","MTU": {{if (index .Options "com.docker.network.driver.mtu")}}{{(index .Options "com.docker.network.driver.mtu")}}{{else}}0{{end}}, "ContainerIPs": [{{range $k,$v := .Containers }}"{{$v.IPv4Address}}",{{end}}]}": exit status 1
stdout:
stderr:
Error response from daemon: network minikube not found
    ▪ Generating certificates and keys ...
    ▪ Booting up control plane ...
    ▪ Configuring RBAC rules ...
🔗  Configuring bridge CNI (Container Networking Interface) ...
    ▪ Using image gcr.io/k8s-minikube/storage-provisioner:v5
🔎  Verifying Kubernetes components...
🌟  Enabled addons: storage-provisioner, default-storageclass
💡  kubectl not found. If you need it, try: 'minikube kubectl -- get pods -A'
🏄  Done! kubectl is now configured to use "minikube" cluster and "default" namespace by default
```

클러스터 중단:

```sh
$ minikube stop
W0317 00:19:03.974387   49732 main.go:291] Unable to resolve the current Docker CLI context "default": context "default" does not exist
✋  Stopping node "minikube"  ...
🛑  Powering off "minikube" via SSH ...
🛑  1 node stopped.
```






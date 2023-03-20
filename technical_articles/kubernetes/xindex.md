# Kubernetes

## Kubespray

```sh
sudo apt update -y && sudo apt upgrade -y
sudo apt install python-pip -y
sudo apt install ansible
pip install --upgrade pip
git clone https://github.com/kubernetes-sigs/kubespray.git
cd kubespray
git checkout -b v2.11.2
git status
```

```sh
On branch v2.11.2
```

```sh
sudo pip install -r requirements.txt
```

`./inventory/sample/`에 설정에 필요한 기본 템플릿이 있다.  `./inventory/mycluster/`로 복사하여 변경한다.

```sh
cp ./inventory/sample/ ./inventory/mycluster/
tree ./inventory/mycluster/
```

```
group_vars	 					# 클러스터 설치에 필요한 설정 내용
--- all							# 설치 환경 및 방법에 대한 설정
    --- all.yml					# Kubespray 설치 및 설정
    --- azure.yml				# 애저 환경에 설치할 때 적용할 설정
    --- coreos.yml				# 코어OS 환경에 설치할 때 적용할 설정
    --- docker.yml				# 도커를 설치할 때 적용할 설정
    --- oci.yml					# 오라클 클라우드 환경에 설치할 때 적용할 설정
    --- openstack.yml			# 오픈스택 환경에 설치할 때 적용할 설정
--- etcd.yml					#
--- k8s-cluster
    --- addons.yml
    --- k8s-cluster.yml			# 쿠버네티스 클러스터를 설치할때 적용할 설정
    --- k8s-net-calico.yml
    --- k8s-net-canal.yml
    --- k8s-net-cilium.yml
    --- k8s-net-contiv.yml
    --- k8s-net-flannel.yml
    --- k8s-net-kube-router.yml
    --- k8s-net-macvlan.yml
    --- k8s-net-weave.yml
inventory.ini					# 설치대상 서버들의 정보
```


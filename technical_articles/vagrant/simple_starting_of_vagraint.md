[Up](./index.md)

# Vagrant 간단한 시작

## 설치

터미널에서 간단히 설치 할 수 있습니다.

```bash
sudo apt install -y vagrant
```

## 예제 프로젝트

프로젝트 경로를 만들고 이동 합니다.

```bash
mkdir -p vagrant_examples/getting_started
cd vagrant_examples/getting_started
```

Ubuntu 18.04 Bionic으로 프로젝트를 설정합니다.

```bash
vagrant init hashicorp/bionic64
```

우분투 18.04 box 설치

```bash
vagrant box add hashicorp/bionic64
```

box는 [HashiCorp's Vagrant Cloud box catalog](https://app.vagrantup.com/boxes/search)에서 볼 수 있다. 하면

```
==> box: Loading metadata for box 'hashicorp/bionic64'
    box: URL: https://vagrantcloud.com/hashicorp/bionic64
This box can work with multiple providers! The providers that it
can work with are listed below. Please review the list and choose
the provider you will be working with.

1) hyperv
2) virtualbox
3) vmware_desktop

Enter your choice: 
```

성능을 위해 1번 선택해 보았습니다.

그러면 https://vagrantcloud.com/hashicorp/boxes/bionic64/versions/1.0.282/providers/hyperv.box를 다운로드 합니다.

프로젝트 폴더에 `Vagrantfile`가 생성되고 내용은 다음과 같습니다.

```
Vagrant.configure("2") do |config|
  config.vm.box = "hashicorp/bionic64" # box 지정
  config.vm.box_version = "1.1.0" # 특정 버전을 지정
  config.vm.box_url = "https://vagrantcloud.com/hashicorp/bionic64" # 특정 URL의 box를 사용하도록 지정
end
```

vagrant 는 box가 실행될때 Vagrantfile을 자동으로 다운로드 합니다.

## Vagrant 게스트를 시작

```bash
vagrant up
```

그러면 Vagrantfile를 참조하여 box를 다운로드 하고 default 이름을 가진 vm이 만들어 지며 호스트와 게스트 사이의 연동 정보들이 표시 됩니다.

```
Bringing machine 'default' up with 'virtualbox' provider...
==> default: Importing base box 'hashicorp/bionic64'...
==> default: Matching MAC address for NAT networking...
==> default: Checking if box 'hashicorp/bionic64' is up to date...
==> default: Setting the name of the VM: getting_started_default_1593095533593_2577
==> default: Clearing any previously set network interfaces...
==> default: Preparing network interfaces based on configuration...
    default: Adapter 1: nat
==> default: Forwarding ports...
    default: 22 (guest) => 2222 (host) (adapter 1)
==> default: Booting VM...
==> default: Waiting for machine to boot. This may take a few minutes...
    default: SSH address: 127.0.0.1:2222
    default: SSH username: vagrant
    default: SSH auth method: private key
    default: 
    default: Vagrant insecure key detected. Vagrant will automatically replace
    default: this with a newly generated keypair for better security.
    default: 
    default: Inserting generated public key within guest...
    default: Removing insecure key from the guest if it's present...
    default: Key inserted! Disconnecting and reconnecting using new SSH key...
==> default: Machine booted and ready!
==> default: Checking for guest additions in VM...
    default: The guest additions on this VM do not match the installed version of
    default: VirtualBox! In most cases this is fine, but in rare cases it can
    default: prevent things such as shared folders from working properly. If you see
    default: shared folder errors, please make sure the guest additions within the
    default: virtual machine match the version of VirtualBox you have installed on
    default: your host and reload your VM.
    default: 
    default: Guest Additions Version: 6.0.10
    default: VirtualBox Version: 5.2
==> default: Mounting shared folders...
    default: /vagrant => /home/booil/linspace/vagrant_examples/getting_started

```

마지막을 보면 게스트 컴퓨터의 `/vagrant` 폴더가 호스트 컴퓨터 프로젝트 폴더에 마운트 되었다는 것을 확인 할 수 있다.

### Vagrant 게스트 컴퓨터에 접속하고 나오기

```bash
vagrant ssh
```

그러면 게스트 컴퓨터 로그인 된다.

```bash
Welcome to Ubuntu 18.04.3 LTS (GNU/Linux 4.15.0-58-generic x86_64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage

  System information as of Thu Jun 25 14:28:36 UTC 2020

  System load:  0.13              Processes:           91
  Usage of /:   2.5% of 61.80GB   Users logged in:     0
  Memory usage: 11%               IP address for eth0: 10.0.2.15
  Swap usage:   0%

 * "If you've been waiting for the perfect Kubernetes dev solution for
   macOS, the wait is over. Learn how to install Microk8s on macOS."

   https://www.techrepublic.com/article/how-to-install-microk8s-on-macos/

0 packages can be updated.
0 updates are security updates.

vagrant@vagrant:~$ 
```

유저 이름은 `vagrant` 입니다.

유저의 홈 경로를 확인해 봅니다.

```bash
vagrant@vagrant:~$ pwd
```

하면 

```
/home/vagrant
```

입니다.

게스트 컴퓨터에 마운트 된 호스트 컴퓨터의 프로젝트 폴더를 확인해 봅니다.

```bash
ls /vagrant
```

하면 

```
Vagrantfile
```

이 있을 것입니다.


작업이 끝나고 로그 아웃

```bash
logout
```

그러면 호스트의 셸로 돌아 온다.

### 게스트 컴퓨터 정지

```bash
vagrant suspend
```

### 게스트 컴퓨터 중단

```bash
vagrant halt
```

### default 게스트 컴퓨터 제거

```bash
vagrant destroy
```

# box를 제거하려면

```bash
vagrant box remove 
```

명령을 참조 합니다.

# 프로비저닝

프로젝트 폴더에 `bootstrap.sh` 파일을 만들고 내용을 채웁니다.

```sh
apt update
apt upgrade
apt install -y apache2
```

프로젝트 폴더의 `Vagrantfile`에 프로비저닝 파일을 지정합니다.

```
Vagrant.configure("2") do |config|
  config.vm.box = "hashicorp/bionic64"
  config.vm.provision :shell, path: "bootstrap.sh"
end
```

게스트 환경이 없다면 

```bash
vagrant up
```

으로 프로피저닝을 할 수 있고, 기존 게스트 컴퓨터에 프로비저닝을 변경한다면

```bash
vagrant reload --provision
```

을 하면 됩니다.

# 네트워킹

포트 포워딩은 `Vagrantfile`에 지정합니다.

```
Vagrant.configure("2") do |config|
  config.vm.box = "hashicorp/bionic64"
  config.vm.provision :shell, path: "bootstrap.sh"
  config.vm.network :forwarded_port, guest: 80, host: 4567
end
```

## 참조

- [Vagrant Documentation](https://www.vagrantup.com/docs/index)












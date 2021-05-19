# 앤서블 간단 안내서

**Ansible 이란:**

- 파이썬으로 만들어 졌다.
- 그래서, 느리다는 하소연이 많다.
- 원격의 컴퓨터를 구성-관리 한다.
- ssh를 통해 원격 컴퓨터에 명령한다. 원격 컴퓨터에 에이전트를 설치할 필요가 없다.
- 원격 컴퓨터에 앤서블 명령을 내리는 컴퓨터는 앤서블 코어가 설치 되어야 한다.

**앤서블 서버**

앤서블 코어가 설치된 컴퓨터를 말한다.

우분투에 앤서벌 코어 설치:

```bash
sudo apt install ansible -y
```

앤서블 명령어 목록 확인:

```sh
ls /usr/bin/ansible*
```

```
/usr/bin/ansible
/usr/bin/ansible-config
/usr/bin/ansible-connection
/usr/bin/ansible-console
/usr/bin/ansible-doc
/usr/bin/ansible-galaxy
/usr/bin/ansible-inventory
/usr/bin/ansible-pull
/usr/bin/ansible-playbook
/usr/bin/ansible-vaul
```

**앤서블 노드:**

앤서블 서버의 명령을 받아서 실행될 컴퓨터들

앤서블 서버의 `/etc/ansible/hosts` 파일에 컴퓨터 주소를 나열

```
192.168.1.11
192.168.1.12
...
192.168.1.30
192.168.1.31
```

앤서블 노드 연결 확인:

```sh
ansible all -m ping
```

이때 SSH 키값을 입력 받기 위해 `yes`를 입력

앤서블 명령 전달 확인:

```
ansible all -m ping -k
```

이때 암호를 물으면 암호를 입력 한다.

**앤서블 구성 파일:**

`/etc/ansible/ansible.cfg` 파일:

**앤서블 노드 그룹:**

`/etc/ansible/hosts`에 그룹 지정.

```
[그룹이름]
192.168.1.11
192.168.1.12
...
192.168.1.30
192.168.1.31

all 대신 노드 그룹을 지정하여 노드 그룹에 앤서블 명령 전달 확인:

```sh
ansible <그룹이름> -m ping -k
```

**인벤토리 파일 지정:**

`/etc/ansible/hosts`파일을 사용하지 않고 특정한 노드 목록 파일을 지정.

`-i <인벤토리 파일 경로>` 또는 `---inventory-file <인벤토리 파일 경로>` 로 지정.

```sh
ansible -i <인벤토리 파일 경로> ...
```

인벤토리 파일내의 모든 호스트에 대해 지정.

```sh
ansible -i <인벤토리 파일 경로> all ...
```

인벤토리 파일내의 특정 호스트에 대해 지정.

```sh
ansible -i <인벤토리 파일 경로> <호스트 주소> ...
```

**노드에 대한 패스워드를 물어 보도록 지정:**

```sh
ansible -k
또는
ansible --ask-pass
```

**적용되는 노드들 확인:**

실제로 앤서블 명령이 실행 되는 것이 아니라 명령 라인에 대해 적용되는 노드 목록만을 확인 한다. `-k`  같은 파라미터는 무시되며 지정하지 않아도 된다.

```sh
ansible --list-hosts
```

## 앤서블 애드혹

shell에서 명령줄로 주는 1회성 명령. 반복 사용할 수 있게 YAML로 작성하는 것을 플레이북이라고 함.

**사용할 모듈 지정:**

`-m` 또는 `--module-name`으로 사용할 모듈 이름을 지정

앞에서 사용하였던 `-m ping`은 `ping` 모듈을 사용 한 것

**shell 모듈:**

sh나 bash 같은 명령을 노드에서 직접 실행

```sh
ansible ... -m shell -a "<셸 명령줄>" ...
```

예를 들어 가동 시간을 확인:

```sh
ansible ... -m shell -a "uptime"
```

모듈 지정을 생략:

모듈을 지정하지 않으면 기본적으로 shell 모듈이기 때문에 `-m shell`은 생략하고 `-a "<셸 명령줄>"` 만을 지정 가능.

```sh
ansible ... -a "uptime"
```

명령줄에 따옴표 생략:

 `-a` 뒤의 셸 명령줄이 한개의 단어라면 따옴표에 넣지 않아도 된다.

```sh
ansible ... -a uptime
```

**사용자 모듈:**

사용자 출가:

```sh
ansible ... -m user -"name=<사용자 이름>" -k
```

마지막 추가된 사용자 확인:

```sh
ansible ... -a "tail -n 1 /etc/passwd" -k
```

사용자 제거:

```sh
ansible ... -m user - "name=<사용자 이름> state=absent" -k
```

**패키지 모듈:**

패키지 설치:

```sh
ansible ... -m <yum> -a "name=httpd state=present" -k
```

```sh
ansible ... -m <apt> -a "pkg=httpd update_cache=yes state=present" -k
```

패키지 제거:

```sh
ansible ... -m <yum> -a "name=httpd state=absent" -k
```

```sh
ansible ... -m <apt> -a "name=httpd autoremove=yes state=absent" -k
```

**파일 복사  모듈:**

앤서블 서버에서 앤서블 노드로 파일 복사.

```sh
ansible ... -m copy -a "src=<앤서블 서버 내의 원본 경로> dest=<앤서블 노드 내의 대상 경로>"
```

**서비스  모듈:**

서비스 시작, 중지, 재시작.

```sh
ansible ... -m service -a "name=<서비스 이름> state=<상태>"
```

서비스 상태:

- started:  시작
- stop: 중지
- restarted: 재시작

## 앤서블 플레이북

- 앤서블 애드혹은 1회성
- 앤서블 플레이북은 YAML로 작성
- 반복적으로 사용
- 플레이북 자체가 문서

명령줄에서 실행:

```sh
ansible-playbook <앤서플 플레이북 파일.yml> -k
```

```ansible
--- # 첫줄은 반드시 ---로 시작해야 함.
- name: <이 플레이북에 대한 설명>
  hosts: <호스트 지정>
  gather_facts: <yes 또는 no를 
  become: <아래 작업들에 대해 수퍼 유저 권한을 사용하려면 yes 아니면 no>
tasks:
   <이하 작업 내용들 기술>
```

**hosts:**

플레이북 파일을 실행할 호스트를 지정.

- `all`: /etc/ansible/hosts의 모든 노드들을 지정.
- `<노드 그룹 이름>`:  /etc/ansible/hosts에서 특정 그룹의 노드들을 지정.
- `localhost`: 앤서블 플레이북이 실행되고 있는 컴퓨터를 지정.

**수퍼유저 권한 획득하여 작업 수행:**

```
  tasks:
    - name: <설명>
      become: <수퍼 유저 권한을 사용하려면 yes 아니면 no>
```

**셸명령 실행:**

```
  tasks:
    - name: <설명>
      shell: "{{ item }}"
      with_items:
        - "<명령줄 1>"
        - "<명령줄 2>"
        - ...
```

**텍스트 파일 멱등성 라인 추가:**

```ansible
  tasks:
    - name: <설명>
      lineinfile:
        path: <파일경로>
        line: "{{ item }}"
      with_items:
        - "<라인 1>"
        - "<라인 2>"
        - ...
```

**파일 내에서 검색하여 치환:**

```
  tasks:
    - name: <설명>
      replace:
        path: <파일경로>
        regexp: <검색할 정규식>
        replace: <치환할 텍스트>
```

**호스트 추가:**

```ansible
  tasks:
    - name: <설명>
      blockinfile:
        path: /etc/ansible/hosts
        block: |
          [<그룹 이름>]
          <노드 주소 1>
          ...
          <노드 주소 2>
```

**패키지 설치:**

```ansible
  tasks:
    - name: <설명>
      yum:
        name: <패키지 이름>
        status: <present 또는 latest>
```

```
  tasks:
    - name: <설명>
      apt:
        pkg: <패키지 이름>
        update_cache: true
        status: <present 또는 latest>
```

패키지 제거:

```ansible
  tasks:
    - name: <설명>
      yum:
        name: <패키지 이름>
        status: absent
```

```
  tasks:
    - name: <설명>
      apt:
        pkg: <패키지 이름>
        status: absent
        autoremove: yes
```

**Git 저장소 클론:**

```ansible
  tasks:
    - name: <설명>
      git:
        repo: <git 저장소 URL>
        dest: <로컬 컴퓨터의 경로>
```

**.bashrc 구성:**

```ansible
  tasks:
    - name: <설명>
      lineinfile:
        path: /home/.../.bashrc
        line: "{{ item }}
      with_items:
        - "..."
        - "..."
```

**앤서블 서버에서 노드로 파일 복사하기:**

```
  tasks:
    - name: <설명>
      copy:
        src: "<앤서블 서버에서 원본 파일 경로>"
        dest: "<앤서블 노드에서 대상 파일 경로>"
        owner: <파일 소유 유저 지정>
        group: <파일 그룹 지정>
        mode: <파일 권한 지정>
```

**앤서블 노드에서 앤서블 서버로 파일 복사해오기:**

```
  tasks:
    - name: <설명>
      fetch:
        src: "<앤서블 노드에서 원본 파일 경로>"
        dest: "<앤서블 서버에서 대상 파일 경로>"
```

이때 앤서블 서버은 다수의 앤서블 노드에서 파일을 복사해 오므로 `<dest 경로>/<앤서블 노드 이름>/<src 경로>` 폴더에 저장 된다. 경로가 복잡하여 불편하면 다음과 같이 할 수 있다.

```
  tasks:
    - name: <설명>
      fetch:
        src: "<앤서블 노드에서 원본 파일 경로>"
        dest: "<앤서블 서버에서 대상 디렉토리>/{{ inventory_hostname }}-파일명.확장자"
        flat: true
```

`flat`을 `true`로 지정하여 앤서블 노드의 경로를 제거 한다.

**파일 내용 표시:**

```
  tasks:
    - name: <설명>
      debug:
        var: item
      with_file:
        - "<앤서블 노드에서 파일 경로 1>"
        - "<앤서블 노드에서 파일 경로 2>"
        ...
```

**디렉토리 만들기:**

```
  tasks:
    - name: <설명>
      file:
        state: directory
        path: <만들 디렉토리 경로>
        recurse: <기본값은 no이며 깊은 디렉토리를 만드러면 yes를 지정>
```

**다수의 디렉토리 만들기:**

아래는 `/tmp` 폴더에 `test-00` 부터 `test-99`까지 100개의 하위 디렉토리를 만든다.

```
  tasks:
    - name: <설명>
      file:
        state: directory
        path: "/tmp/test-{{ item }}"
        recurse: <기본값은 no이며 깊은 디렉토리를 만드러면 yes를 지정>
      with_sequence:
        start: 0
        end: 99
        format: %02d
```

아래는 `/tmp` 폴더에 지정한 다수의 디렉토리를 만든다.

```
  tasks:
    - name: <설명>
      file:
        state: directory
        path: "/tmp/test-{{ item }}"
        recurse: <기본값은 no이며 깊은 디렉토리를 만드러면 yes를 지정>
      with_items:
        - foo
        - bar
        - baz
```

**심볼릭 링크 만들기:**

```
  tasks:
    - name: <설명>
      file:
        state: link
        src: <원본 경로>
        path: <심볼링 링크 경로>
```

**셸에서 디렉토리 만들기:**

```ansible
  tasks:
    - name: <설명>
      shell: "{{ item }}"
      with_items:
        - "mkdir -p <생성할 디렉토리 이름>"
```

**파일 삭제:**

```
  tasks:
    - name: <설명>
      file:
        state: absent
        path: <앤서블 노드에서 파일 경로>
```

**디렉토리 보기:**

```
  tasks:
    - name: <설명>
      command: ls -lht <경로>
      register: <결과를 저장할 변수명>
    - name: <설명>
      debug:
        msg: "{{ <위에서 결과가 저장된 변수명> }}"
```

**URL을 통해 파일 가져오기:**

```
  tasks:
    - name: <설명>
      get_url:
        url: "<원본 URL>"
        dest: "<대상 로컬 경로>"
        mode: <파일 권한 지정>
```

리눅스 파일 권한:

- `0<owner><group><other>` 순서로 지정.
- `r: 4, w: 2, x: 1`
- 읽고 쓰고 실행 가능: `7`
- 읽기만 가능: `4`
- 읽고 쓰기만 가능: `6`
- 읽고 실행만 가능: `5`

**타임존 변경:**

```
  tasks:
    - name: <설명>
      timezone:
        name: Aisa/Seoul
```

리눅스 타임존 목록:

```sh
timedatectl list-timezones
```

**마운트:**

```
  tasks:
    - name: <설명>
      become: yes
      mount:
        path: <마운트 할 대상 경로>
        src: <마운트 할 소스 경로>
        fstype: <파일 시스템 타입>
        opts: <파일 시스템 옵션>
        state: mounted
```




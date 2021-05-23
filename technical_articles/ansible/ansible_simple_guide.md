# 앤서블 간단 안내서

**Ansible 이란:**

- 파이썬으로 만들어 졌다.
- 그래서, 느리다는 하소연이 있다.
- 원격의 컴퓨터를 구성-관리 한다.
- 원격 컴퓨터에 에이전트를 설치할 필요가 없다.
- ssh를 통해 원격 컴퓨터에 명령한다. 
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

앤서블 서버의 `/etc/ansible/hosts` 파일에 앤서블 노드를 나열

```
<앤서블 노드 1>
<앤서블 노드 2>
...
<앤서블 노드 n-1>
<앤서블 노드 n>
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
앤서블 노드 1
앤서블 노드 2
...
앤서블 노드 n-1
앤서블 노드 n
```

all 대신 노드 그룹을 지정하여 노드 그룹에 앤서블 명령 전달 확인:

```sh
ansible <그룹이름> -m ping -k
```

앤서블 노드 상세:

|                 형식                 |             설명             |      예제      |
| :----------------------------------: | :--------------------------: | :------------: |
|         `<앤서블 노드 주소>`         | 앤서블 노드 주소 지정 (필수) | `192.168.0.21` |
|      `ansible_user=<유저이름>`       |  앤서블 SSH 유저 이름 지정   |    `booil`     |
|        `ansible_connection=`         |                              |                |
| `ansible_network_os=<운영체제 이름>` |     노드의 운영체제 종류     |                |

**인벤터리 파일 지정:**

`/etc/ansible/hosts`파일을 사용하지 않고 특정한 노드 목록 파일을 지정.

`-i <인벤터리 파일 경로>` 또는 `---inventory-file <인벤터리 파일 경로>` 로 지정.

```sh
ansible -i <인벤터리 파일 경로> ...
```

인벤터리 파일내의 모든 호스트에 대해 지정.

```sh
ansible -i <인벤터리 파일 경로> all ...
```

인벤터리 파일내의 특정 호스트에 대해 지정.

```sh
ansible -i <인벤터리 파일 경로> <호스트 주소> ...
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

## 앤서블 모듈

- [앤서블 모듈 인덱스](https://docs.ansible.com/ansible/2.9/modules/modules_by_category.html)



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

**facts 확인**

```
ansible <노드> -m setup
```

노드별로 facts  따로 저장

```
ansible <노드> -m setup --tree
```

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
  gather_facts: <yes 또는 no> 
  become: <아래 작업들에 대해 수퍼 유저 권한을 사용하려면 yes 아니면 no>
tasks:
   <이하 작업 내용들 기술>
```

**facts를 수집하여 json으로 저장**

```
---
- name: <이름>
  hosts: <노드>
  
tasks:
  - name: <이름>
    setup:
    register: facts
    
  - name: save facts
    local_action:
      module: copy
      content: "{{ facts | to_nide_json }}"
      dest: ./{{ ansible_hostname }}_facts_by_collector.txt
```

**facts를 출력**

```
  - name: debug by msg
    debug:
      msg:
        - "eth0's ip {{ ansible_eth0.ipv4.adress }}"
        - "eth1's ip {{ ansible_eth1.ipv4.adress }}"
```

**작업 include**

```
---
- name: <이름>
  hosts: <노드들>
  become: yes
  taks:
    - include_taks: <서브 작업 파일>
```

서브 작업 파일

```
- name: <이름>
  <do something>
  ...
```

**조건에 의한 작업 수행**

구문을 모두 해석하므로 느림

```
tasks:
  - name: <이름>
    <뭔가 함>
    when: ansible_distribution == 'CentOS'
    또는
    when: ansible_distribution == 'Ubuntu'
```

**배포판 조건에 의한 작업 include**

필요 구문만 해석하므로 빠름

```
---
- name: <이름>
  hosts: <노드>
  vars:
    linux_distribution: "{{ 'centos' if ansible_distribution == 'CentOS' }}
                       else 'ubuntu' if ansible_distribution == 'Ubuntu'
                       else 'unknown' }}"
  
tasks:
  - name: <이름>
    include_tasks: "{{ linux_distribution }}.yml"
```

**Magic Variables**

기본 변수

|                    | 타입               | 매직변수                     |                          |
| ------------------ | ------------------ | ---------------------------- | ------------------------ |
| base               | connection         | ansible_connection           |                          |
|                    | module_compression | ansible_module_compression   |                          |
|                    | shell              | ansible_shell_type           |                          |
|                    | executable         | ansible_shell_executable     |                          |
| connection common  | remote_addr        | ansible_ssh_host             | ansible_host             |
|                    | remote_user        | ansible_ssh_user             | ansible_user             |
|                    | password           | ansible_ssh_pass             | ansible_password         |
|                    | port               | ansible_ssh_port             | ansible_port             |
|                    | pipelining         | ansible_ssh_pipelining       | ansible_pipelining       |
|                    | timeout            | ansible_ssha_timeout         | ansible_timeout          |
|                    | private_key_file   | ansible_ssh_private_key_file | ansible_private_key_file |
| networking modules | netwok_os          | ansible_network_os           |                          |
|                    | connection_user    | ansible_connection_user      |                          |
| become             | become             | ansible_become               |                          |
|                    | become_method      | ansible_become_method        |                          |
|                    | become_user        | ansible_become_user          |                          |
|                    | become_pass        | ansible_become_password      | ansible_become_pass      |
|                    | become_exe         | ansible_exe                  |                          |
|                    | become_flags       | ansible_become_flags         |                          |

**hosts:**

플레이북 파일을 실행할 호스트를 지정.

- `all`: /etc/ansible/hosts의 모든 노드들을 지정.
- `<노드 그룹 이름>`:  /etc/ansible/hosts에서 특정 그룹의 노드들을 지정.
- `localhost`: 앤서블 플레이북이 실행되고 있는 컴퓨터를 지정.

**유저 전환:**

수퍼 유저로 전환:

```
  tasks:
    - name: <이름>
      become: <수퍼 유저 권한을 사용하려면 yes 아니면 no>
```

지정한 유저로 전환:

```
  tasks:
    - name: <이름>
      become: <유저 권한을 사용하려면 yes 아니면 no>
      become_user: <유저 이름>
```

**앤서블 모듈:**

플레이북에서 작업을 수행하는 수단. 파일 복사, 다운로드, 패키지 설치 등 각 작업을 수행하는 기능을 제공.

**패스워드 없이 수행하기 위해 앤서블 서버와 노드사이의 인증 생성:**

```
---
- name: Create known_hosts between server and hosts
  host: nodes
  connection: local
  serial: 1

tasks:
  - name: ssh-keyscan for known_hosts file
    command: /usr/bin/ssh-keyscan -t ecdsa {{ ansible_host }}
    register: keyscan
    
  - name: input key
    lineinfile:
      path: ~/.ssh/known_hosts
      line: "{{ item }}"
      create: yes
    with_item:
      - "{{ keyscan.stdout_lines }}"
      
- name: Create authority_key between server and hosts
  host: nodes
  connection: local
  gather_facts: no
  vars:
    ansible_password: <비밀번호>   
      
  - name: ssh-keygen for authorized_key file
    command: "ssh-keygen -b 2048 -t rsa -f ~/ssh/id_rsa -q -N ''"
    ignore_errors: yes
    run_once: true
    
  - name: input key for each node
    connection: ssh
    authorized_key:
      user: <유저이름>
      state: present
      key: "{{ loookup('file', '~/.ssh/id_rsa.pub') }}"
```

**셸명령 실행:**

```
  tasks:
    - name: <이름>
      shell: "{{ item }}"
      with_items:
        - "<명령줄 1>"
        - "<명령줄 2>"
        - ...
```

**텍스트 파일 멱등성 라인 추가:**

```ansible
  tasks:
    - name: <이름>
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
    - name: <이름>
      replace:
        path: <파일경로>
        regexp: <검색할 정규식>
        replace: <치환할 텍스트>
```

다수의 검색어 치환:

```
  tasks:
    - name: <이름>
      replace:
        path: "{{ item.path }}"
        regexp: "{{ item.regexp }}"
        replace: "{{ item.replace }}"
      with_items:
        - { path: "<경로1>", regexp: "<검색 정규식1>", replace: "치환문자열1" }
        - { path: "<경로2>", regexp: "<검색 정규식2>", replace: "치환문자열2" }
```

**호스트 추가:**

```ansible
  tasks:
    - name: <이름>
      blockinfile:
        path: /etc/ansible/hosts
        block: |
          [<그룹 이름>]
          <호스트1>
          <호스트2>
          ...
          <호스트n-1>
          <호스트n>
```

**SSH 키 생성:**

```
  tasks:
    - name: <이름>
      become: yes
      become_user: <유저이름>
      shell: "{{ item }}"
      with_items:
        - "ssh-keyscan <IP주소> >> ~/.ssh/known_hosts"
        - "ssh-keyscan <IP주소> >> ~/.ssh/known_hosts"
        ...
        - "ssh-keyscan <IP주소> >> ~/.ssh/known_hosts"
        - "ssh-keyscan <IP주소> >> ~/.ssh/known_hosts"
```

**패키지 설치:**

레드헷:

```ansible
  tasks:
    - name: <이름>
      yum:
        name: <패키지 이름>
        status: <present 또는 latest>
```

레드헷 다수의 패키지 설치:

```
  tasks:
    - name: <이름>
      yum:
        name: " {{ item }}"
        status: <present 또는 latest>
        update_cache: true
      with_items:
        - "<패키지 이름 1>"
        - "<패키지 이름 2>"
        - ...
        - "<패키지 이름 n-1>"
        - "<패키지 이름 n>"
```

우분투:

```
  tasks:
    - name: <이름>
      apt:
        pkg: " {{ item }}"
        status: <present 또는 latest>          
        update_cache: true
      with_items:
        - "<패키지 이름 1>"
        - "<패키지 이름 2>"
        - ...
        - "<패키지 이름 n-1>"
        - "<패키지 이름 n>"
```

윈도우:

```
  tasks:
    - name: <이름>
      win_chocolatey:
        name: <패키지 이름>
```

공통: 

```
  tasks:
    - name: <이름>
      action: "{{ ansible_pkg_mgr }} name=<패키지이름> state=<present 또는 latest>"
```

**패키지 제거:**

레드햇:

```ansible
  tasks:
    - name: <이름>
      yum:
        name: <패키지 이름>
        status: absent
```

우분투:

```
  tasks:
    - name: <이름>
      apt:
        pkg: <패키지 이름>
        status: absent
        autoremove: yes
```

공통:

```
  tasks:
    - name: <이름>
      action: "{{ ansible_pkg_mgr }} name=<패키지이름> state=absent"
```

**git 저장소 클론:**

```ansible
  tasks:
    - name: <이름>
      git:
        repo: <git 저장소 URL>
        dest: <로컬 컴퓨터의 경로>
```

[**cmake로 빌드:**](https://github.com/mdklatt/ansible-cmake-module)

```
  tasks:
    - name: <이름>
      cmake:
        binary_dir: <빌드 디렉토리, 필수>
        source_dir: <소스 디렉토리>
        build_type: <빌드 타입, 기본 Debug>
        target: <빌드 타겟>
        cache_vars: <캐시 변수>
```

**.bashrc 구성:**

```ansible
  tasks:
    - name: <이름>
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
    - name: <이름>
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
    - name: <이름>
      fetch:
        src: "<앤서블 노드에서 원본 파일 경로>"
        dest: "<앤서블 서버에서 대상 파일 경로>"
```

이때 앤서블 서버은 다수의 앤서블 노드에서 파일을 복사해 오므로 `<dest 경로>/<앤서블 노드 이름>/<src 경로>` 폴더에 저장 된다. 경로가 복잡하여 불편하면 다음과 같이 할 수 있다.

```
  tasks:
    - name: <이름>
      fetch:
        src: "<앤서블 노드에서 원본 파일 경로>"
        dest: "<앤서블 서버에서 대상 디렉토리>/{{ inventory_hostname }}-파일명.확장자"
        flat: true
```

`flat`을 `true`로 지정하여 앤서블 노드의 경로를 제거 한다.

**파일 내용 표시:**

```
  tasks:
    - name: <이름>
      debug:
        var: item
      with_file:
        - "<앤서블 노드에서 파일 경로 1>"
        - "<앤서블 노드에서 파일 경로 2>"
        ...
```

**디렉토리 만들기:**

리눅스:

```
  tasks:
    - name: <이름>
      file:
        state: directory
        path: <만들 디렉토리 경로>
        recurse: <기본값은 no이며 깊은 디렉토리를 만드러면 yes를 지정>
```

윈도우:

```
  tasks:
    - name: <이름>
      win_file:
        state: directory
        path: <만들 디렉토리 경로>
        recurse: <기본값은 no이며 깊은 디렉토리를 만드러면 yes를 지정>
```

**다수의 디렉토리 만들기:**

아래는 `/tmp` 폴더에 `test-00` 부터 `test-99`까지 100개의 하위 디렉토리를 만든다.

```
  tasks:
    - name: <이름>
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
    - name: <이름>
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
    - name: <이름>
      file:
        state: link
        src: <원본 경로>
        path: <심볼링 링크 경로>
```

**셸에서 디렉토리 만들기:**

```ansible
  tasks:
    - name: <이름>
      shell: "{{ item }}"
      with_items:
        - "mkdir -p <생성할 디렉토리 이름>"
```

**파일 삭제:**

```
  tasks:
    - name: <이름>
      file:
        state: absent
        path: <앤서블 노드에서 파일 경로>
```

**디렉토리 보기:**

```
  tasks:
    - name: <이름>
      command: ls -lht <경로>
      register: <결과를 저장할 변수명>
    - name: <이름>
      debug:
        msg: "{{ <위에서 결과가 저장된 변수명> }}"
```

**URL을 통해 파일 가져오기:**

리눅스:

```
  tasks:
    - name: <이름>
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

윈도우:

```
  tasks:
    - name: <이름>
      win_get_url:
        url: "<원본 URL>"
        dest: "<대상 로컬 경로>"
```

**윈도우 서비스 등록:**

- 먼저 nssm이 설치되어 있어야 한다.

```
  tasks:
    - name: <이름>
      win_nssm:
        name: <애플리케이션 이름>
        application: <애플리케이션 실행파일 경로>
        state: present
```

**타임존 변경:**

리눅스:

```
  tasks:
    - name: <이름>
      timezone:
        name: Aisa/Seoul
```

리눅스 타임존 목록:

```sh
timedatectl list-timezones
```

윈도우:

```
  tasks:
    - name: <이름>
      win_timezone:
        timezone: 'Korea Standard Time'
```

**마운트:**

리눅스:

```
  tasks:
    - name: <이름>
      become: yes
      mount:
        path: <마운트 할 대상 경로>
        src: <마운트 할 소스 경로>
        fstype: <파일 시스템 타입>
        opts: <파일 시스템 옵션>
        state: mounted
```

윈도우 NFS 마운트:

```
  tasks:
    - name: <설명 1>
      win_feature:
        name: NFS-Client
        state: present
    - name: <설명 2>
      win_command: net use "드라이브 문자:" "NFS 경로"
```

**파일 압축 풀기:**

윈도우:

```
  tasks:
    - name: <이름>
      win_unzip:
        src: <압축된 파일 소스 경로>
        dst: <풀어진 파일 저장 경로>
        delete_archive: <압축 파일을 삭제하려면 yes 아니면 no>
```

**서비스 시작:**

리눅스:

```
  tasks:
    - name: <이름>
      service:
        name: <서비스 이름>
        state: started
```

윈도우:

```
  tasks:
    - name: <이름>
      win_service:
        name: <서비스 이름>
        state: started
```

**서비스 중지:**

리눅스:

```
  tasks:
    - name: <이름>
      service:
        name: <서비스 이름>
        state: stop
```

윈도우:

```
  tasks:
    - name: <이름>
      win_service:
        name: <서비스 이름>
        state: stop
```

**서비스 재시작**

리눅스:

```
  tasks:
    - name: <이름>
      service:
        name: <서비스 이름>
        state: restarted
```

윈도우:

```
  tasks:
    - name: <이름>
      win_service:
        name: <서비스 이름>
        state: restarted
```

**리부팅:**

윈도우:

```
  tasks:
    - name: <이름>
      win_reboot:
```

**서비스 데몬 재시작:**

```
  tasks:
    - name: <이름>
      systemd:
        state: restarted
        daemon_reload: yes
        name: tftp
```

**핸들러**

- 전 단계가 정상적으로 이루어진 경우에 동작
- 변경 사항이 발생한 경우에만 동작

핸들러:

```
handlers:
  - name: <핸들러 이름>
    <Do something>
```

핸들러 실행을 요청:

```
- name: <이름>
  <Do something>
  notify: <핸들러 이름>
```

## 참조

- https://www.ansible.com
- https://nirsa.tistory.com/281
- 우아하게 앤서블, 조훈, 김정민, 비제이퍼블릭
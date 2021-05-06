# Ansible Simple Guide

호스트 추가하기

```ansible
tasks:
  - name: Add "/etc/ansible/hosts"
    blockinfile:
      path: /etc/ansible/hosts
      block: |
        [Ubuntu]
        192.168.0.100
        192.168.0.101
        192.168.0.102
```

디렉토리 만들기

```ansible
tasks:
  - name: Make directory if not exists
    shell: "{{ item }}      
    with_items:
      - "mkdir -p directory"
```

패키지 설치하기

```ansible
tasks:
  - name: Install package
    yum:
      name: git
      status: present
```

```ansible
tasks:
  - name: Install package
    apt:
      name: git
      status: present
```

패키지 제거하기

```ansible
tasks:
  - name: Install package
    yum:
      name: git
      status: absent
```

```ansible
tasks:
  - name: Install package
    apt:
      name: git
      status: absent
```

Git 저장소 클론

```ansible
tasks:
  - name: Clone git repository
    git:
      repo: https://github.com/.../...
      dest: /home/.../.../
```

.bashrc 구성

```ansible
tasks:
  - name: Configure .bashrc
    lineinfile:
      path: /home/.../.bashrc
      line: "{{ item }}
    with_items:
      - "..."
      - "..."
```


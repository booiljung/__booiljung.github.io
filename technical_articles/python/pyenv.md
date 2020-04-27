# pyenv on Ubuntu

파이썬 언어는 간결하여 배우기 쉽고 간단하고 짧은 프로그램을 빠르게 작성할 수 있습니다. 파이썬의 문제는 다양한 파이썬 인터프리터/컴파일러와 버전이 있으며, 패키지 관리자가 엉망이라는 것입니다. 패키지 관리자가 엉망이라는 것은 dart의 pub와 비교해 보면 알 수 있을 것입니다. 일단 파이썬 인터프리터나 컴파일러가 pytion2, python3, cython, ipython, iron-python 등 다양하기도 할뿐만 아니라, 운영체제에 패키지의 버전도 하나밖에 설치할 수 없습니다. 프로젝트에 따라 여러가지 다른 버전의 패키지를 사용할 수도 있는데 기본 파이썬은 그렇지 않습니다.

하나의 운영체제에서 다양한 파이썬 인터프리터와 패키지 버전을 사용하기 위해 pyenv를 사용합니다.

## 설치

먼저 git을 설치 하고, github에서 클론합니다.

```sh
sudo apt update
sudo apt upgrade -y
sudo apt install git -y
git clone https://github.com/pyenv/pyenv.git ~/.pyenv
```

사용자 로그인 프로플에 환경 변수를 설정합니다.

```sh
echo '' >> ~/.profile
echo '# pyenv' >> ~/.profile
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.profile
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.profile
```

로그인시 `pyenv init` 를 추가 합니다.

```sh
echo '' >> ~/.profile
echo '# pyenv' >> ~/.profile
echo 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.profile
```

## 파이썬 인터프리터/컴파일러 설치하기

pyenv는 다양한 파이썬 인터프리터나 컴파일러를 설치해주는 도구입니다. 설치 할 수 있는 파이썬 버전을 확인해 보겠습니다.

```sh
pyenv install --list
```

설치할 수 있는 버전에서 필요한 버전을 설치 합니다.

```sh
pyenv install <version_name>
```

## 파이썬 가상환경

새 파이썬 가상환경 생성은 `virtualenv`를 사용합니다.

```sh
pyenv virtualenv <version_name> <virtual_env_name>
```

예를 들어 python 3.6.1 인터프리터를 사용하는 ml이라는 가상환경 이름으로 가상환경을 생성하면 아래와 같습니다.

```sh
pyenv virtualenv 3.6.1 ml
```

기 만들어진 가상환경에 들어가려면 `local`이라는 명령을 사용합니다. 이때 반드시 이 가상환경을 사용할 폴더에서 해야 합니다.

```
pyenv local <virtual_env_name>
```

예를 들어 새로 만들어진 `ml` 이라는 가상환경을 사용하겠습니다.

```sh
cd ~/myproject
pyenv local ml
```

이려면 prompt가 `(virtual_env_name)`으로 바뀝니다. 예를들어 `ml` 가상환경을 사용하면 아래와 같습니다.

```sh
~$ cd myproject
~\myproject$ pyenv local ml
(ml)~\myproject$
```

가상환경을 빠져나오려면 `deactive`를 사용합니다.

```sh
pyenv deactive
```

가상환경을 제거하려면 `uninstall`을 사용합니다. 

```sh
pyenv uninstall <virtual_env_name>
```

예를 들어 `ml`을 제거하려면 아래와 같습니다.

```sh
pyenv uninstall ml
```

## 참조

- github: [pyenv](https://github.com/pyenv/pyenv)
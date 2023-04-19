# pymavlink가 포함되었을때 pyinstaller 사용시 주의 사항.

Pymavlink가 포함된 파이썬 프로젝트를 pyinstaller로 빌드 할때는 몇개의 모듈과 히든 모듈을 포함해야 한다.

submodules:

- lxml
- future

hidden modules:

- pymavlink.dialects.v10 
- pymavlink.dialects.v10.ardupilotmega
- pymavlink.dialects.v20
- pymavlink.dialects.v20.ardupilotmega

```sh
pyinstaller --onefile ./src/main.py \
    --name swarmspy \
    --distpath ./dist \
    --workpath ./build \
    --runtime-tmpdir ./tmp \
    --collect-submodules=lxml \
    --collect-submodules=future \
    --hidden-import=pymavlink.dialects.v10 \
    --hidden-import=pymavlink.dialects.v10.ardupilotmega \
    --hidden-import=pymavlink.dialects.v20 \
    --hidden-import=pymavlink.dialects.v20.ardupilotmega
```


# 현재 경로를 실행하는 코드 작성하기

```sh
BASEDIR=$(pwd)
export GZ_SIM_SYSTEM_PLUGIN_PATH=$BASEDIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$BASEDIR/models:$BASEDIR/worlds:${GZ_SIM_RESOURCE_PATH}
source $BASEDIR/../../uav/ardupilot/Tools/completion/completion.bash
```


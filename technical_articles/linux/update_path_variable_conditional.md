# 조건부 PATH 변수 조작

```sh
if ! ${PATH} =~ "/autotest" ; then
	PATH="${PATH}:/home/booil/linspace/uav/ardupilot/Tools/autotest"
fi
```


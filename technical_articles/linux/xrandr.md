# xrandr 사용법

## xrandr

### 모니터 및 지원 해상도 목록 보기

```
xrandr
```

그러면 출력, 해상도, 주사율, 활성화된 목록을 확인 할 수 있다.

```
Screen 0: minimum 8 x 8, current 5760 x 2160, maximum 32767 x 32767
HDMI-0 disconnected (normal left inverted right x axis y axis)
HDMI-1 connected primary 3840x2160+1920+0 (normal left inverted right x axis y axis) 600mm x 340mm
   3840x2160     60.00*+  59.94    50.00    30.00    29.97    25.00    23.98
   ...
   640x480       59.94    59.93
HDMI-2 connected (normal left inverted right x axis y axis)
   640x400       59.94 +
   ...
   640x480       59.93  
DP-0 disconnected (normal left inverted right x axis y axis)
DP-1 disconnected (normal left inverted right x axis y axis)
DP-2 connected 1920x1080+0+643 (normal left inverted right x axis y axis) 527mm x 296mm
   1920x1080     60.00*+
   ...
   640x480       75.00    59.94  
DP-3 disconnected (normal left inverted right x axis y axis)
DP-4 disconnected (normal left inverted right x axis y axis)
DP-5 disconnected (normal left inverted right x axis y axis)
```

여기서 출력은 `HDMI-0`, `HDMI-1`, `HDMI-2`, `DP-0` 같은 것이다.

각 출력별 선택된 모드와 해상도에 *이 표시되어 있다.

### 다른 소프트웨어에 의해 출력이 꺼졌을때 켜는 방법

```
xrandr --output <출력> --auto
```

### 즉시 설정 방법

#### 해상도 변경

```
xrandr --output <출력> --mode 3840x2160 --rate 60.00
```

#### 듀얼모니터

 <주출력> 오른쪽에 위치하는 화면으로 만든다.

```
xrandr --output <상대출력> --mode 1024x768 --right-of <주출력>
```

주출력 화면을 Primary로 만든다.

```
xrandr --output <주출력> --primary
```

#### 스케일 HiDPI

```
xrandr --output <출력이름> --primary --pos 0x0 --scale 2x2
```

```
xrandr --dpi 120
```

#### 밝기 조절

밝기는 <0.0~1.0> 범위다.

```
xrandr --output <출력> --brightness <밝기>
```

### 강제 해상도 추가에 의한 방법

주사율을 얻는다.

```
gtf <수평해상도> <수직해상도> <리프레시>
```

그러면 설정값을 돌려 준다.

```
  # <수평해상도>x<수직해상도> @ <리플레시> Hz (GTF) hsync: 67.08 kHz; pclk: 172.80 MHz
  Modeline "<수평해상도>x<수직해상도>_<리프레시>"  172.80  1920 2040 2248 2576  1080 1081 1084 1118  -HSync +Vsync
```

여기서 `Modeline `이후 내용을 복사해 둔다. `"<수평해상도>x<수직해상도>_<리프레시>"  172.80  1920 2040 2248 2576  1080 1081 1084 1118  -HSync +Vsync`

새 모드를 생성한다.

```
xrandr --newmode "<수평해상도>x<수직해상도>_<리프레시>"  172.80  1920 2040 2248 2576  1080 1081 1084 1118  -HSync +Vsync
```

새 모드를 출력에 추가한다.

```
xrandr --addmode <출력> <수평해상도>x<수직해상도>_<리프레시>
```

새 모드로 전환 한다. (문제가 있는데 다른 출력도 `<수평해상도>x<수직해상도>_<리프레시>`를 지원하므로 다른 출력이 전환된다. 특정 출력을 지정하는 방법을 연구 할 필요가 있다.)

```
xrandr -s <수평해상도>x<수직해상도>_<리프레시>
```

## arandr

arandr은 xrandr의 GUI.

화면을 설정한 뒤에 `Save as…`로 셸 스크립트를 저장하고, 해당 스크립트를 시작 프로그램으로 등록하면 항상 원하는 화면으로 시작할 수 있다.

## autorandr

```
sudo apt install autorandr
autorandr
```

하드웨어 자동인식으로 `xrandr` 실행

# 새 Gnome Terminal에서 명령 실행하기

새 명령을,

Gnome Terminal 탭에서 명령을 실행:

```
gnome-terminal --tab --title "<헤드 타이틀>" --working-directory "<시작 디렉토리>" -- bash -c "<실행할 명령>; exec bash" &
```

`--tab`를 명시하지 않으면 새 창에서 실행 된다.

Terminator에서 명령을 실행:

```
terminator --new-tab --working-directory "<시작 디렉토리>" &
```

`--new-tab`을 명시하지 않으면 새 창에서 실행된다.
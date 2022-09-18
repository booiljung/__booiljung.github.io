# VSCODE Launcher

## 다수의 디바이스 디버깅하는 방법

```dart
{
	"version": "0.2.0",
	"configurations": [
    {
        "name": "Run Dev Android",
        "request": "launch",
        "deviceId": "SM",
        "type": "dart",
         "program": "lib/app/flavors/main_development.dart",
         "flutterMode": "debug",
         "args": [
             "--flavor", "development",
         ]
       
    },
    {
        "name": "Run Dev Iphone",
        "flutterMode": "debug",
        "deviceId": "Iphone",
         "program": "lib/app/flavors/main_development.dart",
         "type": "dart",
         "args": [
             "--flavor", "development",
         ]
    },
    {
        "name": "Run Dev",
        "program": "lib/app/flavors/main_development.dart",
        "flutterMode": "debug",
        "deviceId": "Android",
        "type": "dart",
        "args": [
            "--flavor", "development",
        ]
    },
    {
        "name": "Run Stage",
        "program": "lib/app/flavors/main_staging.dart",
        "flutterMode": "debug",
        "type": "dart",
        "args": [
            "--flavor", "staging"
        ]
    },
    {
        "name": "Run Prod",
        "program": "lib/app/flavors/main_development.dart",
        "flutterMode": "release",
        "type": "dart",
        "args": [
            "--flavor", "production"
        ]
    },
],
"compounds": [{
        "name": "All Devices",
        "configurations": ["Run Dev Android", "Run Dev Iphone"],
       
    }]

}
```

참조: https://stackoverflow.com/questions/50877842/vscode-and-flutter-how-to-connect-multiple-devices
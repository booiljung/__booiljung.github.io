# VSCODE launch.json

```json
{
    "version": "0.0.1",
    "configurations": [
        
        {
            "name": "cppdemo",
            "type": "cppdbg",
            "request": "launch",
            "cwd": "${workspaceRoot}/build/",
            "program": "${workspaceRoot}/build/cppdemo",
            "args": [
                "-n 27",
                "-u cam_left.yml",
                "-v cam_right.yml",
                "-L ../calib_imgs/1/",
                "-R ../calib_imgs/1/",
                "-l left",
                "-r right",
                "-o cam_stereo.yml",
                "-e jpg"
            ],
            "stopAtEntry": false,
            "environment": [
                {
                    "name": "PATH",
                    "value": "$PATH:$HOME/some_path"
                },
                {
                    "name": "OTHER_VALUE",
                    "value": "Something something"
                }
            ],
            "externalConsole": true,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```


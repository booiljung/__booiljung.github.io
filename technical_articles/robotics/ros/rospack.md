# [rospack](https://docs.ros.org/independent/api/rospkg/html/rospack.html)

```
rospack help
USAGE: rospack <command> [options] [package]
  Allowed commands:
    help
    cflags-only-I     [--deps-only] [package]
    cflags-only-other [--deps-only] [package]
    depends           [package] (alias: deps)
    depends-indent    [package] (alias: deps-indent)
    depends-manifests [package] (alias: deps-manifests)
    depends-msgsrv    [package] (alias: deps-msgsrv)
    depends-on        [package]
    depends-on1       [package]
    depends-why --target=<target> [package] (alias: deps-why)
    depends1          [package] (alias: deps1)
    export [--deps-only] --lang=<lang> --attrib=<attrib> [package]
    find [package]
    langs
    libs-only-L     [--deps-only] [package]
    libs-only-l     [--deps-only] [package]
    libs-only-other [--deps-only] [package]
    list
    list-duplicates
    list-names
    plugins --attrib=<attrib> [--top=<toppkg>] [package]
    profile [--length=<length>] [--zombie-only]
    rosdep  [package] (alias: rosdeps)
    rosdep0 [package] (alias: rosdeps0)
    vcs  [package]
    vcs0 [package]
  Extra options:
    -q     Quiets error reports.

 If [package] is omitted, the current working directory
 is used (if it contains a package.xml or manifest.xml).

```


# gradle error

**`Could not resolve all dependencies for configuration ':classpath`:**

다음을 `build.gradle`에 추가한다.

```
allprojects {
    repositories {
        google()
        jcenter()
    }
}
```


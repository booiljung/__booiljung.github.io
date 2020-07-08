# Qt 매크로

## 클래스

`Q_OBJECT`: 클래스가 메타오브젝트 기능을 사용할 수 있게 합니다.

```c++
class MyClass
{
     Q_OBJECT
	...
}
```

`Q_INVOKABLE`: QML에서 메서드를 호출 할 수 있습니다.

```c++
class MyClass
{
    ...
    Q_INVOKABLE QString name() const;
    ...
}
```


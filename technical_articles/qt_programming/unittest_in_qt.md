# UnitTest in Qt

## 테스트 만들기

테스트 클래스는 `QObject`를 상속하여 private slot들을 추가해야 합니다. private slot들은 테스트 함수들입니다. `QTest::qExec()` 함수에 개체를 전달하여 모든 테스트 함수들을 테스트 할 수 있습니다.

다음은 테스트를 준비하고 삭제하는 함수들입니다.

- `initTestCase()` 테스트 함수가 실행되기 전에 호출 됩니다. 실패하면 케이스 내의 모든 테스트 함수는 호출되지 않습니다.
- `initTestCase_data()` 전역 테스트 데이터를 생성하기 위해 호출 됩니다.
- `cleanupTestCase()` 마지막 테스트 함수가 실행된 다음에 호출됩니다.
- `init()` 각각의 테스트 함수들이 실행되기 전에 호출 됩니다. 실패하면 해당 테스트 함수는 호출되지 않으며, 다음 테스트로 진행 됩니다.
- `cleanup()` 각각의 테스트 함수들이 실행된 이후에 호출 됩니다.
- 테스트 함수는 `private slots:` 지정자 이어야 합니다.

```C++
class MyFirstTest: public QObject
{
    Q_OBJECT

private:
    bool myCondition()
    {
        return true;
    }

private slots:
    void initTestCase()
    {
        qDebug("Called before everything else.");
    }

    void myFirstTest()
    {
        QVERIFY(true); // check that a condition is satisfied
        QCOMPARE(1, 1); // compare two values
    }

    void mySecondTest()
    {
        QVERIFY(myCondition());
        QVERIFY(1 != 2);
    }

    void cleanupTestCase()
    {
        qDebug("Called after myFirstTest and mySecondTest.");
    }
};
```

### qmake 빌드 방법

테스트를 위해 프로젝트 파일에 다음을 추가 합니다.

```
QT += testlib
```

`make check`로 테스트 하기를 원하면 다음을 추가 합니다.

```
CONFIG += testcase
```

## Autotest 방법

QtCreator는 Qt Test, Google C++ Testing, Boost.Test 프레임워크를 통합하고 있습니다.

Qt Test:

1. File > New File or Project > Other Project > Auto Test Project > Choose 를 선택하여 테스트 프로젝트를 만듭니다. 테스트 boiler plate는 Qt test 나 Qt Quick test 중 선택 할 수 있습니다.

## QuickTest

QuickTest는 qml 파일을 테스트 합니다. 이때 qml 파일 이름은 반드시 `tst_*.qml` 이어야 하며, QuickTest 는 `pro`파일에 등록되지 않은 파일이라도 `tst_*.qml`파일을 검색하여 테스트 합니다.

## 참조

- https://doc.qt.io/qtcreator/creator-autotest.html
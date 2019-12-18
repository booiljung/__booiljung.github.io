# .NET SQLite dll failure

2019년 9월 3일

`SQLite 1.0.109`에서 정상으로 되었던 애플리케이션을 다시 불러와서 빌드하고 실행하는데 `System.DllNotFoundException: 'Unable to load DLL 'System.Data.SQLite' ... version 1.0.109'` 예외가 발생하며 동작하지 않습니다.

NuGet 패키지 매니저에서 `System.Data.SQLite 1.0.109`를 찾아보면 없습니다. 구글링한 결과 `System.Data.SQLite`를 설치하지 말고, `System.Data.SQLite.Core`를 설치하라고 합니다.

그래서, `System.Data.SQLite.Core`만을 설치하면 SQLite 어셈블리를 호출시 `Failed to add referene to Sqlite.Interop` 오류가 발생합니다. 그래서 NuGet 패키지 매니저에서 `SQLite.Interop`을 설치하려고 시도하였으나 권한이 없다며 설치되지 않습니다.

이 문제로 4일을 허비 했습니다. 수십가지 검색과 조합을 해가며 해결 방법을 찾아 비교하였으며, 이 글로 기록해 둡니다.

 비주얼 스튜디오에서 `Package Manager Console`에서 `Install-Package System.Data.SQLite` 로 패키지를 설치하여 이 문제를 해결 할 수 있었습니다.

## 참조

- [Error on add reference to SQLite.Interop.dll](https://stackoverflow.com/questions/33632471/error-on-add-reference-to-sqlite-interop-dll)




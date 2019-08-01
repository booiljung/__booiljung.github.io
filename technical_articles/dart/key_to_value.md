# Dart에서 키를 주고 값을 얻기

이렇게 `enum`으로 선언된  오류 코드가 있습니다. 멤버는 12개 입니다.

```dart
enum AuthRequestError {
  invalidRequest,
  invalidClient,
  invalidGrant,
  invalidScope,
  unsupportedGrantType,
  unsupportedResponseType,
  unauthorizedClient,
  accessDenied,
  serverError,
  temporarilyUnavailable,
  invalidToken
}
```

이 코드를 다루기 쉽게 문자열로 바꿀 필요가 있을 수도 있습니다.

다음은 `switch-case` 문을 사용하여 오류 코드를 문자열로 변환 합니다. 이 예제는 A번 입니다.

```dart
// A번코드
String errorString1(AuthRequestError error) {
  switch (error) {
    case AuthRequestError.invalidRequest:
      return "invalid_request";
    case AuthRequestError.invalidClient:
      return "invalid_client";
    case AuthRequestError.invalidGrant:
      return "invalid_grant";
    case AuthRequestError.invalidScope:
      return "invalid_scope";
    case AuthRequestError.invalidToken:
      return "invalid_token";

    case AuthRequestError.unsupportedGrantType:
      return "unsupported_grant_type";
    case AuthRequestError.unsupportedResponseType:
      return "unsupported_response_type";

    case AuthRequestError.unauthorizedClient:
      return "unauthorized_client";
    case AuthRequestError.accessDenied:
      return "access_denied";

    case AuthRequestError.serverError:
      return "server_error";
    case AuthRequestError.temporarilyUnavailable:
      return "temporarily_unavailable";
  }
  return null;
}
```

다음은 해시 테이블을 사용하여 오류 코드를 문자열로 변환 합니다. 이 예제는 B번 입니다.

```dart
// B번 코드
String errorString2(AuthRequestError error) {
  const _errorStrings = <AuthRequestError, String> {
    AuthRequestError.invalidRequest: "invalid_request",
    AuthRequestError.invalidClient: "invalid_client",
    AuthRequestError.invalidGrant: "invalid_grant",
    AuthRequestError.invalidScope: "invalid_scope",
    AuthRequestError.invalidToken: "invalid_token",
    AuthRequestError.unsupportedGrantType: "unsupported_grantType",
    AuthRequestError.unsupportedResponseType: "unsupported_response_type",
    AuthRequestError.unauthorizedClient: "unauthorized_client",
    AuthRequestError.accessDenied: "access_denied",
    AuthRequestError.serverError: "server_error",
    AuthRequestError.temporarilyUnavailable: "temporarily_unavailable",
  };
  return _errorStrings[error];
}
```

자. 질문입니다.

1. A번과 B번 어느 코드가 성능에 좋을까요?
2. 그 코드가 성능에 좋거나 다른 코드가 성능이 나쁜 이유는요?
3. 어느 코드가 가독성에 좋을까요?
4. 여러분은 주로 어떻게 하십니까?


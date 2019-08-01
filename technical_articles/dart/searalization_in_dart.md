# Dart에서 직렬화

Dart는 웹브라우저의 JavaScript를 대체하기 위해 탄생하였습니다. 그래서 Dart를 Json으로 직렬화를 주로 하게 됩니다.

## 간단한 Json 직렬화

`json_annotation`과 `json_serializable` 패키지를 설치 합니다.

```yaml
dependencies:
	equatable: any
	json_annotation: any
	
dev_dependencies:
	json_serializable: any
```

간단한 회원 클래스가 Json으로 직렬화를 지원하도록 구현해 보겠습니다.


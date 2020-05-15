# Visual Studio `Metadata file could not be found`

Visual Studio 2017 ~ 2019 프로젝트를 다른 컴퓨터로 옮기거나 VCS에서 클론을 하면 발생하는 오류 입니다.

`Metadata file XXX could not be found` 오류가 발생 합니다.

여러 방법을 시도 한 끝에 해결 방법을 알아 냈습니다.

1. Clean Solution을 합니다.
2. Solution Explorer에서 전체 프로젝트들을 선택하고 Unload Project를 합니다.
3. Visual Studio 를 종료 합니다.
4. Visual Studio를 다시 시작 합니다. 이러면 Solution Explorer에 노란 메시지와 링크가 표시됩니다. 링크를 클릭 합니다.
5. Visual Studio가 종료하고 Visual Studio Installation Managre가 실행되며 패키지들이 설치 될 것입니다.
6. Solution Explorer에서 전체 프로젝트들을 선택하고 Reload Project를 합니다.


# Deeplearning in C++ with CMake

Feat by cmake, Qt5, libtorch, QCustomPlot, OpenCV, dlib

세상에는 몇개의 선형대수 라이브러리들이 있습니다. 그중에 상업용 임베디드 시스템에 사용할만한 라이브러리들은 보지 못한것 같습니다. Facebook이 공헌중인 PyTorch는 Python에서 구동되며, C++을 위해 libtorch가 제공 됩니다.

이 실험은 libtorch로 신호처리(선형대수), 딥러닝 트레이닝 및 인퍼런스가 어디까지 가능한가 호기심에 시작하였습니다. 먼저 신호처리 문제를 풀어보고, 다음은 딥러닝 문제를 풀어 보도록 하겠습니다.

## 준비

- QtCreator를 사용하지 않고, 더 범용적인 CMake를 사용할 것입니다 .[이 글]()을 참조하여 설치합니다.

- C++용 PyTorch인 libtorch를 [이 글](installation_of_libtorch_and example.assets)을 참조하여 설치합니다.

- [이 글](../qt_programming/installation_of_qt5_on_ubuntu.md)을 참조하여 Qt5를 설치합니다. 아래의 QCustomPlot이 사용하게 됩니다.

- matplotlib.pyplot은 Python에서 matplot과 유사한 그래픽 드로잉을 제공합니다. C++에서는 QCustomPlot을 사용할 것인데  [이 글](../qt_programming/build_applications_with_qcustomplot.md)을 참조하여 준비합니다.

## 간단한 칼만필터

칼만필터는 다양한 자동 제어 분야에 사용됩니다. libtorch를 사용하여 간단한 칼만필터 문제를 플어 보도록 하겠습니다. 

$$
[Up](index.md)

# Kalman filter

## 역행렬 최적화

칼만 이득을 계산하는 식을 보겠습니다.
$$
\begin{align}
K_k = \breve P_k H ^\top (H \breve P_k H^\top + R)^{-1}
\end{align}
$$
역행렬(inverse matrix)이 사용되었습니다. 역행렬은 행렬식(determinant)를 사용해서 계산량이 많습니다. 가능한 역행렬을 피할 수 있다면 피하는 것이 좋습니다. 칼만 이득에서 역행렬을 제거 해 보겠습니다. 칼만 이득 계산식에서 제거는 불가능하며 시스템 설계 단계에서 제거할 수 있습니다.

먼저 $\breve P_k H$를 먼저 계산해 보겠습니다.
$$
\begin{align}
\breve P_k H ^\top
&=
\begin{bmatrix}
\breve P_{11} & \breve P_{12} \\
\breve P_{21} & \breve P_{22} \\
\end{bmatrix}
\begin{bmatrix}
1 \\
0 \\
\end{bmatrix}
\\
&=
\begin{bmatrix}
\breve P_{11} \\
\breve P_{21} \\
\end{bmatrix}
\end{align}
$$
다음은 나머지 역행렬 $(H \breve P_k H^\top + R)^{-1}$를 정리해보겠습니다.
$$
\begin{align}
H \breve P_k H^\top + R
&=
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
\breve P_{11} \\
\breve P_{21} \\
\end{bmatrix} + R
\\
&= \breve P_{11} + R
\end{align}
$$


간단히 정리되었습니다. 이 결과들을 칼만 이득 계산식에 적용해 보겠습니다.
$$
\begin{align}
K_k
&=
\breve P_k H ^\top (H \breve P_k H^\top + R)^{-1}
\\
&=
\begin{bmatrix}
\breve P_{11} \\
\breve P_{21} \\
\end{bmatrix}
(\breve P_{11} + R)^{-1}
\\
&=
\frac{1}{\breve P_{11} + R}
\begin{bmatrix}
\breve P_{11} \\
\breve P_{21} \\
\end{bmatrix}
\end{align}
$$
칼만 이득 계산식이 아주 단순해 졌습니다. 새 계산식을 C++ 코드로 구현해 보겠습니다.

```c++

```
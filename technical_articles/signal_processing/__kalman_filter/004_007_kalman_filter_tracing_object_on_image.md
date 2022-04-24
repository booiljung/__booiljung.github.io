[Up](index.md)

# Kalman filter

## 2D 이미지에서 개체 추적하기

상태 변수는 아래와 같습니다.
$$
\begin{aligned}
x = \begin{bmatrix}
\text{pos}_x\\
\text{vel}_x\\
\text{pos}_y\\
\text{vel}_y\\
\end{bmatrix}
\end{aligned}
$$
시스템 모델은 다음과 같습니다.

$$
\begin{aligned}
x_{k+1}	&= A x_k + w_k \\

z_k &= H x_k + v_k \\

A &= \begin{bmatrix}
	1 & \Delta t & 0 & 0 \\
	0 & 1 & 0 & 0 \\
	0 & 0 & 1 & \Delta t \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\

H &= \begin{bmatrix}
	1 & 0 & 0 & 0 \\
	0 & 0 & 1 & 0 \\
\end{bmatrix}
\end{aligned}
$$
행렬 $A$를 시스템 모델에 대입합니다.
$$
\begin{aligned}
x_{k+1} &= A x_k + w_k \\
&=
\begin{bmatrix}
	1 & \Delta t & 0 & 0 \\
	0 & 1 & 0 & 0 \\
	0 & 0 & 1 & \Delta t \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} x_k + w_k \\
\end{aligned}
$$
여기에 상태변수 정의를 대입해 보겠습니다.
$$
\begin{aligned}

\begin{bmatrix}
	\text{pos}_x \\
	\text{vel}_x \\
	\text{pos}_y \\
	\text{vel}_y \\
\end{bmatrix} _{k+1}

&=

\begin{bmatrix}
	1 & \Delta t & 0 & 0 \\
	0 & 1 & 0 & 0 \\
	0 & 0 & 1 & \Delta t \\
	0 & 0 & 0 & 1 \\
\end{bmatrix}

\begin{bmatrix}
	\text{pos}_x \\
	\text{vel}_x \\
	\text{pos}_y \\
	\text{vel}_y \\
\end{bmatrix} _{k}
+
\begin{bmatrix}
	0 \\
	w \\
	0 \\
	w \\
\end{bmatrix}_k \\

&= 

\begin{bmatrix}
	\text{pos}_x + \text{vel}_x \cdot \Delta t \\
	\text{vel}_x + w \\
	\text{pos}_y + \text{vel}_y \cdot \Delta t \\
	\text{vel}_y + w \\
\end{bmatrix} _k

\end{aligned}
$$
위치와 속도에 대한 식을 보면
$$
\begin{aligned}

\text{pos} _{k+1} 

&=

\text{pos} _{k} 
+
\text{vel}_k \cdot \Delta t

\end{aligned}
$$
입니다.  개체의 속도는
$$
\begin{aligned}

\text{vel} _{k+1} 

&=

\text{vel} _{k} 
+
w_k

\end{aligned}
$$
이며, 시스템 노이즈 $w_k$의 영향을 받습니다.

시스템 모델의 측정값 $z_k$를 보겠습니다.
$$
\begin{aligned}

z_k &= H x_k + v_k \\

&=
\begin{bmatrix}
	1 & 0 & 0 & 0 \\
	0 & 0 & 1 & 0 \\
\end{bmatrix}

\begin{bmatrix}
	\text{pos}_x \\
	\text{vel}_x \\
	\text{pos}_y \\
	\text{vel}_y \\
\end{bmatrix}_k + v_k \\

&=

\begin{bmatrix}
	\text{vel}_x \\
	\text{vel}_y \\
\end{bmatrix}_k + v_k \\

\end{aligned}
$$
노이즈의 공분산 행렬 $Q$, $R$을 정해야 하는데 노이즈 특성을 따라야 하며, 센서 제작사에서 제공하는 것을 기본으로 사용하고, 제공되지 않는다면 실험과 경험을 통해 결정해야 합니다.
$$
\begin{aligned}
Q
&=
\begin{bmatrix}
	1 & 0 & 0 & 0 \\
	0 & 1 & 0 & 0 \\
	0 & 0 & 1 & 0 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix}
\\
R
&=
\begin{bmatrix}
	50 & 0 \\
	0 & 50 \\
\end{bmatrix}
\end{aligned}
$$








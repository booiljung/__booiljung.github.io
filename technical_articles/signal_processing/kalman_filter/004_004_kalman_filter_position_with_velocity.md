[Up](index.md)

# Kalman filter

## 속도로 위치 얻기

속도로 위치를 얻어 보겠습니다. 

상태 변수는 아래와 같습니다.
$$
\begin{aligned}
x = \begin{bmatrix}
\text{position}\\
\text{velocity}\\
\end{bmatrix}
\end{aligned}
$$
상태변수 $H$가 다릅니다.
$$
\begin{aligned}
x_{k+1} &= A x_k + w_k \\
z_k &= H x_k + v_k \\
A &= \begin{bmatrix}
	1 & \Delta t\\
	0 & 1 \\
\end{bmatrix} \\
H &= \begin{bmatrix}
	0 & 1 \\
\end{bmatrix}
\end{aligned}
$$
행렬 $A$를 시스템 모델에 대입합니다.
$$
\begin{aligned}
x_{k+1} &= A x_k + w_k \\

&= \begin{bmatrix}
	1 & \Delta t\\
	0 & 1 \\
\end{bmatrix} x_k + w_k \\
\end{aligned}
$$
여기에 상태변수 정의를 대입해 보겠습니다.
$$
\begin{aligned}

\begin{bmatrix}
	\text{position} \\
	\text{velocity} \\
\end{bmatrix} _{k+1} 

&=

\begin{bmatrix}
	1 & \Delta t\\
	0 & 1 \\
\end{bmatrix} 

\begin{bmatrix}
	\text{position} \\
	\text{velocity} \\
\end{bmatrix}_k + 

\begin{bmatrix}
	0 \\
	w_k \\
\end{bmatrix} \\

&= 

\begin{bmatrix}
	\text{position} + \text{velocity} \cdot \Delta t \\
	\text{velocity} + w
\end{bmatrix} _k

\end{aligned}
$$
위치와 속도에 대한 식을 보면
$$
\begin{aligned}

\text{position} _{k+1} 

&=

\text{position} _{k} 
+
\text{velocity}_k \cdot \Delta t

\end{aligned}
$$
입니다. 배의 속도는
$$
\begin{aligned}

\text{velocity} _{k+1} 

&=

\text{velocity} _{k} 
+
w_k

\end{aligned}
$$
시스템 노이즈 $w_k$에 대해서만 영향을 받습니다.

시스템 모델의 측정값 $z_k$를 보겠습니다.
$$
\begin{aligned}

z_k &= H x_k + v_k \\

&=

\begin{bmatrix}
	0 & 1
\end{bmatrix}

\begin{bmatrix}
	\text{position} \\
	\text{velocity} \\
\end{bmatrix} + v_k
\\
&=

\text{velocity} _k + v_k
\end{aligned}
$$
노이즈의 공분산 행렬 $Q$, $R$을 정해야 하는데 노이즈 특성을 따라야 하며, 센서 제작사에서 제공하는 것을 기본으로 사용하고, 제공되지 않는다면 실험과 경험을 통해 결정해야 합니다.
$$
\begin{aligned}
Q
&=
\begin{bmatrix}
	1 & 0 \\
	0 & 3 \\
\end{bmatrix}
\\
R
&=
10
\end{aligned}
$$


```c++

```

호출은 아래와 같습니다.

```c++

```






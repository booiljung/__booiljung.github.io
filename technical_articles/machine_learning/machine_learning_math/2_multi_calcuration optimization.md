# 머신러닝을 위한 수학

## 다변수 미적분학과 최적화

##### 수렴, $f(x)$의 극한 (limit)

##### 미분가능 (differentiable)

##### 최대값 최소값의 정리

##### 적분 (integral)

##### 면적 (area), 리만 합 (Riemann sum), 적분값 (integral)

##### 적분에 대한 평균값의 정리

##### 미적분학의 기본정리 (적분과 미분의 연결고리)

$f(x)$가 $[a, b]$에서 연속이고 $F(x)$를 $f(x)$의 임의의 한 부분적분 $F'(x) = f(x)$이라 하면 다음식이 성립한다.
$$
\int_a^b f(x)\text dx
= F(b) - F(a)
$$

##### 벡터와 공간기하

##### 외적 (cross product)

##### C-S 부등식

##### 편도함수 (partial derivative)

$z=f(x, y)$가 $x$와 $y$의 함수라 하자. $f$의 $x$에 관한 편도함수는 다음과 같이 정의된다.
$$
\begin{align}
\frac{\partial z}{\partial x}
&= \frac{\partial f}{\partial x} \\
&= z_x \\
&= f_x(x, y) \\
&= \lim_{h \rightarrow 0} \frac{f(x + h, y) - f(x, y)}{h}
\end{align}
$$
마찬가지로 $f$의 $y$에 관한 편도함수(partial derivative)는 다음과 같이 정의된다.
$$
\begin{align}
\frac{\partial z}{\partial y}
&= \frac{\partial f}{\partial y} \\
&= z_y \\
&= f_y(x, y) \\
&= \lim_{h \rightarrow 0} \frac{f(x, y+h) - f(x, y)}{h}
\end{align}
$$

##### $f_{xy} = f_{yx}$되는 조건

##### 연쇄법칙 (chain rule)

##### 방향도함수 (directional derivative), 그래디언트 (gradient), 헤시안 (Hessian)

##### 방향도함수 (directional derivative)

##### 경사하강법 (gradient descent method)

##### Tayler 정리

##### 함수의 극대 (local maximum)

##### 함수의 극소 (local minimum)

##### 함수의 최대

##### 함수의 최소

##### Fermat의 임계점 정리 (Fermat's theorem on critical points)

##### 임계점 (critical point)

$\nabla f(a, b) = 0$을 만족하는 점 $(a, b)$를 $f$의 임계점 (critical point)이라 한다.

##### 극대, 극소, 안장점 (saddle point)

##### 극소, 극대 판정법

##### 경사기울기하강법 (gradient descent algorithm)

##### 이중적분

##### Fubini 정리

##### 연속함수의 2중적분의 경우, 적분영역이 직사각형이 아닌 경우

##### 야코비안 (Jacobian)

##### 이중적분이 변수변환

##### 극좌표 (polar coordinates)에서의 2중적분


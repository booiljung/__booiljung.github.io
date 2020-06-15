# 미분

## 상미분 (ordinary derivative)

변수가 하나인 함수를 미분 하는 것을 상미분이라고 합니다.
$$
\begin{aligned}
\frac{\mathrm df(a)}{\mathrm dx}
&= \lim_{\Delta x \rarr 0} \frac{\Delta f(a)}{\Delta x} \\
&= \lim_{\Delta h \rarr 0} \frac{f(a + h) - f(a)}{(a+h)-a} \\
&= \lim_{\Delta h \rarr 0} \frac{f(a+h)-f(a)}{h}
\end{aligned}
$$

### 공식

$$
\begin{aligned}
\frac{\mathrm d}{\mathrm dx}
\left\{
x^r
\right\}
= rx^{r-1}
\end{aligned}
\tag{1}
$$

$$
\begin{aligned}
\frac{\mathrm d}{\mathrm dx}
\left\{
f(x) + g(x)
\right\}
= \frac{\mathrm d f(x)}{\mathrm d x} + \frac{\mathrm d g(x)}{\mathrm d x}
\end{aligned}
\tag{2}
$$

$$
\begin{aligned}
\frac{\mathrm d}{\mathrm dx}
\left\{
kf(x)
\right\}
= k\frac{\mathrm d f(x)}{\mathrm d x}
\end{aligned}
\tag{3}
$$

## 편미분 (partial derivative)

다변수 함수의 특정 변수를 제외한 나머지 변수를 상수로 간주하고 미분하는 것이다.

다음 함수
$$
f(x, y, ...)
$$
에 대해 편미분은
$$
f'x
,\quad f_x
,\quad \partial_x f
,\quad \frac{\partial}{\partial x} f
,\quad \frac{\partial f}{\partial x}
$$
로 표기 합니다.

다음 함수가 있다.
$$
\begin{aligned}
y &= u \times v
\end{aligned}
$$
$v$를 상수로 취급하고 $u$에 대해 미분
$$
\begin{aligned}
\mathbb dy_u &= v \mathbb d u
\\
& = \frac{\partial y}{\partial u} \mathbb d u
\end{aligned}
$$
$u$를 상수로 취급하고 $v$에 대해 미분
$$
\begin{aligned}
\mathbb dy_v &= u \mathbb d v
\\
& = \frac{\partial y}{\partial v} \mathbb d v
\end{aligned}
$$
$y$의 변량은 두 조건이 동시에 만족될 때 성립한다.
$$
\begin{aligned}
\mathbb dy
&= 
\frac{\partial y}{\partial u} \mathbb d u
+
\frac{\partial y}{\partial v} \mathbb d v
\end{aligned}
$$

---

다음 
$$
\begin{aligned}

w = 2ax^2 + 3bxy + 4cy^3

\end{aligned}
$$
의 편도함수를 구하라

$y$를 상수로하여 편미분
$$
\frac{\partial w}{\partial y} =
4ax + 3by
$$
$x$를 상수로하여 편미분
$$
\frac{\partial w}{\partial x} =
3bx + 12cy^2
$$
그래서
$$
\mathbb d w =

(4ax + 3by) \mathbb d x
+
(3bx + 12cy^2) \mathbb d y
$$

---

|                              식                              |                 $\mathbf x$에 대한 미분결과                  |
| :----------------------------------------------------------: | :----------------------------------------------------------: |
| $\mathbf a \cdot \mathbf x = \mathbf a^\top \mathbf x = x^\top \mathbf a$ |                     $$ \mathbf a^\top$$                      |
|                    $\mathbf A \mathbb x$                     |                        $ \mathbf A $                         |
|                $ \mathbf x ^\top \mathbf A $                 |                      $ \mathbf A^\top $                      |
|           $ \mathbf x ^\top \mathbf A \mathbf x $            |      $ \mathbf x ^\top (\mathbf A + \mathbf A ^\top) $       |
|           $ \mathbf y ^\top \mathbf A \mathbf z $            | $ \mathbf y ^\top \mathbf A \frac{\partial \mathbf z}{\partial \mathbf x} + \mathbf z ^\top \mathbf A^\top \frac{\partial \mathbf x}{\partial \mathbf x} $ |
|   $ \mathbf y ^\top \mathbf z = \mathbf y \cdot \mathbf z$   | $ \mathbf y ^\top \frac{\partial \mathbf z}{\partial \mathbf x} + \mathbf z ^\top \frac{\partial \mathbf x}{\partial \mathbf x} $ |
| $\left\| \mathbf x \right\|^2 = \mathbf x\cdot \mathbf x = \mathbf x^\top \mathbf x$ |                      $2\mathbf x^\top$                       |
|                $ \left\|\mathbf x \right\| $                 |    $ \frac{\mathbf x^\top}{\left\| \mathbf x \right\| } $    |
|          $ \left\| \mathbf x - \mathbf a \right\| $          | $ \frac{(\mathbf x - \mathbf a)^\top}{\left\| \mathbf x - \mathbf a \right\| } $ |
|   $ \left\| \mathbf A \mathbf x - \mathbf a \right\| ^2 $    |     $2(\mathbf A \mathbf x - \mathbf b) ^\top \mathbf A$     |




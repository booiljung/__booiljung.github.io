# 머신러닝을 위한 수학

## Linear algebra

##### [Scalar]

길이, 넓이, 질량, 온도 - 크기만 주어지만 완전히 표시되는 양

##### [Vector]

속도, 위치이동, 힘 - 크기뿐만 아니라 방향까지 지정하지 않으면 완전히 표현할 수 없는 양

벡터는 크기와 방향을 갖는 유향선분 - 2차원, 3차원 공간의 벡터는 화살표로 표현 가능

시작점과 끝점이 같아서 크기가 $0$인 벡터를 영벡터라 한다(영벡터는 크기가 $0$이므로 방향은 임의의 방향으로 한다).
$$
\mathbf x 
= (x_1, x_2, \dots, x_n)
= \begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_n
\end{bmatrix} _{n \times 1}
$$

##### [벡터의 상등]

$\mathbb R^n$의 벡터
$$
\mathbf x 
= \begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_n
\end{bmatrix}
\mathbf y
= \begin{bmatrix}
y_1 \\
y_2 \\
\vdots \\
y_n
\end{bmatrix}
$$
에 대하여 $x_i = y_i (i = 1, 2, \dots, n)$이면 $\mathbf x = \mathbf y$라고 한다.

##### [벡터 합과 스칼라 배]

$\mathbb R^n$의 벡터
$$
\mathbf x 
= \begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_n
\end{bmatrix}
\mathbf y
= \begin{bmatrix}
y_1 \\
y_2 \\
\vdots \\
y_n
\end{bmatrix}
$$


와 스칼라 $k$에 대하여 두 벡터의 합 $\mathbf x + \mathbf y$와 $k$에 의한 $\mathbf x$의 스칼라배 $k \mathbf x$를 각각 다음과 같이 정의 한다.
$$
\mathbf x + \mathbf y
= \begin{bmatrix}
x_1 + y_1 \\
x_2 + y_2 \\
\vdots \\
x_n + y_n \\
\end{bmatrix}
$$

$$
k\mathbf x
= \begin{bmatrix}
kx_1 \\
kx_2 \\
\vdots \\
kx_n \\
\end{bmatrix}
$$



또한 $\mathbb R^n$에서 모든 성분이 $0$ 인 벡터를 영벡터 또는 원점이라 하고 $\mathbb 0$ 으로 나타낸다. 그러면 임의의 벡터 $\mathbf x \in \mathbb R^n$ 에 대하여
$$
\mathbf x + 0 \mathbf x, \\
\mathbf x + (-1) \mathbf x = 0
$$
이 성립함을 쉽게 알 수 있다. 여기서 $(-1)\mathbf x = -\mathbf x$로 정의하며 $-\mathbf x$를 $\mathbf x$의 음벡터라 한다.

모든  $n$차원 벡터 전체의 집합을 $n$-공간 ($n$차원 공간)$\mathbb R^n$으로 나타낸다. 즉
$$
\mathbb R^n
= \{(x_1, x_2, \dots, x_n) | x_i \in \mathbb R, i=1, 2, \dots, n \}
$$

##### $\mathbb x$의 노름 (norm, length, magnitude)

$\mathbb R^n$의 벡터 $\mathbf x = (x_1, x_2, \dots, \x_n)$에 대하여
$$
\| \mathbf x \|
= \sqrt{
x_1^2 + x_2^2 + \cdots + x_n^2
}
$$
을 $\mathbf x$의 노름(norm, length, magnitude)이라 한다.

위의 정의에서 $\| \mathbf x \|$는 원점 $P(x_1, x_2, \dots, x_n)$에 이르는 거리로 정의됨을 의미한다. 따라서 $\mathbb R^n$의 두 벡터 $\mathbf x = (x_1, x_2, \dots, x_n), \mathbf y=(y_1, y_2, \dots, y_n)$에 대하여 $

\| \mathbf x - \mathbf y \|$ 는 두 점 



##### 내적 Euclidean inner product, dot product



##### 코시-슈바르츠 부등식



##### $\mathbf x$위로의 $\mathbf y$의 정사영 projection of $\mathbf y$ onto $\mathbf x$



##### 정사영



##### 선형연립방정식 system of linear equations



##### consistent 연립방정식



##### inconsitent 연립방정식



##### 첨가행렬 augmented matrix



##### 가우스소거법



##### 가우스-요르단 소거법



##### 행렬의 합 matrix sum

$$
\mathbf A + \mathbf B
= \begin{bmatrix}
a_{ij} + b_{ij}
\end{bmatrix}_{m \times n}
$$

이때, 행렬의 크기는 같아야 한다.

##### 행렬의 스칼라 배 scalar multiple

$$
k \mathbf A
= \begin{bmatrix}
k a_{ij}
\end{bmatrix}_{m \times n}
$$

##### 행렬의 곱 product

$$
\mathbf A \mathbf B 
= \begin{bmatrix}
c_{ij}
\end{bmatrix} _{m \times n}
$$

여기서
$$
c_{ij} 
= a_{i1} b_{1j} + a_{i2} b_{2j} + \cdots a_{ip} b_{pj}
= \sum_{k=1}^p a_{ij} b_{kj}
(1 \le i \le m, 1 \le j \le n)
$$
행렬을 곱할때 행렬의 크기
$$
\mathbf A _{m \times p} \mathbf B _{p \times n}
= \mathbf C _{m \times n}
$$

##### 영행렬 zero matrix

성분이 모두 $0$인 행렬
$$
\mathbf O
$$
로 나타낸다.

##### 단위행렬 identity matrix

대각선 성분이 모두 $1$이고 나머지 성분은 모두 $0$인 정사각행렬로 $\mathbf I_n$을 나타낸다.
$$
\mathbf I_n
= \begin{bmatrix}
1 & 0 & 0 & \cdots & 0 & 0 \\
0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 1 & \cdots & 0 & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & 0 & \cdots & 1 & 0 \\
0 & 0 & 0 & \cdots & 0 & 1 \\
\end{bmatrix}_n
$$

##### 전치행렬 transpose

행렬 $\mathbf A$에 대한 전치 행렬 $\mathbf A ^ \top$은 다음과 같이 정의 한다.

전치 행렬은 원행렬의 행과 열을 바꾸어 얻어진다.

##### 전치 행렬의 성질

$$
\begin{align}
(\mathbf A^\top)^\top &= \mathbf A \\
(\mathbf A + \mathbf B)^\top &= \mathbf A^\top + \mathbf B^\top \\
(\mathbf A \mathbf B)^\top &= \mathbf B^\top \mathbf A^\top \\
(k \mathbf A)^\top &= k \mathbf A^\top \\
\end{align}
$$

##### 행렬의 대각합 trace

행렬 $\mathbf A = \begin{bmatrix} a_{ij}\end{bmatrix}_{n \times n}$의 대각합(trace)는 $\text{tr}(\mathbf A) = a_{11} + a_{22} + \dots + a_{nn} = \sum_{i=1}^n a_{ii}$ 이다.

##### 역행렬

$n$차의 정사각행렬 $\mathbf A$에 대하여 다음을 만족하는 행렬 $\mathbf B$가 존재하면 $\mathbf A$는 가역(invertible, nosingular)이라고 한다.
$$
\mathbf A \mathbf B
= \mathbf I_n
= \mathbf B \mathbf A
$$

##### 가역행렬 invertible matrix

##### 가역행렬의 성질 1

$n$차의 정사각행렬 $\mathbf A$, $\mathbf B$가 가역이고 $k$가 $0$이 아닌 스칼라일 때, 다음이 성립한다.

- $\mathbf A^{-1}$은 가역이고, $(\mathbf A^{-1})^{-1} = \mathbf A$이다.
- $\mathbf A \mathbf B$는 가역이고, $(\mathbf A \mathbf B) ^{-1} = \mathbf B ^{-1} \mathbf A ^{-1}$이다.
- $k \mathbf A$는 가역이고, $(k \mathbf A)^{-1} = \frac{1}{k} \mathbf A ^{-1}$이다.
- $\mathbf A^n$도 가역이고, $(\mathbf A^n)^{-1} = \mathbf A^{-n} = (\mathbf A^{-1})^n$

##### 가역행렬의 성질 2

##### 기본행렬

##### 기본행연산 elementary row operator, ERO

$\mathbf I_n$에 기본행연산 (elementary row operator, ERO)을 한 번 적용해서 얻은 행렬을 기본행렬(elementary matrices)이라 한다. 그리고 치환(permutation)행렬은 $\mathbf I_n$의 행들을 교환하여 얻어진 행렬이다.

##### 역행렬을 구하는 방법 1

-  주어진 행렬 $\mathbf A$에 단위행렬 $\mathbf I_n$을 첨가하여 $n \times 2n$ 행렬 을 $\begin{bmatrix}\mathbf A & \vdots & \mathbf I_n\end{bmatrix}$만든다.
- 단계 1에서 만든 행렬  $\begin{bmatrix}\mathbf A & \vdots & \mathbf I_n \end{bmatrix}$의 RREF를 구한다.
- 단계 2에서 얻어진 RREF를 $\begin{bmatrix}\mathbf C & \vdots & \mathbf D \end{bmatrix}$라고 하면 다음이 성립한다.
  - $\mathbf C = \mathbf I_n$이면 $\mathbf D = \mathbf A^{-1}$이다.
  - $\mathbf C \ne \mathbf I_n$이면 $\mathbf A$는 비가역이고 $\mathbf A^{-1}$은 존재하지 않는다. 

##### 대각선행렬 diagonal matrix, 스칼라행렬 scalar matrix

##### 부호화 함수 signature function

##### 치환 permutation, 순열

##### 가역행렬

##### 수반행렬 adjugate, adjunct 또는 classical adjoint matrix

##### 수반행렬을 이용한 가역행렬의 역행렬

$n$차의 정사각행렬 $\mathbf A$가 가역일 때, $\mathbf A$의 역행렬은 $\mathbf A^{-1} = \frac{1}{|\mathbf A|} \text{adj} \mathbf A$이다.

##### 일차독립 (linearly indenpendent)

##### 일차종속

##### 일차독립 판정법

##### 부분공간 subspace 판정법

##### 생성집합 spanning set

span

##### 기저 basis

##### 일차독립, 일차종속, 각 기저에 속하는 벡터의 개수

##### 차원 dimension

##### $\mathbf V$의 모든 벡터 $\mathbf v$는 기저 벡터들의 유일한 일차결합으로 표현된다.

##### $\text{nullity}(\mathbf A)$

$m \times n$ 행렬 $\mathbf A$에 대하여 $\mathbf A \mathbf x = \mathbf O$의 해공간, 즉 $\mathbf A$의 영공간의 차원을 $\text{nullity}(\mathbf A)$라고 나타낸다. 즉,
$$
\text{dim} \text{Null}(\mathbf A) = \text{nullity}(\mathbf A)
$$
이다.

##### 열공간 column space, $\text{Col}(\mathbf A)$, 행공간 row space, $\text{Row}(\mathbf A)$

##### 계수 rank, Rank-Nullity  정리

##### 최소제곱해 least squre solution

##### 정규직교기저 othonormal basis

##### Gram-Schmidt 정규직교화 기저의 존재성

##### QR분해

##### 행렬변환 matrix transformation, 선형변환 linear transformation

##### 표준행렬

##### 핵 kernel, 단사, 전사, 전단사, 동형사상 isomorphism

##### 선형변환이 단사일 필요충분조건

##### 고유값 eigenvalue, 고유벡터 eigenvector

##### 닮은 similar 행렬

##### 대각화가능한 diagonalizable 행렬

##### 대각화하가능할 필요충분조건

##### 직교행렬 real orthogonal matrix

정사각행렬 $\mathbf A$에 대하여 $\mathbf A^{-1} = \mathbf A ^\top$ 이면 $\mathbf A$를 직교행렬이라고 한다.

##### 직교행렬 정리

행렬 $\mathbf A$가 직교행렬이면 다음을 만족한다.

##### 직교대각화

##### 직교대각화가능할 필요충분조건

##### 교유값분해 eigen-decomposition

##### SVD 특이값분해 singular value decomposition

##### 특이값분해 정리

##### pseudo-inverse (Moore-Penrose Generalized Inverse)

##### full column rank를 갖는 $m \times n$ 행렬의 pseudo-inverse

##### 최소제곱해 (least square solution)

##### 이차형식 (quadratic form)

이차형식 (quadratic form)은 각 항이 2차식인 다항식으로 수학, 물리학, 경제학, 통계학, 이미지 처리기법 등 다양한 분야에 사용된다.

##### 이차곡선의 방정식의 행렬표현

두 변수 $x, y$를 갖는 이차곡선의 방정식
$$
ax^2 + 2bxy + cy^2 + dx + ey + f = 0
$$
을 행렬로 표현하면 다음과 같다.
$$
\begin{bmatrix}
x, y
\end{bmatrix}
\begin{bmatrix}
a & b \\
b & c \\
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
\end{bmatrix}
+
\begin{bmatrix}
d, e
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
\end{bmatrix}
+ f = 0
$$

##### $\mathbb R^2$상의 이차형식 (quadratic form)

##### $\mathbb R^2$의 주축정리 (principal axis theorem)

##### 양의 정부호 (positive definite), 음의 정부호, 부정부호(indefinite)

##### 이차형식의 정부호, 부정부호(indefinite)

##### 이차형식 정부호의 필요충분조건
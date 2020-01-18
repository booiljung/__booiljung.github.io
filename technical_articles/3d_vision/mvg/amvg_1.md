강의 스터디 - 황성수

https://www.youtube.com/watch?v=N6vP0T1Xabg&list=PLoJdZ7VvEiRNQwM3pcwHWwLQutIYMs4KK

# Multiple View Geometry

## Homogeneous Coordinate

### Projective Geometry and Transformations of 2D

#### Homogeneous Coordinate

##### Homogeneous coordinate란?

그래픽스 또는 3D 비전 분야에서 많이 사용되는 좌표계의 한 종류다.

Homogenous 좌표계를 사용하면 affine 변환과 perspective 변환이 하나의 행렬로 정의 된다.

##### 선의 homogeneous coordinate 표현

직선의 방정식 $ax + by + c = 0$ 을 $ax + by + cw = 0$ 로 표현 된다.

이떄 $ax + by + c = 0$ 상의 점으로 투영되는 점 $(x, y, w)$ 들은 $ax + by + cw = 0$을 만족해야 한다.

##### 점의 homogeneous coordinate 표현

임의의 상수 $w$에 대해 점 $P(x, y)$의 좌표를 $P(wx, wy, w)$로 표현하는 방식이다. $(w \ne 0)$

2D 이미지에 한 점으로 투영되는 모든 점(같은 ray상의 점)들을 표현 할 수 있다.

원래의 좌표를 구하고 싶다면 $w = 1$로 만들면 된다. $P(x, y, 1)$

#### Points and Lines

직선 $l$과 $l'$이 교차하는 점 $x$는 두 직선의 외적으로 구할 수 있다. $x=l\times l'$

점 $x$와 $x'$를 이은 선 $l$은 두 점의 외적으로 구할 수 있다. $l=x\times x'$

평행한 두 직선의 교차점은 무한대에 존재하고 이 교차점을 ideal point 라고 한다.

두개의 ideal points를 연결한 직선을 ideal line이라고 한다.

직선 $l$과 $l'$이 평행할 때 두 직선의 외적은 $0$이다. $l \times l' = 0$

Duality: $x = l \times l' \leftrightarrow l = x \times x'$

##### 투영평면

$x$: 3차원에서 원점을 지나는 ray

$l$: 3차원에서 원점을 지나는 평면

투영 평면에서 점과 직선은 3차원에서 각각 선과 평면을 나타낸다.

![image-20200116212642670](/home/booil/.config/Typora/typora-user-images/image-20200116212642670.png)

#### Projective Transformation

##### 평면의 projective transformation

선형변환을 수행하는 $3 \times 3$ 행렬로 수행한다.

평면 $\pi$ 를 $\pi'$로 변환하는 행렬 $\mathbf H$라고 할 때 $x'=\mathbf H x$이다.

평면 위의 점 4개의 관계를 안다면 그 어떤 선형변환도 표현 할 수 있다.

왜곡된 영상을 선형변환을 통해 보정할 수 있다.

![image-20200116212746938](/home/booil/.config/Typora/typora-user-images/image-20200116212746938.png)

#### Isometries/Euclidean Transformations (등거리 변환/유클리드 변환)

![image-20200116214210250](/home/booil/.config/Typora/typora-user-images/image-20200116214210250.png)

3 dof (1 rotation, 2 translations)
$$
\begin{pmatrix}
x'\\
y'\\
1
\end{pmatrix}
=
\begin{pmatrix}
\varepsilon\cos\theta & -\sin\theta & t_x \\
\varepsilon\sin\theta & \cos \theta & t_y \\
0 & 0 & 1 \\
\end{pmatrix}
\begin{pmatrix}
x\\
y\\
1
\end{pmatrix}
, \varepsilon
= \pm 1
$$

$$
x' = H_Ex = 
\begin{pmatrix}
r_{11} & r_{12} & t_x \\
r_{21} & r_{22} & t_y \\
0 & 0 & 1
\end{pmatrix}
x
=
\begin{pmatrix}
\mathbf R & t \\
0^\intercal & 1
\end{pmatrix}
x
$$

#### Affine Transformations

6 dof (2 scale, 2 rotation, 2 translation)

![image-20200116214317463](/home/booil/.config/Typora/typora-user-images/image-20200116214317463.png)
$$
\mathbf A
= \mathbf R(\theta) \mathbf R(-\theta) \mathbf D \mathbf R(\theta), \mathbf D
= \begin{pmatrix}
\lambda_1 & 0 \\
0 & \lambda_2
\end{pmatrix}
$$

$$
x'
= \mathbf H_Ax
= \begin{pmatrix}
a_{11} & a_{12} & t_x \\
a_{21} & a_{22} & t_y \\
0 & 0 & 1
\end{pmatrix}
x
= \begin{pmatrix}
\mathbf A & t\\
0^\intercal & 1
\end{pmatrix}
x
$$

#### Projective Transformations

![image-20200116215129831](/home/booil/.config/Typora/typora-user-images/image-20200116215129831.png)
$$
x'
= \mathbf H_P x 
= \begin{pmatrix}
h_{11} & h_{12} & h_{13} \\
h_{21} & h_{22} & h_{23} \\
h_{31} & h_{32} & h_{33} \\
\end{pmatrix}
x
= \begin{pmatrix}
\mathbf A & t \\
\mathbb v^\intercal & v \\
\end{pmatrix}
x
$$

#### Recovery of Affine and Metric Properties form Images

Line at infinity 와 Orthogonal lines 의 복원

![image-20200116215547662](/home/booil/.config/Typora/typora-user-images/image-20200116215547662.png)

## Image Features

An image feature is a pice of information which is relevant for solving the computational task related to a certain application.

Features may be specific structures in the image such as points, edges or objects.

Features may also be the result of a general neighborhood operation or feature detection applied to the image.

#### Why do we need to extract a feature?

![image-20200116221940664](/home/booil/.config/Typora/typora-user-images/image-20200116221940664.png)

#### What is a good feature?

A good feature should be invariant to...

- Illumination
- Translation
- Scale
- Rotation
- Perspective transform

![image-20200116222115100](/home/booil/.config/Typora/typora-user-images/image-20200116222115100.png)

#### What is a good feature?

- A good feature should be computationally inexpensive.
- A good feature should be memory efficient.

#### Several Images features

Widely used feature extractor & descriptor

|                  Name                   | detector | descriptor |
| :-------------------------------------: | :------: | :--------: |
|          Harris Corner (1988)           |    O     |     X      |
| Shi & Tomasi (1994) goodFeaturesToTrack |    O     |     X      |
|               SIFT (1999)               |    O     |     O      |
|               MSER (2004)               |    O     |     O      |
|               SURF (2006)               |    O     |     O      |
|               FAST (2006)               |    O     |     X      |
|           ORB (FAST + BRISK)            |    O     |     O      |
|              AGAST (2010)               |    O     |     O      |
|              BRIEF (2012)               |    O     |     O      |
|            AKAZE (2012) KAZE            |    O     |     O      |

#### ORB

oFast detector + r-BRIEF descriptor

- FAST

  - Determines the corner by having more than N consecutive pixels whose intensities are higher (or lower).
  - 9 consecutive pixels when radius is 3

  ![image-20200116222813043](/home/booil/.config/Typora/typora-user-images/image-20200116222813043.png)

- BRIEF

  - A bit string descri

## 참조

- [컴퓨터 비전 특강: MVG](https://www.youtube.com/watch?v=N6vP0T1Xabg&list=PLoJdZ7VvEiRNQwM3pcwHWwLQutIYMs4KK)









https://www.youtube.com/playlist?list=PLoJdZ7VvEiRNUxlIXlgy7Fh8ziyt4Hw50&fbclid=IwAR3PMxyhePHkBkdwErdjt5TgCMAUhknX-9Jfdls5vYw1wROXcsKEs3AH5es
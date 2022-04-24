# Epipolar Geometry

Epipolar geometry는 2-view 비전에서의 기하로, 동일한 사물 또는 장면에 대한 영상을 서로 다른 두 지점에서 획득하였을 때, 두 영상의 매칭쌍들 사이의 기하관계를 다루는 것입니다.



여기서, 두개의 카메라가 점 $\mathbf{c}_0, \mathbf{c}_1$에 있고, 각각의 카메라의 이미지센서에서 획득한 영상평면 $\mathbf{G}_0, \mathbf{G}_1$가 있고, 3차원상 물체의 위치를 $\mathbf{p}$를 각각의 영상평면상의 점 $\mathbf{p}_0, \mathbf{p}_1$으로 나타낸다.

두 카메라의 위치 $\mathbf{c}_0, \mathbf{c}_1$를 잇는 직선이 영상평면 $\mathbf{c}_0, \mathbf{c}_1$에서 만나는 점은 $\mathbf{p}_0, \mathbf{p}_1$이며 이를 epipole이라고 하며, $\mathbf{e}_0, \mathbf{e}_1$과 $\mathbf{p}_0, \mathbf{p}_1$를 잇는 선을 $\mathbf{l}_0, \mathbf{l}_1$으로 나타내며 이를 epiline 또는 epipolar line이라고 하며, 

두 카메라 사이의 기하학적 관계는 알고 있으며 이동 또는 회전이며 이를  $\mathbf{[R|T]}$로 나타낸다.







## Essential Matrix

$$
\mathbf{e}_0^\top \mathbf{E} \mathbf{e}_1 = \mathbf{0}
\tag{1}
$$

$$
\begin{bmatrix} u_0 & v_0 & 1 \end{bmatrix} \mathbf{E} \begin{bmatrix} u_1 \\ v_1 \\ 1 \end{bmatrix} = \mathbf{0}
\tag{2}
$$

임의 두 지점에서 찍은 영상의 매칭점들은 항상 (1)의 식을 통해 관계 지을 수 있으며, 이때 식(1)을 epipolar constraint 또는 essential constraint라고 하며, 이 $3\times3$ 행렬 $\mathbf{E}$를 Essential Matrix라고 부른다.



Essential Matrix 가 항상 존재하는 이유는 임의의 두 카메라 좌표축 사이의 관계는 회전, 평행이동에 의해 관계 지울 수 있기 때문에, 두 카메라 좌표축 사이의 $3\times3$ 회전행렬을 $\mathbf{R}$  평행이동 벡터를 $\mathbf{t}$라 하였을 때, 외부 공간상의 한 점을 두 카메라 좌표계에서 보았을때의 관계를
$$
\mathbf{p}_1 = \mathbf{R} \mathbf{p} + \mathbf{t}
\tag{3}
$$
와 같이 잡을 수 있다.

이때 essential matrix $E$를 다음과 같이 잡으면
$$
\mathbf{E} = [\mathbf{t}]_{x} \mathbf{R}
\tag{4}
$$
이며, 아래의 식 (5)가 만족됨을 알 수 있다. 단, 식(4)에서 $\mathbf{t}_{x}$는 벡터 $\mathbf{t}_{x}$와의 외적(outer product)를 의미한다. 또한, $\mathbf{t}_{x}\mathbf{R}$은 $\mathbf{t}$와 $\mathbf{R}$과의 외적을 의미하는게 아니라, 먼저 $\mathbf{R}$로 회전을 시킨 후 다음에 $\mathbf{t}$와 외적을 시키는 일련의 변환을 의미한다. 즉,
$$
\mathbf{E}\mathbf{p} = \mathbf{t}_{x} \mathbf{R} \mathbf{p} = \mathbf{t} \times (\mathbf{R} \mathbf{p})
$$
다.
$$
\mathbf{e}_1^\top \mathbf{E} \mathbf{e}_1 = \mathbf{0}
\tag{5}
$$

$$
\begin{bmatrix} x_1 & y_1 & z_1 \end{bmatrix} \mathbf{E} \begin{bmatrix} x_0 \\ y_0 \\ z_0 \end{bmatrix} = \mathbf{0}
\tag{2}
$$

식(5)의 좌변에 





```
$\mathbf{p}$
$\mathbf{G}_0, \mathbf{G}_1$
$\mathbf{c}_0, \mathbf{c}_1$
$\mathbf{p}_0, \mathbf{p}_1$
$\mathbf{e}_0, \mathbf{e}_1$
$\mathbf{l}_0, \mathbf{l}_1$
$\mathbf{[R|T]}$
```


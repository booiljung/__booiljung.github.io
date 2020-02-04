# Computer vision

## 좌표계 (coordinate system)

### 월드좌표계 (world coordinate system)

월드좌표계는 임의의 위치를 기준으로 삼는 좌표계이며 다음과 같이 대문자표 표기 합니다.
$$
P = (X, Y, Z)
$$

### 카메라 좌표계 (camera coordinate system)

카메라를 기준으로 하는 좌표계입니다. 카메라의 정면을 $Z$축, 아래 방향을 $Y$축, 우측 방향을 $X$축으로 합니다.
$$
P_c = (X_c, Y_c, Z_c)
$$

### 픽셀좌표계 (pixel image coordinate system)

영상좌표계 (image coordnate system)이라고도 합니다. 좌측 상단을 기준으로, 오른쪽으로 $+x$축, 아래쪽으로 $+y$축입니다. 이 $x$축과 $y$축으로 결정되는 평면을 이미지 평면 (Image plane)라고도 합니다.

3D 공간의 한 점 $P = (X, Y, Z)$는 카메라 렌즈의 촛점을 지나서 이미지 평면의 한점  $p_\text{img} = (x, y)$에 투영(projection) 됩니다. 3D 점 $P$부터 $p_{img}$의 좌표는 유일하게 결정 할 수 있지만, 투영되는 과정에서 월드좌표계의 $Z$ 좌표를 잃게 되며 따라서 $p_\text{img}$에서 $P$를 복구하는 것은 부가적인 정보가 없이는 불가능 합니다.

픽셀좌표계의 단위는 픽셀이며, 픽셀 좌표는 소문자로 표기 합니다.
$$
p_\text{img} = (x, y)
$$




## 참조

- [다크프로그래머](https://darkpgmr.tistory.com)
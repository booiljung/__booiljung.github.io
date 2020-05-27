# Kalman filters

## Average Filter

평균은 데이터의 총합을 개수로 나눈 값입니다. $k$개의 데이터($x_1, x_2, x_3, \dots, x_k$)가 있을때 , 평균은 아래와 같습니다.
$$
\bar x_k = \frac{x_1 + x_2 + \dots + x_k}{k}\tag{1.1}
$$
이렇게 데이터를 모두 수집하여 한꺼번에 계산하는 식을 배치식(batch expression)이라고 합니다.

여기에 데이터 하나 $x_{k+1}$ 가 추가되면 처음부터 다시 계산해야 합니다.
$$
\bar x_{k+1} = \frac{x_1 + x_2 + \dots + x_{k+1}}{k+1}
$$
만일 평균을 계산하는다 $k$초가 걸렸다면 다시 $k+1$초 동안 계산해야 하고, $k+1$개를 보관할 저장소를 필요로 하게 됩니다. 만일 $k$가 천만개라면 116일 동안 다시 계산해야 합니다. 더 효율적이 방법이 필요한데, 처음부터 계산하지 않고 이전 평균을 재활용하여 다시 데이터를 추가하여 평균을 얻는 방법이 있습니다. 이런 방법을 재귀식(recursive expression)이라고 합니다.

식 (1.1)의 양변에 $k$를 곱합니다.
$$
k \bar x_k = x_1 + x_2 + \dots + x_k
$$
다시 양변을  $k-1$로 나눕니다.
$$
\frac{k}{k-1} \bar x_k = \frac{x_1 + x_2 + \dots + x_k}{k-1}
$$
$x_k$만을 따로 분리 합니다.
$$
\frac{k}{k-1} \bar x_k = \frac{x_1 + x_2 + \dots + x_{k-1}}{k-1} + \frac{x_k}{k-1}
$$
이 식은 아래처럼 다시 쓸 수 있습니다.
$$
\frac{k}{k-1} \bar x_k = \bar x_{k-1} + \frac{x_k}{k-1}
$$
이제 양변을 $\frac{k}{k-1}$로 나눕니다.
$$
\begin{aligned}\bar x_k &= \frac{k}{k-1}(\bar x_{k-1} + \frac{x_k}{k-1}) \\\\&= \frac{k-1}{k}\bar x_{k-1} + \frac{1}{k} x_k\end{aligned}\tag{1.2}
$$
$a = \frac{k-1}{k}$로 놓겠습니다.
$$
a \equiv \frac{k-1}{k} = 1-\frac{1}{k}
$$
그래서
$$
\frac{1}{k} = 1-a
$$
이것을 (1.2)에 적용하면
$$
\bar x_k = a \bar x_{k-1} + (1-a)x_k
$$
이것을 평균필터 (average filter)라고 합니다.

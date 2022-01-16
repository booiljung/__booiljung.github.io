[Up](index.md)

# 칼만필터 (Kalman filters)

## 이동평균필터 (Moving average filter)

시간에 따라 변화하는 주식 가격과 같은 데이터에 평균을 취하면 변화가 뭉게지고 맙니다. 그래서 주식 가격은 최근의 몇개의 데이터에 대해 평균을 얻고 이것을 이동평균 (Moving average)라고 합니다. 이동평균은 가장 오래된 데이터를 버리고 최근의 데이터를 추가하여 데이터 갯수를 유지하며 평균을 얻습니다.

$k$개의 데이터 중 마지막 $n$개의 데이터에 대한 이동평균을 배치식은 아래와 같습니다.
$$
\bar x_{k} = \frac{x_{k-n+1} + x_{k-n+2} + \dots + x_{k}}{n}
\tag{1.1}
$$

이 식을 재귀식으로 유도합니다.
$$
\bar x_{k-1} = \frac{x_{k-n} + x_{k-n+1} + \dots + x_{k-1}}{n}
\tag{1.2}
$$
식 (1.1)에서 (1.2)를 뺍니다.
$$
\begin{aligned}
\bar x_{k} - \bar x_{k-1} 
&= \frac{x_{k-n+1} + x_{k-n+2} + \dots + x_{k}}{n} - \frac{x_{k-n} + x_{k-n+1} + \dots + x_{k-1}}{n}
\\\\
&= \frac{x_k - x_{k-n}}{n}
\end{aligned}
$$
이를 $\bar x_k$에 대해 정리 합니다.
$$
\bar x_{k} 
= \bar x_{k-1} + \frac{x_k - x_{k-n}}{n}
$$
 이동 평균은 재귀식을 사용하는 이점이 많지 않습니다.
# Kalman filters

## Average filter

평균필터
$$
\begin{align}
\bar x_k
&= \frac{k-1}{k}\bar x_{k-1} + \frac{1}{k}x_k \\
&= a \bar x_{x-1} + (1-a)x_k \\
\end{align}
$$
여기서,


$$
\begin{align}
\bar x_k &: \text{average of } k \text{ samples}\\
k &: \text{number of samples}\\
a &: \frac{k-1}{k} \\
\end{align}
$$
이다.

## Moving average filter

이동 평균 필터
$$
\bar x_k
= \bar x_{k-1} + \frac{x_k - x_{k-n}}{n}
$$
여기서,
$$
\begin{align}
\bar x_k &: \text{moving average of } k \text{ samples}\\
n &: \text{number of last samples} \\
\end{align}
$$
이다.

## Exponentially weighted moving average filter

지수 가중 이동평균 필터 또는 1차 저주파 통과 필터 (First-order low pass filter)
$$
\begin{align}
x_k = a \bar x_{k-1} + (1-a)x_k
\end{align}
$$
여기서,
$$
\begin{align}
0 < a < 1
\end{align}
$$
이다.

## Kalman filter

칼만필터

##### 0. 초기값 선정

$$
\begin{align}
\hat x_0, P_0
\end{align}
$$

##### 1. 추정값과 오차 공분산 예측

$$
\begin{align}
\hat x _\bar k &= A \hat x_{k-1} \\
P_\bar k &= AP_{k-1} A^\top + Q \\
\end{align}
$$

##### 2. 칼만 이득 계산

$$
\begin{align}
K_k = P_\bar k H ^\top (H P_\bar k H^\top + R)^{-1}
\end{align}
$$

##### 3. 추정값 계산

$$
\hat x_k = \hat x_{\bar k} + K_k (z_k - H\hat x_\bar k)
$$

##### 4. 오차 공분산 계산

$$
P_k = P_\bar k - K_k H P_\bar k
$$

여기서,
$$
\begin{align}
\hat x_k &: k \text{ th estimate} \\
z_k &: k \text{ th measurement} \\
K_k &: k \text{ th Kalman gain} \\
A &: \text{system model} \\
H &: \text{system model} \\
Q &: \text{system model} \\
R &: \text{system model} \\
\end{align}
$$

### 요약

1. 시스템 모델 ($A$, $Q$)를 기초로 다음 시각에 상태와 오차 공분산이 어떤 값이 될지를 예측합니다.

$$
\hat x _\bar k, P_\bar k 
$$

2. 측정값과 예측값의 차이를 보정해서 새로운 추정값을 계산합니다. 이 추정값이 칼만 필터의 최종 결과물입니다.

$$
\hat x_k, P_k
$$

3. 위의 두 과정을 반복합니다.

### 추정값 계산

$$
\begin{align}
\hat x_k &= \hat x_{\bar k} + K_k (z_k - H\hat x_\bar k) \\
\end{align}
$$


# 칼만필터 (Kalman filter)

칼만필터 알고리즘은 아래와 같습니다.

##### 1. 초기 추정값  및 초기 오차공분산 $P_0$ 설정:

$$
\begin{align}
&\hat x_0, P_0 \\
\\
&\hat x_0: 초기 추정값 \\
&P_0: 초기 오차공분산 \\
\end{align}
$$

##### 2. 추정값과 오차 공분산 예측:

$$
\begin{align}
\hat x _\bar k &= A \hat x_{k-1} \\
P _\bar k &= A P_{k-1} A^\top + Q \\
\\
\hat x^k&: 추정값 \\
P_{\bar k}&: 오차공분산예측값 \\
A&: 시스템 모델 행렬\\
Q&: 시스템 노이즈 \\
\end{align} \\
$$

3. 칼만이득 계산:

$$
\begin{align}
K_k &= P_{\bar k} H ^\top (HP_{\bar k} H^\top + R)^{-1} \\
\\
K_k&: 칼만 이득 \\
H&: 행렬 \\
R&: 측정값 노이즈 \\
\end{align}
$$

4. 추정값 계산:

$$
\begin{align}
\hat x_k &= \hat x_{\bar k} + K_k (z_k - H \hat x _{\bar k}) \\
\\
\hat x_k&: 추정값\\
z_k&: 측정값 \\ 
H&: 행렬 \\
\end{align}
$$

5. 오차 공분산 계산:

$$
\begin{align}
P_k &= P_{\bar k} - K_k H P_{\bar k} \\
\\
P_k &: 오차공분산 \\
\end{align}
$$

6. 2로 반복.

## 참조

- [칼만 필터 기초 - 알고리즘](https://lovely-embedded.tistory.com/15)










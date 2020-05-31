[Up](index.md)

# Kalman filter

## Simple Kalman

간단한 예제를 만들어 보겠습니다. 먼저 시스템 모델을 알아야 합니다. 이 시스템은 배터리의 전압을 측정합니다.
$$
\begin{aligned}
x_{k+1} &= x_k \\
z_k &= x_k + v_k \\
x_0 &= 0 \\
v_k &= N(0, 2^2)
\end{aligned}
$$
초기 전압은 알 수 없으며 초기 추정값을 $x_0$는 0 volt 로 시작 합니다.

전압 측정시 노이즈도 입력되며 노이즈는 $z_k$이며 정규 분포(normal distribution 또는 Gaussian distribution)으로 평균은 0, 표준편차는 2입니다. 실제 시스템에서는 노이즈의 분포가 일치하는지 센서의 신호를 샘플링하여 통계를 내야 합니다.

칼만 필터에 4개의 상수가 있었습니다. $A$, $H$, $Q$, $R$ 이 그것들입니다. 이 상수의 값도 지정합니다.
$$
\begin{aligned}
A &= 1\\
H &= 1\\
Q &= 0\\
R &= 3\\
\end{aligned}
$$
을 주었습니다. $Q$가 $0$인 이유는 $w_k$가 없기 때문입니다.

초기 예측값도 정해야 하는데, 초기 예측값을 알 수 없다면, 초기 예측 추정값은 평균이나 중간값, 초기 오차공분산 예측값을 크게 지정하는 것이 좋습니다. 이 시스템의 전압은 0V ~ 5V 사이이므로 초기 예측 추정값은 2.5, 초기 오차공분산 예측값은 10을 지정하였습니다.
$$
\begin{aligned}
\breve x_0 &= 2.5\\
\breve P_0 &= 10\\
\end{aligned}
$$
예측 추정값 계산식은 
$$
\begin{align}
\breve x_{k} &= A \hat x_{k-1} \\
\end{align}
$$
이며, 예측 오차공분산 계산식은
$$
\begin{align}
\breve P_k &= AP_{k-1} A^\top + Q \\
\end{align}
$$
입니다. 칼만 이득 계산식은
$$
\begin{align}
K_k = \breve P_k H ^\top (H \breve P_k H^\top + R)^{-1}
\end{align}
$$
입니다. 추정값과 오차공분산의 계산식은
$$
\begin{aligned}
\hat x_k &= \breve x_k + K_k (z_k - H \breve x_k)
\\
P_k &= \breve P_k - K_k H \breve P_k
\end{aligned}
$$
입니다.

이 칼만필터 시스템 모델을 C++로 구현 하겠습니다.

```c++
#include <iostream>
#include <torch/torch.h>
#include <random>
#include <thread>

class simple_kalman
{
	double A = 1.0,
		H = 1.0,
		Q = 0.0,
		R = 3.0;

	double x = 2.5,
		P = 10;

public:
	double gain(double z)
	{
		double xp = A * x;
		double Pp = A * P * A;
		double K = Pp * H * (1.0/(H * Pp * H + R));

		x = xp + K * (z - H * xp);
		P = Pp - K * H * Pp;
		return x;
	}
};
```

호출은 아래와 같습니다.

전압은 5V에서 시작하여 0V까지 감소하고 다시 5V로 증가하며 반복 합니다.

```c++
int main() {
	using namespace std;
	using namespace std::chrono;
	using namespace std::chrono_literals;

	std::default_random_engine rand;
	normal_distribution<double> gaussian(0.0, 2.0);
	simple_kalman sk;

	auto start = high_resolution_clock::now();
	std::this_thread::sleep_for(1s);

	double z = 5.0;
	double d = -0.001;
	while (true)
	{
		std::this_thread::sleep_for(0.1s);		
		double v = gaussian(rand);
		z += d;
		if (d < 0.0 && z <= 0.0)
			d = +0.001;
		else if (0.0 < d && 5.0 <= d)
			d = -0.001;
		double x = sk.gain(z + v);
		cout << z << ", " << z + v << ", " << x << endl;
	}
}
```






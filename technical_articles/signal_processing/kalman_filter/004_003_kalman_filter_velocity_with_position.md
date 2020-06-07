[Up](index.md)

# Kalman filter

## 위치로 속도 얻기

배가 이동할때 위치로 속도를 얻어 보겠습니다. 위치를 미분하면 속도를 쉽게 얻을 것 같지만, 노이즈로 인해 의외로 정확도가 낮습니다.

상태 변수는 아래와 같습니다.
$$
\begin{aligned}
x = \begin{bmatrix}
\text{pos}\\
\text{vel}\\
\end{bmatrix}
\end{aligned}
$$
먼저 시스템 모델을 설계 해야 합니다.
$$
\begin{aligned}
x_{k+1} &= A x_k + w_k \\
z_k &= H x_k + v_k \\
A &= \begin{bmatrix}
	1 & \Delta t\\
	0 & 1 \\
\end{bmatrix} \\
H &= \begin{bmatrix}
	1 & 0 \\
\end{bmatrix}
\end{aligned}
$$
행렬 $A$를 시스템 모델에 대입합니다.
$$
\begin{aligned}
x_{k+1} &= A x_k + w_k \\

&= \begin{bmatrix}
	1 & \Delta t\\
	0 & 1 \\
\end{bmatrix} x_k + w_k \\
\end{aligned}
$$
여기에 상태변수 정의를 대입해 보겠습니다.
$$
\begin{aligned}

\begin{bmatrix}
	\text{pos} \\
	\text{vel} \\
\end{bmatrix} _{k+1} 

&=

\begin{bmatrix}
	1 & \Delta t\\
	0 & 1 \\
\end{bmatrix} 

\begin{bmatrix}
	\text{pos} \\
	\text{vel} \\
\end{bmatrix}_k + 

\begin{bmatrix}
	0 \\
	w_k \\
\end{bmatrix} \\

&= 

\begin{bmatrix}
	\text{pos} + \text{vel} \cdot \Delta t \\
	\text{vel} + w
\end{bmatrix} _k

\end{aligned}
$$
위치와 속도에 대한 식을 보면
$$
\begin{aligned}

\text{pos} _{k+1} 

&=

\text{pos} _{k} 
+
\text{vel}_k \cdot \Delta t

\end{aligned}
$$
입니다. 배의 속도는
$$
\begin{aligned}

\text{velocity} _{k+1} 

&=

\text{velocity} _{k} 
+
w_k

\end{aligned}
$$
시스템 노이즈 $w_k$에 대해서만 영향을 받습니다.

시스템 모델의 측정값 $z_k$를 보겠습니다.
$$
\begin{aligned}

z_k &= H x_k + v_k \\

&=

\begin{bmatrix}
	1 & 0
\end{bmatrix}

\begin{bmatrix}
	\text{pos} \\
	\text{vel} \\
\end{bmatrix} + v_k
\\
&=

\text{loc} _k + v_k
\end{aligned}
$$
노이즈의 공분산 행렬 $Q$, $R$을 정해야 하는데 노이즈 특성을 따라야 하며, 센서 제작사에서 제공하는 것을 기본으로 사용하고, 제공되지 않는다면 실험과 경험을 통해 결정해야 합니다.
$$
\begin{aligned}
Q
&=
\begin{bmatrix}
	1 & 0 \\
	0 & 3 \\
\end{bmatrix}
\\
R
&=
10
\end{aligned}
$$


```c++
#include <iostream>
#include <torch/torch.h>
#include <random>
#include <thread>
#include <tuple>

class velocity_kalman
{
	double dt = 0.1;

	torch::Tensor A = torch::tensor
	({
		{ 1.0, dt  },
		{ 0.0, 1.0 },
	});
	torch::Tensor H = torch::tensor({{ 1, 0 }});
	torch::Tensor Q = torch::tensor
	({
		{ 1, 0 },
		{ 0, 3 },
	});
	double R = 10.0;

	torch::Tensor x = torch::tensor({{ 0, 20 }});
	torch::Tensor P = 5.0 * torch::eye(2);

public:
	std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> gain(double z)
	{
		torch::Tensor xp = A * x;
		torch::Tensor Pp = A * P * torch::transpose(A, 0, 1) + Q;
		torch::Tensor K = Pp * torch::transpose(H, 0, 1) * torch::inverse((H * Pp * torch::transpose(H, 0, 1) + R));

		x = xp + K * (z - H * xp);
		P = Pp - K * H * Pp;
		return std::make_tuple(x, P, K);
	}

private:
	std::default_random_engine rand;
	std::normal_distribution<double> gaussian = std::normal_distribution<double>(0.0, 2.0);
	double pp = 0.0, vp = 80.0;

public:
	double rand_position()
	{
		double w = 0.0 + 10.0 * this->gaussian(this->rand);
		double v = 0.0 + 10.0 * this->gaussian(this->rand);
		double z = pp + vp * dt + v;
		pp = z - v; // true position
		vp = 80.0 + w; // true velocity
		return z;
	}

};
```

호출은 아래와 같습니다.

```c++
int main()
{
	using namespace std;
	using namespace std::chrono;
	using namespace std::chrono_literals;

	std::default_random_engine rand;
	normal_distribution<double> gaussian(0.0, 2.0);
	velocity_kalman vk;

	high_resolution_clock::now();

	while (true)
	{
		std::this_thread::sleep_for(0.1s);
		double z = vk.rand_position();
		torch::Tensor x, P, K;
		tie(x, P, K) = vk.gain(z);	
		cout << "=============" << endl
			<< "z:" << z << endl
			<< "x:" << x << endl
			<< "P:" << P << endl
			<< "K:" << K << endl;
	}
}
```






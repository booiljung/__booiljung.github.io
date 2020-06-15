# Laplacian Transform

라플라시안 변환은 미분방정식을 푸는 강력한 방법중 하나입니다. 미분방정식을 공간으로 변환하여 단순하게 만든 후 이를 풀어내는 방법이다. 미분방정식의 고유값 (eigenvalue)만 추출하여 계산하는 기법이라고 할 수 있습니다.
$$
\begin{aligned}

F(x)
&= \mathcal L {f}(s)
\\
&= \int_0^\infty e ^{-st} f(t) dt

\end{aligned}
\tag{1}
$$
## 개요

원래 라플라시안 변환은 자연계의 운동들, 예를 들어 포락선(envelope)같은 감쇄현상을 설명하기 위해 고안된 개념입니다. 인간이 이해하기 쉬운 단위를 기계가 이해하기 쉬운 단위로 변환하는 개념이라고 생각하면 됩니다.

식 (1)에서 복소수 
$$
s = \sigma + \mathcal i \omega
$$
 라는 것을 생각해 보겠습니다.

여기서,

- $\sigma$와 $\omega$는 실수이며
- $\mathcal i$는 허수단위입니다.
- $s$는 네이피어수 $e$를 밑으로하는 지수인데, 지수법칙을 이용하여 실수부와 허수부로 분리할 수 있습니다. 실수부는 감쇄를, 허수부는 오일러공식에 의해 정현파(사인함수와 코사인 함수)형태로 표현됩니다. 이 둘을 곱하면 진동운동이 표현됩니다.

변환 과정에 들어있는 자세한 수학적 증명 등을 제외하고 전체적인 개념을 설명하면

1. $t-$ 공간에서의 복잡한 미분 방정식
2. 1의 방정식을 적절하게 라플라스 변환
3. $s-$ 공간에서의 윗 식보다는 간단한 대수 방정식 혹은 미분방정식.
4. 3의 해를 다시 적절하게 라플라스 역변환
5. $t-$ 공간에서 미분 방정시의 해

$t-$ 공간에서 결과물을 얻기 위해,  $s-$ 공간에서 무언가를 수행하는 방법입니다.

라플라스 변환으로는 선형 미분방정식에 국한되며, 비선형 방정식은 특별한 경우가 아닌 이상 수치해석을 믿을 수 밖에 없다.

## 사용

미분방정식은 그 차수가 높아질수록 그 문제의 해를 구하는 것은 거의 불가능하기 때문에 라플라스 변환을 사용합니다. 주어진 미분방정식을 곧바로 푸는 것이 아니라 먼저 라플라스 변환한 후 방정식 해를 구하고 다시  역변환하는 것입니다. 정확히 말하면 '일정 주기를 갖고 반복되는 함수형'일 경우 해를 유리식의 사칙연산을 사용하여 구할 수 있다는  것입니다. 반복되는 신호 처리 등에 대단히 유용해서 공정제어나 전기신호 등의 표현에 필수적으로 사용됩니다. 이 목적을 위하여 원래  함수 - 변환된 함수를 세트로 모아놓은 표도 있습니다. 이름하여 라플라스 변환표. 표 안에 세트가 수십 개 정도 있습니다.

보통 라플라스 변환을 배우면 다음에 나오는 내용은 푸리에 변환이며 라플라스 변환과 매우 닮은 꼴입니다. 편미분방정식(PDE)쪽으로 넘어가면 어지간해서는 이것을 사용하게 됩니다.

주로 공과대학에서 공업수학을 통해 처음 배우며, 이후 회로이론, 제어공학, 신호 및 시스템 등의 과목에서 활용합니다.

라플라스 변환의 이산 버전으로 Z-변환이라는게 있는데, 이는 계차 방정식(Difference equation)을 대수 방정식으로 바꿔줍니다. 대부분의 성질이 라플라스변환과 유사하며, 주로 디지털 시스템을 다루는데 사용됩니다.

## 구하는 방법

수학전공자라면 변환과 역변환의 과정을 직접 계산해서 보여야 할 일이 많을 것입니다. 여기서는 라플라스 변환 및 역변환에 관한 기술을 설명합니다.

### 라플라스 변환 표

$$
\begin{aligned}
F(s) &= \mathcal L{f(t)}(s)
\\
&= (\mathcal L f)(s)
\end{aligned}
$$

처럼 다양한 표기가 통용됩니다. ()

|             함수              |               $f(t)$                |                   $F(s)$                   |       ROC(수렴영역)        |
| :---------------------------: | :---------------------------------: | :----------------------------------------: | :------------------------: |
|       단위 임펄스 함수        |            $\delta (t)$             |                     1                      |          모든 $s$          |
|        단위 계단 함수         |               $u(t)$                |                  $s^{-1}$                  |     $\mathbf R(s) > 0$     |
|        단위 램프 함수         |               $tu(t)$               |                  $s^{-2}$                  |     $\mathbf R(s) > 0$     |
|  위 함수를 포함한 n승꼴 함수  |              $t^nu(t)$              |             $\frac{n!}{s^n+1}$             | $\mathbf R(s) > 0, n > -1$ |
|           지수 함수           |            $e^{at}u(t)$             |              $\frac{1}{s+a}$               |    $\mathbf R(s) > -a$     |
|           사인 함수           |     $f(t) = \sin(\omega t)u(t)$     |    $F(s) = \frac{\omega}{s^2+\omega^2}$    |     $\mathbf R(s) > 0$     |
|          코사인 함수          |     $f(t) = \cos(\omega t)u(t)$     |      $F(s) = \frac{s}{s^2+\omega^2}$       |     $\mathbf R(s) > 0$     |
| 지수적으로 감쇄하는 사인 함수 | $f(t) = e^{-at} \sin(\omega t)u(t)$ | $F(s) = \frac{\omega+a}{(s+a)^2+\omega^2}$ |    $\mathbf R(s) > -a$     |
| 지수적으로 감쇄하는 사인 함수 | $f(t) = e^{-at} \cos(\omega t)u(t)$ |   $F(s) = \frac{s+a}{(s+a)^2+\omega^2}$    |    $\mathbf R(s) > -a$     |
|        쌍곡 사인 함수         |    $f(t) = \sinh(\omega t)u(t)$     |    $F(s) = \frac{\omega}{s^2-\omega^2}$    | $\mathbf R(s) > |\omega|$  |
|       쌍곡 코사인 함수        |    $f(t) = \cosh(\omega t)u(t)$     |      $F(s) = \frac{s}{s^2-\omega^2}$       | $\mathbf R(s) > |\omega|$  |

### 함수와 다항식의 곱

$$
\mathcal L \{-t f(t)\} = F' (s)
$$

예를 들어
$$
\begin{aligned}
\mathcal L \{t e^{at}\}
&= -\frac{d}{ds} \left( \frac{1}{s-a} \right)
\\
&= \frac{1}{(s-a)^2}
\end{aligned}
$$
증명
$$
\begin{aligned}
F(s) &= \int_0^\infty e^{-st} f(t) dt \\
F'(s) &= \int_0^\infty e^{-st} f(t) dt \\
&= \mathcal L\{ -tf(t) \}

\end{aligned}
$$
일반화하면
$$
\begin{aligned}
\mathcal L \{t^n f(t)\}
&= (-1)^n \frac{d^n}{ds^n}F(s)
\end{aligned}
$$
아래와 같이 변환 할 수도 있다.
$$
\begin{aligned}
\mathcal L^{-1} \{ F(s) \}
&= f(t) \\
&= -\frac{1}{t} \mathcal L^{-1} {F'(s)}

\end{aligned}
$$
예를 들면
$$
\begin{aligned}
\mathcal L^{-1} \left\{ \ln\left(1-\frac{a^2}{s^2}\right) \right\}
&= \mathcal L^{-1} \{ \ln (s^2 - a^2) - 2 \ln s \} \\
&= -\frac{2}{t} - \frac{2 \cosh (at)}{t}

\end{aligned}
$$

### 주파수 평행이동

$$
\begin{aligned}
\mathcal{L}\left\{e^{at}f(t)\right\} = F(s-a)
\end{aligned}
$$

예를 들어
$$
\begin{aligned}
\mathcal{L}\left\{e^{at}\sin\left(wt\right)\right\} = \frac{w}{\left(s-a\right)^2 +w^2}
\end{aligned}
$$
증명
$$
\begin{aligned}
\mathcal{L}\left\{e^{at}f(t)\right\} &= \int_{0}^{\infty}e^{-st}e^{at}f(t)dt\\
&= \int_{0}^{\infty}e^{-(s-a)} f(t)dt \\
&= F(s-a)
\end{aligned}
$$

## 몫 형태

함수  $f(t)$의  라플라스 변환과 $\lim_{t \to 0}\frac{f\left(t\right)}{t}$ 가 존재하면
$$
\begin{aligned}
\mathcal{L}\left\{\frac{f(t)}{t} \right\} = \int_{s}^{\infty} F(u)du
\end{aligned}
$$
예를 들어
$$
\begin{aligned}
\mathcal{L}\left\{\frac{\cos(at)-1}{t} \right\} &= \int_{s}^{\infty}\frac{u}{u^2+a^2}-\frac{1}{u}du\\
&= -\ln\sqrt{1+\frac{a^2}{s^2}}
\end{aligned}
$$
이며 $a\neq0$ 일 경우 극한값이 존재한다는것도 따로 보여야 합니다.

증명
$$
\begin{aligned}
\int_{s}^{\infty} F(u)du &= \int_{s}^{\infty}\int_{0}^{\infty}e^{-ut} f(t) dtdu \\
&= \int_{0}^{\infty} \int_{s}^{\infty}e^{-ut} f(t) dudt \\ 
&= \int_{0}^{\infty} f(t) \int_{s}^{\infty} e^{-ut} dudt \\
&= \int_{0}^{\infty}\frac{1}{t}e^{-st} f(t) dt \\
&= \mathcal{L}\left\{ \frac{f(t)}{t} \right\}
\end{aligned}
$$

* 푸비니의 정리를 사용한다.

## 합성곱 (Convolution)

함수 $f, g$가 주어졌을 때, Convolution $\left(f*g\right)\left(t\right)$를 $\int_{0}^{t}f\left(t-u\right)g\left(u\right)du$로 정의합니다.

이 convolution은 몇 가지 성질이 있는데 다음과 같습니다.
 1. $$f*0 = 0 = 0*f$$ (영원)
 1. $$f*g = g*f$$ (교환법칙)
 1. $$f*(g+h) = f*g + f*h$$ (분배법칙)
 1. $$f*(g*h) = (f*g)*h$$ (결합법칙)

특히 중요한 것은 아래 정리로, 라플라스 역변환을 할 때 자주 쓰입니다.

정리
$$
\mathcal{L}\left\{f*g\right\} = \mathcal{L}\left\{f\right\}\times \mathcal{L}\left\{g\right\}
$$

예시
$$
\begin{aligned}
\frac{1}{\left(s^2+1\right)^2} &= \mathcal{L}\left\{\sin t \right\}\times \mathcal{L}\left\{ \sin t \right\} \\
&= \mathcal{L}\left\{(\sin *\sin) t \right\}
\end{aligned}
$$

$$
\begin{aligned}
\mathcal{L}^{-1} \left\{\frac{1}{ \left(s^2+1\right)^2} \right\} &= \int_{0}^{t}\sin\left(t-u\right)\sin u du \\
&= \frac{\sin t-t\cos t}{2}
\end{aligned}
$$

증명

$$
\begin{aligned}
\text{좌변} &= \int_{0}^{\infty}e^{-st}\int_{0}^{t} f(t-u) g(u) dudt \\
&= \int_{0}^{\infty}\int_{u}^{\infty}e^{-su}g(u) e^{-s(t-u)} f(t-u) dtdu \\
&= \int_{0}^{\infty}e^{-su}g(u) \int_{u}^{\infty}e^{-s(t-u)} f(t-u) dtdu
\end{aligned}
$$

$\xi = t-u$라 치환하면,

$$
\begin{aligned}
\int_{0}^{\infty}e^{-su} g(u) \int_{0}^{\infty}e^{-s\xi} f(\xi) d\xi du = \text{우변}
\end{aligned}
$$

## 참조

- 나무위키: [라플라스변환](https://namu.wiki/w/%EB%9D%BC%ED%94%8C%EB%9D%BC%EC%8A%A4%20%EB%B3%80%ED%99%98)

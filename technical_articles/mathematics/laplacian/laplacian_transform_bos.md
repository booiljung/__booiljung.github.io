# Laplacian Transform

## 1편 기본

$$
\begin{aligned}

\mathcal L [f(t)]
&= \int_0^\infty f(t) e^{-st} dt \\

\end{aligned}
$$

$f(t) = C $ 라고 합니다.
$$
\begin{aligned}
F(s) 
&= \mathcal L (f(t)) \\
&= \int_0^\infty C e^{-st} dt \\
&= C \int_0^\infty e^{-st} dt \\
&= C \lim_{a \rarr \infty} \left[ - \frac{1}{s} e^{-st} \right] _0 ^a \\
&= C \lim_{a \rarr \infty} \left[ - \frac{1}{s} e^{-sa} - \left( -\frac{1}{s}e^0 \right) \right] _0 ^a \\
&= \frac{C}{s} \lim_{a \rarr \infty} e^{-sa} + \frac{C}{s} \\
&= \frac{C}{s} \\
\end{aligned}
$$
필수 기본 공식
$$
\begin{aligned}

\mathcal L [t^n]
&= \frac{n!}{s^n+1} \\

\end{aligned}
$$

$$
\begin{aligned}

\mathcal L [e^{at}]
&= \frac{1}{s-a} \\

\end{aligned}
$$

$$
\begin{aligned}

\mathcal L [\cos \omega t]
&= \frac{s}{s^2 + \omega^2} \\

\end{aligned}
$$

$$
\begin{aligned}

\mathcal L [\sin \omega t]
&= \frac{\omega}{s^2 + \omega^2} \\

\end{aligned}
$$

## 2편 선형성

$$
\begin{aligned}

\mathcal L [ \alpha f(t) + \beta g(t) ]
&= \int _0 ^\infty \left( \alpha f(t) + \beta g(t) \right) e^{-st} dt
\\
&= \int _0 ^\infty \alpha f(t) e^{-st} dt
+ \int _0 ^\infty \beta g(t) e^{-st} dt
\\
&= \alpha \int _0 ^\infty f(t) e^{-st} dt
+ \beta \int _0 ^\infty g(t) e^{-st} dt
\\
&= \alpha \mathcal L [ f(t) ] + \beta \mathcal L [ g(t) ]

\end{aligned}
$$

예를 들어
$$
\begin{aligned}

y'' + y' + y &= \sin 3t \\
\\
\mathcal L [y''] + \mathcal L [y'] + \mathcal L [y] &= \mathcal L [\sin 3t] \\

\end{aligned}
$$

## 3편 $f(t)$의 $n$계 도함수의 라플라스변환 공식 증명, 설명

$$
\begin{aligned}

y'' + y' + y &= \sin 3t \\
\\
\mathcal L [y''] + \mathcal L [y'] + \mathcal L [y] &= \mathcal L [\sin 3t] \\

\end{aligned}
$$

에 이어서,
$$
\begin{aligned}

\mathcal L [f'] = \int _0 ^\infty f'(t) e^{-st} dt

\end{aligned}
$$
여기서,
$$
\begin{aligned}

u' &= f'(t) \\
v &= e^{-st} \\
\\
u &= f(t) \\
v' &= se^{-st}

\end{aligned}
$$
이라고 하면,
$$
\begin{aligned}

\int _a ^b u'v
= \left[uv\right]_a^b \int _a ^b uv'

\end{aligned}
$$

$$
\begin{aligned}

\left[ f(t)e^{-st} \right]_0^\infty &- \int_0^\infty f(t) \left( -se^{-st}  \right) dt
\\
0 &- s \int_0^\infty f(t) \left( -e^{-st}  \right) dt \\

\end{aligned}
$$




$$
\begin{aligned}


\end{aligned}
$$

$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$





$$
\begin{aligned}


\end{aligned}
$$



$$
\begin{aligned}


\end{aligned}
$$


## 참조

- BOS의 스터디룸: [라플라스 변환 쉽게 배우기 [1편]](https://www.youtube.com/watch?v=x1ldtSIVMqw)
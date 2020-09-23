# 리뷰

[Generalizing from a Few Examples: A Survey on Few-ShotLearning](https://arxiv.org/abs/1904.05046)

키워드:

- Few-Shot Learning
- One-Shot Learning
- Low-Shot Learning
- Small SampleLearning
- Meta-Learning
- Prior Knowledge

## 1 소개

- 튜링 머신
- 하드웨어 발전 (GPU, 분산 컴퓨팅)
- 모델의 발전
- 인간 챔피온을 이긴 모델들
- 현 딥러닝 모델의 단점
- Few-Shot Learning (FSL) 소개
  - one-shot imitation [147]
  - multi-armed bandits [33]
  - visual navigation [37]
  - continuous control [156]
- 학계에서 주목
  -  meta-learning [37, 106, 114]
  - embedding learning [14, 126, 138]
  - generative modeling [34, 35, 113]

## 1.2 표기

- Task: $T$

- 데이터셋: $D = \{ D_\text{train}, D_\text{test}\}$

- 훈련셋: $D_\text{train} = \{ (x_i, y_i )\}_{i=1}^I$

- 시험셋: $D_\text{test} = \{ x^\text{test}\}_{i=1}^I$

- Ground-truth 결합확률분포: $p(x, y)$

- $x$에서 $y$를 구하는 가설: $\hat h$

- $h(\cdot;\theta)$에 대한 가설 공간: $\mathcal H$
  - $h$의 모든 매개변수: $\theta$

- FSL 알고리즘은 $\theta$를 발견하여 $\mathcal H$를 찾기 위한 최적화 전략.

- 최선의 매개변수화: $h^\ast \in \mathcal H$

- 성능 측정을 위한 손실함수: $\mathcal l(\hat y, y)$

  - $\hat y = h(x;\theta)$


## 2.1 문제정의

> Definition 2.1 (Machine Learning [92, 94] ). A computer program is said to learn from experience $E$ with respect to some classes of task $T$ and performance measure $P$ if its performance can improvewith $E$ on $T$ measured by $P$.

$E$: 경험 (experience) - 데이터

$T$: 작업 (task)

$P$: 성능 측정 (performance measure)

성능  $P$는 $T$에 기반한 $E$에 의해 향상되고 $P$에 의해 측정

> Definition 2.2.
>
> Few-Shot Learning(FSL) is a type of machine learning problems (specified by $E$, $T$ and $P$), where $E$ contains only a limited number of examples with supervised information for the target $T$.

### FSL 예

- 영상 분류 [138]
- 단문 의미 분류 [157]
- 객체인식 [35]

### Few-shot classification

$h$: 분류기

각 입력 $x_i$에 대해 라벨 $y_i$를 예측

### N-way-K-shot classification [37, 138]

$D_\text{train}$ 은 $I = K N$을 포함.

$N$: 클래스

$K$: examples

### Few-shot regression [37, 156]






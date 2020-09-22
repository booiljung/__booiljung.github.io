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
  - one-shot imitation
  - multi-armed bandits
  - visual navigation
  - continuous control
- 학계에서 주목
  -  meta-learning
  - embedding learning
  - generative modeling

## 1.2 Notation

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

  




# 리뷰: A Survey on Few-ShotLearning

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

#### N-way-K-shot classification [37, 138]

$D_\text{train}$ 은 $I = K N$을 포함.

$N$: 클래스

$K$: examples

$I$: 샘플 수

### Few-shot regression [37, 156]

$h$: 회귀 추정 함수

$y_i$ observed value of the dependent variable $y$

$x_i$: observed value of independent variable $x$

### Few-short reinforcement learning [3, 33]

 finding a policy given only a few trajectories consisting of state-action pairs

### 3 typical scenarios of FSL

- Acting as a test bed for learning like human
- Learning for rare cases
- Reducing data gathering effort and computational cost

experience $E$: prior knowledge

One typical type of FSL methods: Bayesianlearning [35,76]

​	- It combines the provided training set $D_\text{train}$with some prior probability distribution which is available before $D_\text{train}$ is given.

One-shot learning: one example with supervised information in $E$, FSL called One-shot learning [14, 35, 138]

Zero-shot learningproblem (ZSL): $E$ does not contain any example with supervised information for the target $T$, FSL becomes azero-shot learningproblem.

## 2.2  Relevant Learning Problems

Weakly supervised learning [163] : learns from experience $E$ containing only weak supervision (such as incomplete, inexact, inaccurate or noisy supervised information)

- Semi-supervised learning [165]: learns from a small number of labeled samples and (usually a large number of) unlabeled samples in $E$. Example applications are text and webpage classification
  - Positive-unlabeled learning [81]: a special case of semi-supervisedlearning, in which only positive and unlabeled samples are given. 
    - example, to recommend friends in social networks, we only know the users’ current friends according to the friend list, while their relationships to other people are unknown

- Active learning [117]: Elects informative unlabeled data to query an oracle for output $y$. This is usually used for applications where annotation labels are costly, such aspedestrian detection.

Weakly supervised learning with incomplete supervision: only a smallamount of samples have supervised information

- includes only classification and regression
  - while FSL also includes reinforcement learning problems
- mainly uses unlabeled data as additional information in $E$

Imbalanced learning [54]: learns from experienceEwith a skewed distribution for $y$

- some values of $y$ are rarely taken
- as in fraud detection and catastropheanticipation applications
- trains and tests to choose among all possible $y$
- FSL trains and tests forywith a few examples, while possibly taking the other $y$’s as prior knowledge for learning

Transfer learning [101]: 

- transfers knowledge from the source domain/task, where training data is abundant, to the target domain/task
- used inapplications such as cross-domain recommendation
- Domain adaptation [11]: the source/target tasks are the same but the source/target domains are different
  -  For example, in sentiment analysis, the source domain data contains customer comments on movies, while the target domain data contains customer comments on daily goods
- Transfer learning methodsare popularly used in FSL [7,82,85]
  - prior knowledge is transferred from thesource task to the few-shot task

Meta-learning [59]: improves $P$ of the new task $T$ by the provided data set and the meta-knowledge extracted across tasks by a **meta-learner**

-  meta-learner gradually learns generic information (meta-knowledge) across tasks, and the learner generalizes the meta-learner for a new task $T$ using task-specific information
-  learning optimizers [5,80]
-  dealing with the cold-start problemin collaborative filtering [137]
-   guiding policies by natural language [25]
-  meta-learner is taken as prior knowledge to guide each specific FSL task
- formal definition, using --> Appendix A 참조 TODO

## Core Issue

the core issue of FSL based on error decomposition in supervised machine learning [17,18]

FSL supervised learning includingclassification and regression

provide insights for understanding FSL reinforcement learning

### Empirical Risk Minimization

$$
R(h) = \int \mathcal l (h(x), \mathbb d p(x, y)
\\
= \mathbb E \left [ \mathcal l (h(x), y) \right ]
$$



where:

$\mathcal l$: 성능측정을 위한 손실 함수

$h$: hypothesis

$R$: we want to minimize its expected **risk**

$p(x, y)$: 찾고자 하는 함수

empirical risk

$$
R_I (h) = \frac{1}{I} \sum _{i=1}^I \mathcal l(h(x_i), y_i)
$$
sed as a proxy forR(h), leading toempirical risk minimization[94,136] (with possiblysome regularizers)

$I$: 샘플 수

sed as a proxy forR(h), leading to **empirical risk minimization** [94,136] (with possibly some regularizers)




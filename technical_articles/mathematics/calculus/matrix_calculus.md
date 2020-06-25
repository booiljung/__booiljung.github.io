[Up](index.md)

# 행렬의 미적분

수학에서 행렬 미적분학은 다변량 미적분학, 특히 행렬 공간에서 수행되는 특수 표기법입니다. 그것은 많은 변수에 대한 단일 함수 및/또는 단일 변수에 대한 다변량 함수의 다양한 편미분을 단일 엔티티로 취급 될 수있는 벡터 및 행렬로 수집합니다. 이를 통해 다변량 함수의 최대 값 또는 최소값을 찾고 미분 방정식을 푸는 것과 같은 연산을 크게 단순화합니다. 여기서 사용되는 표기법은 통계 및 엔지니어링에서 일반적으로 사용되는 반면 텐서 인덱스 표기법은 물리학에서 선호됩니다.

두 개의 경쟁 표기법이 행렬 미적분학 분야를 두 개의 개별 그룹으로 나눕니다. 두 그룹은 벡터에 대한 스칼라의 도함수를 열 벡터 또는 행 벡터로 쓰는지 여부에 의해 구별 될 수 있습니다. 벡터가 행렬 (행 벡터가 아닌)과 결합 될 때 열 벡터로 취급되어야 한다는 일반적인 가정이 있더라도 이러한 두 규칙은 모두 가능합니다. 단일 계산법은 일반적으로 행렬 미적분학 (예 : 계량 경제학, 통계, 추정 이론 및 기계 학습)을 사용하는 단일 필드 전체에서 다소 표준 일 수 있습니다. 그러나 주어진 필드 내에서도 경쟁 규칙을 사용하여 다른 저자를 찾을 수 있습니다. 두 그룹의 저자는 종종 특정 협약이 표준 인 것처럼 씁니다. 호환 가능한 표기법이 사용되었는지 주의 깊게 확인하지 않고 다른 저자의 결과를 결합 할 때 심각한 실수가 발생할 수 있습니다. 이 두 규칙의 정의와 이들 간의 비교는 레이아웃 규칙 섹션에서 수집됩니다.

## 범주 (Scope)

Matrix calculus refers to a number of different notations that use matrices and vectors to collect the derivative of each component of the dependent variable with respect to each component of the independent variable. In general, the independent variable can be a scalar, a vector, or a matrix while the dependent variable can be any of these as well. Each different situation will lead to a different set of rules, or a separate [calculus](https://en.wikipedia.org/wiki/Calculus), using the broader sense of the term. Matrix notation serves as a convenient way to collect the many derivatives in an organized way.

행렬 미적분은 행렬과 벡터를 사용하여 독립 변수의 각 성분에 대한 종속 변수의 각 성분의 미분을 수집하는 여러 가지 다른 표기법을 말합니다. 일반적으로 독립 변수는 스칼라, 벡터 또는 행렬 일 수 있으며 종속 변수는 이들 중 하나 일 수 있습니다. 각각의 다른 상황은 더 넓은 의미의 용어를 사용하여 다른 규칙 세트 또는 별도의 미적분으로 이어질 것입니다. 행렬 표기법은 많은 파생 상품을 체계적으로 수집하는 편리한 방법입니다.

As a first example, consider the [gradient](https://en.wikipedia.org/wiki/Gradient) from [vector calculus](https://en.wikipedia.org/wiki/Vector_calculus). For a scalar function of three independent variables, $f(x_1, x_2, x_3)$, the gradient is given by the vector equation

첫 번째 예로 벡터 미적분의 기울기를 고려하십시오. 세 개의 독립 변수 인 스칼라 함수 $f(x_1, x_2, x_3)$의 경우 기울기는 벡터 방정식으로 제공됩니다.
$$
\nabla f = \frac{\partial f}{\partial x_1} \hat{x}_1 + \frac{\partial f}{\partial x_2}  \hat{x}_2 + \frac{\partial f}{\partial x_3} \hat{x}_3
$$




where $\hat  x_i$ represents a unit vector in the $x_i$ direction for $1 \le i \le 3$. This type of generalized derivative can be seen as the derivative of a scalar, $f$, with respect to a vector, $\mathbf x$, and its result can be easily collected in vector form.

여기서 $\hat x_i$는 $1 \le i \le 3$ 대한 $x_i$ 방향의 단위 벡터를 나타냅니다. 이 유형의 일반화 된 도함수는 벡터 $\mathbf x$에 대해 스칼라 $f$의 도함수로 볼 수 있으며 결과는 벡터 형태로 쉽게 수집 할 수 있습니다.
$$
\nabla f = \frac{\partial f}{\partial \mathbf{x}}^\textsf{T} = 
  \begin{bmatrix}
    \frac{\partial f}{\partial x_1} &
    \frac{\partial f}{\partial x_2} &
    \frac{\partial f}{\partial x_3} \\
  \end{bmatrix}^\textsf{T}.
$$
More complicated examples include the derivative of a scalar function with respect to a matrix, known as the [gradient matrix](https://en.wikipedia.org/wiki/Matrix_calculus#Derivatives_with_matrices), which collects the derivative with respect to each matrix element in the corresponding position in the resulting matrix. In that case the scalar must be a function of each of the independent variables in the matrix. As another example, if we have an *n*-vector of dependent variables, or functions, of *m* independent variables we might consider the derivative of the dependent vector with respect to the independent vector. The result could be collected in an *m×n* matrix consisting of all of the possible derivative combinations. There are a total of nine possibilities using scalars, vectors, and matrices. Notice that as we consider higher numbers of components in each of the independent and dependent variables we can be left with a very large number of possibilities.

보다 복잡한 예는 그래디언트 매트릭스로 알려진 매트릭스에 대한 스칼라 함수의 미분을 포함하며, 이는 결과 매트릭스에서 대응하는 위치에서 각 매트릭스 요소에 대한 미분을 수집 합니다. 이 경우 스칼라는 행렬에 있는 각 독립 변수의 함수여야 합니다. 또 다른 예로서, $m$개의 독립 변수의 종속 변수 또는 함수의 $n$벡터가있는 경우 독립 벡터에 대한 종속 벡터의 미분을 고려할 수 있습니다. 결과는 모든 가능한 유도체 조합으로 구성된 $m \times n$ 매트릭스로 수집 될 수있다. 스칼라, 벡터 및 행렬을 사용하는 총 9 가지 가능성이 있습니다. 각각의 독립 및 종속 변수에서 더 많은 수의 구성 요소를 고려할 때 매우 많은 가능성을 가질 수 있습니다.

The six kinds of derivatives that can be most neatly organized in matrix form are collected in the following table.

매트릭스 형태로 가장 깔끔하게 정리할 수있는 6 가지 유도체는 다음 표에 정리되어 있습니다.

|  타입  |                   스칼라                    |                        벡터                         |                    행렬                     |
| :----: | :-----------------------------------------: | :-------------------------------------------------: | :-----------------------------------------: |
| 스칼라 |     $$ \frac{\partial y}{\partial x} $$     |     $$ \frac{\partial \mathbf y}{\partial x} $$     | $$ \frac{\partial \mathbf Y}{\partial x} $$ |
|  벡터  | $$ \frac{\partial y}{\partial \mathbf x} $$ | $$ \frac{\partial \mathbf y}{\partial \mathbf x} $$ |                                             |
|  행렬  | $$ \frac{\partial y}{\partial \mathbf X} $$ |                                                     |                                             |

Here, we have used the term "matrix" in its most general sense, recognizing that vectors and scalars are simply matrices with one column and one row respectively. Moreover, we have used bold letters to indicate vectors and bold capital letters for matrices. This notation is used throughout.

여기서 우리는 벡터와 스칼라가 단순히 하나의 열과 하나의 행을 갖는 행렬이라는 것을 인식하면서 가장 일반적인 의미로 "행렬"이라는 용어를 사용했습니다. 또한 굵은 글자를 사용하여 벡터와 굵은 대문자로 행렬을 나타 냈습니다. 이 표기법은 전체적으로 사용됩니다.

Notice that we could also talk about the derivative of a vector with respect to a matrix, or any of the other unfilled cells in our table. However, these derivatives are most naturally organized in a [tensor](https://en.wikipedia.org/wiki/Tensor) of rank higher than 2, so that they do not fit neatly into a matrix. In the following three sections we will define each one of these derivatives and relate them to other branches of mathematics. See the [layout conventions](https://en.wikipedia.org/wiki/Matrix_calculus#Layout_conventions) section for a more detailed table.

행렬이나 표에서 채워지지 않은 다른 셀과 관련하여 벡터의 미분에 대해서도 이야기 할 수 있습니다. 그러나 이러한 파생물은 가장 자연스럽게 2보다 높은 순위의 텐서로 구성되므로 행렬에 깔끔하게 맞지 않습니다. 다음 세 섹션에서 우리는 이들 파생어 각각을 정의하고 다른 수학 분기(branches)와 관련시킵니다. 자세한 표는 레이아웃 규칙 섹션을 참조하십시오.

### Relation to other derivatives

The matrix derivative is a convenient notation for keeping track of partial derivatives for doing calculations. The [Fréchet derivative](https://en.wikipedia.org/wiki/Fréchet_derivative) is the standard way in the setting of [functional analysis](https://en.wikipedia.org/wiki/Functional_analysis) to take derivatives with respect to vectors. In the case that a matrix function of a matrix is Fréchet differentiable, the two derivatives will agree up to translation of notations. As is the case in general for [partial derivatives](https://en.wikipedia.org/wiki/Partial_derivative), some formulae may extend under weaker analytic conditions than the existence of the derivative as approximating linear mapping.

행렬 도함수는 계산을 위해 부분 도함수를 추적하기위한 편리한 표기법입니다. Fréchet 미분은 벡터에 대한 미분을 취하기 위한 기능 분석 설정의 표준 방법입니다. 행렬의 행렬 함수가 Fréchet의 미분 가능한 경우, 두 도함수는 표기법의 변환에 동의합니다. 부분 유도체의 경우와 마찬가지로, 일부 공식은 근사 선형 맵핑으로서 유도체의 존재보다 약한 분석 조건 하에서 확장 될 수 있습니다.

### 사용법 (Usage)

Matrix calculus is used for deriving optimal stochastic estimators, often involving the use of [Lagrange multipliers](https://en.wikipedia.org/wiki/Lagrange_multipliers). This includes the derivation of:

- [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter)
- [Wiener filter](https://en.wikipedia.org/wiki/Wiener_filter)
- [Expectation-maximization algorithm for Gaussian mixture](https://en.wikipedia.org/wiki/Expectation-maximization_algorithm#Gaussian_mixture)
- [Gradient descent](https://en.wikipedia.org/wiki/Gradient_descent)

행렬 미적분학은 종종 라그랑주 승수를 사용하는 최적의 확률적 추정기를 도출하는 데 사용됩니다. 여기에는 다음의 파생이 포함됩니다.

- 칼만 필터
- 위너 필터
- 가우스 혼합에 대한 기대 최대화 알고리즘
- 그라데이션 하강

## 표기법 (Notation)

The vector and matrix derivatives presented in the sections to follow take full advantage of [matrix notation](https://en.wikipedia.org/wiki/Matrix_notation), using a single variable to represent a large number of variables. In what follows we will distinguish scalars, vectors and matrices by their typeface. We will let $M(n, m)$ denote the space of [real](https://en.wikipedia.org/wiki/Real_number) $n \times m$ matrices with *n* rows and *m* columns.  Such matrices will be denoted using bold capital letters: , $\mathbf A, \mathbf X, \mathbf Y$ etc.  An element of $M(n,1)$, that is, a [column vector](https://en.wikipedia.org/wiki/Column_vector), is denoted with a boldface lowercase letter: $\mathbf a, \mathbf x, \mathbf y$ etc. An element of $M(1,1)$ is a scalar, denoted with lowercase italic typeface: $a, t, x,$ etc. $\mathbf X ^\top$ denotes matrix [transpose](https://en.wikipedia.org/wiki/Transpose), $\text{tr}(\mathbf X)$ is the [trace](https://en.wikipedia.org/wiki/Trace_(linear_algebra)), and $\text{det}(\mathbf X)$ or $|\mathbf X|$ is the [determinant](https://en.wikipedia.org/wiki/Determinant). All functions are assumed to be of [differentiability class](https://en.wikipedia.org/wiki/Differentiability_class) $C^1$ unless otherwise noted. Generally letters from the first half of the alphabet ($a, b, c, \dots$) will be used to denote constants, and from the second half $(t, x, y, \dots)$ to denote variables.

다음 섹션에 제시된 벡터 및 행렬 도함수는 단일 변수를 사용하여 많은 수의 변수를 나타내는 행렬 표기법을 최대한 활용합니다. 다음에서는 스칼라, 벡터 및 행렬을 서체로 구별합니다. $M(n, m)$은 $n$개의 행과 $m$개의 열이 있는 실제 $n \times m$ 행렬의 공간을 나타냅니다. 이러한 행렬은 굵은 대문자 ($\mathbf A, \mathbf X, \mathbf Y$ 등)를 사용하여 표시됩니다. $M(n,  1)$의 요소, 즉 열 벡터는 굵은 소문자 ($\mathbf a, \mathbf x, \mathbf y$) 로 표시됩니다. $M(1, 1)$의 요소는 스칼라이며 소문자 이탤릭체 ($a, t, x$ 등)로 표시됩니다. $\mathbf X ^\top$는 행렬 전치, $\text{tr}(\mathbf X)$는 트레이스, $\text{det}(\mathbf X)$ 또는 $|\mathbf X|$는 행렬식을 나타냅니다. 달리 명시되지 않는 한 모든 기능은 differentialbility class $C^1$ 인 것으로 가정합니다. 일반적으로 알파벳의 전반부 $(a, b, c, \dots) $문자는 상수를 나타내며, 후반부 $(t, x, y, \dots)$는 변수를 나타냅니다.

**NOTE**: As mentioned above, there are competing notations for laying out systems of [partial derivatives](https://en.wikipedia.org/wiki/Partial_derivative) in vectors and matrices, and no standard appears to be emerging yet.  The next two introductory sections use the [numerator layout convention](https://en.wikipedia.org/wiki/Matrix_calculus#Layout_conventions) simply for the purposes of convenience, to avoid overly complicating the discussion.  The section after them discusses [layout conventions](https://en.wikipedia.org/wiki/Matrix_calculus#Layout_conventions) in more detail.  It is important to realize the following:

참고 : 위에서 언급 한 바와 같이 벡터와 행렬에 부분 미분 시스템을 배치하기위한 경쟁 표기법이 있으며 아직 표준이 나오지 않은 것으로 보입니다. 다음 두 소개 섹션에서는 토론의 복잡성을 피하기 위해 편의상 단순히 분자 레이아웃 규칙을 사용합니다. 이후의 섹션에서는 레이아웃 규칙에 대해 자세히 설명합니다. 다음을 인식하는 것이 중요합니다.

1. Despite the use of the terms "numerator layout" and "denominator layout", there are actually more than two possible notational choices involved.  The reason is that the choice of numerator vs. denominator (or in some situations, numerator vs. mixed) can be made independently for scalar-by-vector, vector-by-scalar, vector-by-vector, and scalar-by-matrix derivatives, and a number of authors mix and match their layout choices in various ways.
2. The choice of numerator layout in the introductory sections below does not imply that this is the "correct" or "superior" choice.  There are advantages and disadvantages to the various layout types.  Serious mistakes can result from carelessly combining formulas written in different layouts, and converting from one layout to another requires care to avoid errors.  As a result, when working with existing formulas the best policy is probably to identify whichever layout is used and maintain consistency with it, rather than attempting to use the same layout in all situations.

1. "분자 레이아웃"및 "분모 레이아웃"이라는 용어를 사용 함에도 불구하고 실제로 두 가지 이상의 가능한 표기법 선택이 있습니다. 그 이유는 분자 대 분모 (또는 어떤 상황에서는 분자 대 혼합)를 scalar-by-vector, vector-by-vector, vector-by-vector 그리고 scalar-by-matrix derivatives, 그리고 다양한 방식으로 레이아웃 선택을 혼합하고 일치시킵니다.
2. 아래 소개 섹션에서 분자 레이아웃을 선택한다고해서 이것이 "올바른"또는 "우수한"선택임을 암시하지는 않습니다. 다양한 레이아웃 유형에는 장단점이 있습니다. 다른 레이아웃으로 작성된 수식을 부주의하게 조합하면 심각한 실수가 발생할 수 있으며 한 레이아웃에서 다른 레이아웃으로 변환하면 오류가 발생하지 않도록 주의해야 합니다. 결과적으로, 기존 공식을 사용하여 작업 할 때 최상의 정책은 모든 상황에서 동일한 레이아웃을 사용하려고 시도하는 대신 사용되는 레이아웃을 식별하고 일관성을 유지하는 것입니다.

### 대안 (Alternatives)

The tensor index notation with its Einstein summation convention is very similar to the matrix calculus, except one writes only a single component at a time. It has the advantage that one can easily manipulate arbitrarily high rank tensors, whereas tensors of rank higher than two are quite unwieldy with matrix notation. All of the work here can be done in this notation without use of the single-variable matrix notation. However, many problems in estimation theory and other areas of applied mathematics would result in too many indices to properly keep track of, pointing in favor of matrix calculus in those areas. Also, Einstein notation can be very useful in proving the identities presented here (see section on differentiation) as an alternative to typical element notation, which can become cumbersome when the explicit sums are carried around. Note that a matrix can be considered a tensor of rank two.

아인슈타인 합산 규칙 (Einstein summation convention)을 사용한 텐서 인덱스 표기법은 행렬 미적분학과 매우 유사하지만 한 번에 하나의 구성 요소만 씁니다. 임의의 높은 순위 텐서를 쉽게 조작 할 수 있다는 장점이 있지만, 2보다 높은 순위의 텐서는 매트릭스 표기법에 비해 다루기 힘듭니다. 여기에서 모든 작업은 단일 변수 매트릭스 표기법을 사용하지 않고이 표기법으로 수행 할 수 있습니다. 그러나 추정 이론과 응용 수학의 다른 영역에서 많은 문제는 너무 많은 지수를 적절히 추적하여 그 영역에서 행렬 미적분을 선호합니다. 또한 아인슈타인 표기법은 전형적인 요소 표기법의 대안으로 여기에 제시된 동일성을 증명하는 데 매우 유용 할 수 있습니다 (분화에 대한 섹션 참조). 명시적 합계가 수행 될 때 번거로울 수 있습니다. 행렬은 순위 2의 텐서로 간주 될 수 있습니다.

## Derivatives with vectors

Main article: [Vector calculus](https://en.wikipedia.org/wiki/Vector_calculus)

Because vectors are matrices with only one column, the simplest matrix derivatives are vector derivatives.
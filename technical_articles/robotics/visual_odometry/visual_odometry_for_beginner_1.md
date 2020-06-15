이 글은 https://snacky.tistory.com/96이 수식이 제대로 표시 되지 않아서 옮겨 왔습니다.

---

본 내용은 링크된 **Avi Singh's blog** 의 **[블로그 포스팅](https://avisingh599.github.io/vision/visual-odometry-full/)**을 번역한 글입니다.

영어를 그다지 잘 하는 편이 아니라 오역이나 의역을 넘어선 발번역이 매우 많을 것 같습니다ㅠㅠ 잘못된 부분이 있다면 알려주시는 대로 수정하도록 하겠습니다.

편의상 Visual Odometry 라는 용어는 그대로 사용하거나 VO 로 줄여서 사용하였으며, Trajectory 라는 말을 사선으로 번역하면 잘 와닿지가 않아서 궤적으로 번역했습니다.

번역을 계속 하다보니 영어를 그대로 사용하거나, 조금 더 나은 표현이 있는 듯 하여 바꾸었고, 그렇다보니 두번째 번역과 용어차이가 많네요ㅠㅠ 조만간 수정하도록 하겠습니다.

---

## 초보자를 위한 Visual Odometry

### 시작부터 튜토리얼까지

저는 Visual Odometry 에 대해 몇 달 전 포스트를 작성했지만, 실제로 구현한 내용을 올리지는 않았었습니다. 저는 이  블로그 포스팅이 처음으로 Visual Odometry 를 각자의 로봇에 적용하고자 하는 분들께 시작점으로써 도움이 되었으면  좋겠습니다. 저는 기본적으로 [Real-Time Streo Visual Odometry for Autonoumous Ground Vehicles(Howard2008) ](https://www-robotics.jpl.nasa.gov/publications/Andrew_Howard/howard_iros08_visodom.pdf)논문에 소개된 알고리즘에, 제가 수정한 내용을 포함하여 설명드릴 것입니다. 이 논문은 조금 오래된 편이기는 하지만 이해하기가 쉽기 때문에, 저는 이 논문을 첫 시작점으로 삼았습니다. MATLAB 소스코드는 [깃헙](https://github.com/avisingh599/vo-howard08)에서 보실 수 있습니다.

**Odometry 란 무엇인가?**

자동차의 계기판에 주행거리를 보여주는 작은 기계장치를 보신 적이 있나요? 이것이 [Odometry](https://en.wikipedia.org/wiki/Odometer) 라고 불리는 것입니다. 이는 (아마도) 바퀴의 회전수를 측정하고, 주행 거리를 구하기 위해 이 값에 바퀴의 둘레를 곱하여 측정하는 것입니다. 로봇 공학에서 [Odometry](http://simreal.com/content/Odometry) 는 좀 더 보편적인 용어입니다. 그리고 단순히 주행거리를 측정하는 것 뿐만 아니라, 움직이는 로봇의 전체적인 궤적을 의미하기도  합니다. 어떤 시간 t 에 대해서, 로봇의 완전한 위치를 알려주는 벡터값 $[x^t y^t z^t \alpha^t \beta^t  \gamma^t]$ 가 있습니다. $\alpha^t, \beta^t, \gamma^t$ 는 [오일러 각](http://mathworld.wolfram.com/EulerAngles.html)으로 알려저 있고, $x^t, y^t, z^t$ 는 카테시안 좌표계(직교좌표계)로 알려져 있습니다.

**Visual Odometry 란 무엇인가?**

 움직이는  로봇의 궤적을 결정하는데에는 하나 이상의 방법들이 있습니다. 그러나 우리가 이 포스팅에서 중점적으로 볼 것은 Visual  Odometry 라고 불리우는 것입니다. 이 접근법에서는 우리는 움직이는 물체(자동차나 로봇과 같은) 물체에 단단히 붙여놓은  카메라를 가지고 있어야 합니다. 그리고 우리가 할 것은 이 카메라를 통해 들어오는 비디오 스트림을 이용하여 [6-DOF](http://en.wikipedia.org/wiki/Six_degrees_of_freedom) 궤적을 구하는 것입니다. 우리가 단 한 개의 카메라만을 이용할 때, 이를 **Monoclar Visual Odometry** 라고 부르며, 2개 이상의 카메라를 사용할 때 이를 **Streo Visual Odometry** 라고 부릅니다.

**왜 Streo 또는 Monocular 를 사용하는가?**

이 두 가지 방식을 사용하는 데에는 각기 장단이 있습니다. 그리고 저는 이 포스팅에서 한 가지만 주로 설명할 것입니다(이제  stereo 방식에 대해서만 설명하겠지만, 아마 제가 monocular 에 대해서도 추가적으로 포스팅을 작성하거나 문서를 작성할  수도 있습니다). stereo 방식의 이점은 정확한 궤적을 추측할 수 있다는 것입니다. 반면에 monocular 방식으로는 같은  scale factor를 가질 뿐인 궤적만 추측할 수 있습니다. 그래서 monocular VO 로는 단지, x 축으로 1 단위  만큼 움직였고, y 축으로 2 단위 만큼 움직였다 라는 식으로 말할 수 있을 뿐이지만, streo VO 로는 x 축으로 1m, y 축으로 2m 같은 식으로 이야기 할 수 있습니다. 또한 streo VO 는 보통 더 믿을만 합니다(더 많은 데이터를 사용할 수  있기  때문에). 그러나, 카메라로부터 (streo 카메라끼리의 사이와 비교해) 물체의 거리가 너무 먼 경우에는 monocular 방식보다 stereo 방식이 좋지 않습니다. 그래서 아주 작은 로봇 ([robobees](http://robobees.seas.harvard.edu/publications) 같은) 이라고 하면,  stereo 방식은 쓸모가 없고, [SVO](https://github.com/uzh-rpg/rpg_svo) 같은 monocular VO 알고리즘을 사용하는 것이 더 좋을 수 있습니다. 또한 드론은 점점 더 소형화 되는 추세이고, [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html) 와 같은 그룹들은 현재 monocular VO 접근에 좀 더 초점을 맞추고 있습니다(그는 내가 우연히 끼었던 대화에서 그렇게 말했습니다).

**말로는 충분히 했고, 이제 수학에 대해 말해보자**

**해당 문제의 공식화(Fomulation)**

**Input**

우리는 카메라 쌍으로부터 오는  (grayscale/color) 이미지들의 stream을 가지고 있습니다. 시간 $t$ 와 $t+1$ 에 좌측과 우측 카메라로부터 캡쳐된 프레임들을 각각 $I_l^t, I_r^t, I_l^{t+1}, I_r^{t+1}$ 이라고 하겠습니다. 또한 우리는 이미  수없이 많은 stereo 캘리브레이션 알고리즘들 중 하나를 통해,  stereo 장비의 내부(intrinsic) 및  외부(extrinsic) 캘리브레이션 파라미터들을 알고 있습니다.

**Output**

모든 stereo 이미지 쌍에 대하여,  우리는 회전 행렬(Rotation Matrix) $R$ 과 병진 벡터(Translation Vector) $t$ 를 구할 필요가  있으며, 이는 두 프레임 사이에서 물체의 움직임을 통해 알 수 있습니다.

**알고리즘**

개요:

1. 캡쳐 이미지: $I_l^t, I_r^t, I_l^{t+1}, I_r^{t+1}$

2. 위 이미지들에 대한 왜곡 보정 및 수정

3. $I_l^t, I_r^t$ 로부터 차이도 맵(Disparity Map) $D^t$ 를, $I_l^{t+1}, I_r^{t+1}$ 로부터 $D^{t+1}$ 을 계산

4. FAST 알고리즘을 사용하여 $I_l^t, I_l^{t+1}$ 로부터 특징점들을 찾고, 이를 매칭

5. 차이도 맵 $D^t, D^{t+1}$ 을 사용하여, 이전 단계에서 탐지된 특징점들의 3차위 위치를 계산. 포인트 클라우드 $W^t, W^{t+1}$ 을 얻음

6. 위 포인트 클라우드로부터, 상호 호환 가능한 매칭들로 점들의 부분집합을 선택

7. 이전 단계에서 탐지된 Inlier 들로부터 $R, t$를 추측

여러분이 위에서 보신 내용들 중 차이도 맵(Disparity Map) 이나 FAST 특징점들 등과 같은 전문용어를 이해하지 못했다고 해도 걱정하지 않으셔도 됩니다. 위 내용의 대부분은 MATLAB 에서 코드를 통해, 아래쪽에 훨씬 더 자세히 설명할 것입니다.

**왜곡 보정 및 수정**

차이도 맵을 계산하기에 앞서, 우리는 몇몇 사전단계를 수행해야만 합니다.

 왜곡 보정(Undistortion): 이 단계는 렌즈 왜곡에 대한 처리입니다. 이는 캘리브레이션을 통해 얻은 왜곡 파라미터들로 수행 가능합니다.

 수정(Rectification): 이 단계는 차이도 맵의 문제를 완화시켜주는 역할을 합니다. 이 단계 후에, 에피폴라 라인(Epiplar line)은 수평으로  평행하게 되고, 차이도 계산 단계에서 이는 한쪽 방향으로만 매칭 블록을 찾는 역할을 할 것입니다.

![특징 매칭이 수평 라인으로 평행한 것을 확인할 수 있는 KITTI 데이터 셋으로부터 오버레이된 Streo 이미지](visual_odometry_for_beginner_1.assets/2436934A5969853531)

이 두 연산은 MATLAB 을 통해 수행되고, 제가 이 과정에서 사용했던 KITTI Visual Odometry 데이터 셋은 이미 이 연산 수행을 마친 상태였기 때문에, 제가 수행한 내용에서 해당 내용에 대한 코드는 보실 수 없습니다. 만약 해당 방법을 보고 싶으시다면 [여기](http://kr.mathworks.com/help/vision/ref/rectifystereoimages.html?searchHighlight=rectifyStereoImages)와 [여기](http://kr.mathworks.com/help/vision/ref/undistortimage.html)를 보실 수 있습니다. 이 연산 수행을 위해 사용된 툴은 MATLAB R2014a 버전 혹은 그 이상의 버전을 사용해야 합니다.

**차이도 맵(Disparity Map) 계산**

Stereo 카메라로부터 주어진 이미지 쌍에 대해, 우리는 차이도 맵을 계산할 수 있습니다. 물리 세계에서 측정 3차원 $F$ 에서, 좌측  이미지에 $(x, y)$ 지점이 위치하고, 두번째 이미지에 똑같은 특징점이 $(x+d, y)$ 에 위치한다고 하겠습니다. 그러면  차이도 맵 위의 위치 $(x, y)$ 는 값으로 $d$ 를 가집니다. y-축의 경우 이미지가 수정(Rectify)되었기 때문에  동일합니다. 결국, 우리는 이미지 평면에서 각각의 점들마다의 차이도를 다음과 같이 정의할 수 있습니다.
$$
d=x_l-x_r
$$
KITTI VO 데이터 셋으로부터의 프레임들로 계산된 차이도 맵

**블록 매칭 알고리즘**

 각각의 포인트에서 차이도는 슬라이딩 윈도우(Sliding Window) 를 이용하여 계산됩니다. 좌측 이미지의 모든 픽셀에 대해서  15x15 픽셀 너비의 창(Window)이 픽셀들 주위에 생성되며, 그 창 안에 모든 픽셀들의 값이 저장됩니다. 또한 동시에 우측 이미지에서도 이 창이 생성됩니다. 그리고 이는 차이의 절대값 합(SAD) 가 최소가 될때까지 수평으로 슬라이딩 됩니다. 우리가  수행하는 알고리즘은 이러한 블록 매칭 기술의 향상된 버전으로, [Semi-Global Block Matching algorithm](http://zone.ni.com/reference/en-XX/help/372916M-01/nivisionconceptsdita/guid-53310181-e4af-4093-bba1-f80b8c5da2f4/) 이라고 불립니다. 이 알고리즘은 MATLAB 에서 바로 수행할 수 있습니다:

```matlab
disparityMap1 = disparity(I1_l, I1_r, 'DistanceThreshold', 5);
```

**특징 찾기(Feature Detection)**

제가 접근한 방식에서는 FAST 코너 디텍터(FAST corner detector) 를 이용합니다. 저는 이제 디텍터가 어떻게 작동하는지 짧게 설명할 것입니다. 그렇지만 정말로 이 방식에 대해서 이해하시려면 여기에 있는 [원본 논문과 소스코드](http://www.edwardrosten.com/work/fast.html) 를 보셔야만 합니다. 코너가 있거나 혹은 없다면 우리가 테스트를 하기를 원하는 점 $P$ 가 있다고 가정해 보겠습니다. 그리고  아래의 그림과 같이 16px 의 둘레를 가지는 원을 그립니다. 이 원의 둘레의 위에 있는 모든 픽셀들에 대해서, 우리는 어떤  연속적인 픽셀들의 집합이 어떤 팩터 $I$ 를 가지고, 원래의 픽셀보다 강도가 더 강한 픽셀들의 집합이 존재하는지, 동일한 팩터  $I$ 에 대해 인접한 픽셀들의 집합이 있는지 확인합니다. 만약 그렇다면, 우리는 이 포인트를 코너로 봅니다. 코너가 아닌 부분을 제외하는 데에는 경험적 방법(Heuristic)이 사용되는데, 1, 9, 5, 13 에 위치한 픽셀에 대해 처음 시행되고, 그들 중 셋 이상은 최소한 $I$ 의 양보다 더 강한 강도를 가지거나, 코너가 되는 점에 대해 동일한 $I$ 양에 의해 더 낮은  강도를 가져야만 합니다.(역자: P를 기준으로 봤을 때, 밝기 기준을 비율 $I$ 로 정하고, 해당 비율보다 밝은 점들의 집합  혹은 어두운 점들의 집합이 셋 이상이면 코너로 본다는 의미같습니다.) 이 부분적인 접근법은 SIFT 알고리즘으로 다른 포인트  디텍터들과 비교하여 계산이 더 효율적이기 때문에 선택되었습니다.

![원래의 FAST 특징점 찾기 논문에 소개된 이미지](visual_odometry_for_beginner_1.assets/22534B45596E146B06)

우리가 이 접근법에서 하는 또다른 것은 "버켓팅(Bucketing)" 이라고 불리는 것 때문입니다. 우리가 전체 이미지에  대해서 특징점을 찾고자 한다면, 이미지의 어떤 부분에는 특징점이 몰려있는 반면, 어떤 부분에는 없을 확률이 높습니다. 그리고 우리 알고리즘은 어떤 정적인 장면의 추정에 의존하기 때문에 이를 사용하는 것은 썩 좋지 않고, "실제" 정적인 장면을 찾기 위해서,  우리는 어떤 영역을 찾는 대신에 이미지 전체를 보아야만 합니다. 이러한 이슈에 대한 조치를 위해서 우리는 이미지를 그리드로  나눕니다(대략 100x100px). 그리고 나누어진 각각의 그리드들에 대하여 최대 20개의 피쳐들을 추출하여, 더 균일한 특징점  분포를 유지합니다.

코드에서는, 아래와 같이 수행합니다.

```matlab
points1_l = bucketFeatures(I1_l, h, b, h_break, b_break, numCorners);
```

 이 라인의 함수는 다음과 같습니다:

```matlab
function points = bucketFeatures(I, h, b, h_break, b_break, numCorners)

y = floor(linspace(1, h - h/h_break, h_break));
x = floor(linspace(1, b - b/b_break, b_break));

final_points = [];
for i=1:length(y)
    for j=1:length(x)
    roi =   [x(j),y(i),floor(b/b_break),floor(h/h_break)];
    corners = detectFASTFeatures(I, 'MinQuality', 0.00, 'MinContrast', 0.1, 'ROI',roi );
    corners = corners.selectStrongest(numCorners);
    final_points = vertcat(final_points, corners.Location);
    end
end
points = cornerPoints(final_points);
```

 보시다시피, 이미지는 그리드로 나누어지고, 각 그리드에서 가장 강한 코너들이 그 하위 단계에서 선택됩니다.

**특징 설명 및 매칭(Feature Description and Matching)**

 이전 단계에서 탐지된 FAST 코너는 [KLT  tracker](https://www.ces.clemson.edu/~stb/klt/) 를 사용하는 다음 단계에서 이용됩니다. KLT 트래커는 기본적으로, 트래킹되는 모든 코너를 보고, 다음 이미지에서 코너를 찾는 데에 이러한 일부의 정보를 이용합니다. 여러분은 KLT 링크를 통해 더 많은 정보를 얻을 수 있습니다. $I_l^t$ 에서 탐지된  코너는 $I_l^{t+1}$ 에서 트래킹됩니다. $I_l^t$ 에서 탐지된 특징점들의 집합을 $F^t$ 라고 하고,  $I_l^{t+1}$ 에서 탐지된 특징점들의 집합을 $F^{t+1}$ 라고 하겠습니다.

 MATLAB 에서, 이를 수행하는 것은 매우 쉽습니다. 그저 아래의 라인 세 줄을 작성하여 트래커를 초기화 시키면 됩니다. 한 번 해 보세요.

```matlab
tracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker, points1_l.Location, I1_l);
[points2_l, validity] = step(tracker, I2_l);
```

제가 최근 구현한 것을 상기해 보세요, 저는 단지 어떤 프레임으로부터 다음 프레임까지 특정 점을  찾아 트래킹 했을 뿐이며, 또다시 찾는 과정을 수행하고 있습니다. 그러나 추가적인 구현으로, 이는 특정 임계점 이하의 숫자로  점들의 갯수가 떨어지지 않는 한은 계속될 트래킹 하게 됩니다.

제가 최근 구현한 것을 상기해 보세요, 저는 단지 어떤 프레임으로부터 다음 프레임까지 특정 점을 찾아 트래킹 했을 뿐이며,  또다시 찾는 과정을 수행하고 있습니다. 그러나 추가적인 구현으로, 이는 특정 임계점 이하의 숫자로 점들의 갯수가 떨어지지 않는  한은 계속될 트래킹 하게 됩니다.

**3D 포인트 클라우드의 삼각 측량 (Triangulation of 3D PointCloud)**

$F^t$ 와 $F^{t+1}$ 에서의 모든 점의 실제 세계 3D 좌표는 두 카메라의 투사 행렬(Projection matrices)로  알려진 차이도 맵 $P_1$ 과 $P_2$ 로부터 이 특징점에 대응하는 차이도 값을 사용하여 좌측 카메라 측면에서 계산됩니다.  우리는 처음에 $P_1$ 과 $P_2$ 로부터의 데이터를 이용하여 재투사 행렬 $Q$ 를 만듭니다.
$$
Q=\begin{bmatrix}1 & 0 & 0 & -c_x\\0 & 1 & 0  & -c_y\\0 & 0 & 0 & -f\\0 & 0 & -1/T_x &  0\end{bmatrix}
$$


$c_x$ = 좌측 카메라의 광학 중심(optical center)의 x-좌표(픽셀 단위)

$c_y$ = 좌측 카메라의 광학 중심(optical center)의 y-좌표(픽셀 단위)

$f$ = 첫번째 카메라의 초점 거리(focal length)

$T_x$ = 첫번째 카메라 측면에서 우측 카메라의 x-좌표(미터 단위)

우리는 $F_l^t$ 와 $F_l^{t+1}$ 의 모든 특징점의 3D 좌표를 얻기 위해서 다음의 관계를 이용합니다.
$$
\begin{bmatrix}X\\Y\\Z\\1\end{bmatrix}=Q\times\begin{bmatrix}x\\y\\d\\1\end{bmatrix}
$$
얻은 포인트 클라우드들의 집합을 $W^t$ 와 $W^{t+1}$ 라고 하겠습니다. 위의 방정식에 이어지는 지오메트리에 대해 더 잘 이해하기 위해서, 여러분은 비쥬얼 지오메트리의 바이블 (Hartley and Zisserman's [Multiple View Geometry](http://www.robots.ox.ac.uk/~vgg/hzbook/) 을 볼 수 있습니다.

**인라이어 탐지 단계(The Inlier Detection Step)**

이  알고리즘은 아웃라이어(outlier) 탐지 단계는 있지만, 인라이어(inlier) 탐지 단계가 있다는 부분에서 대부분의 다른  비쥬얼 오도메트리 알고리즘들과의 차이를 가집니다. 우리는 (영상의) 장면을 고정된 것으로 가정하기 때문에, 임의의 시간 $t$ 와 $t+1$  사이에 변하지 않습니다. 결과적으로, 포인트 클라우드 $W^t$ 안에서의 두 특징점 간의 거리는 그에 대응하는  $W^{t+1}$ 안의 포인트 간의 거리와도 동일해야 합니다. 만약 거리가 같지 않다면, 최소한 두 특징점 중 하나의 3D 삼각  측량에 오류가 있거나, 다음 단계에서 사용할 수 없는 움직임이 삼각 측량 되었다고 봐야 합니다. 일관성 있게 최대로 일치되는  집합을 찾기 위해서, 우리는 행렬 M을 다음과 같이 만듭니다 :
$$
M_{i,j}=\begin{cases}1, & \text{if the distance between i and j points is same in both the point clouds} \\0, & \text{oterwise}\end{cases}
$$
원래의 포인트 클라우드들로부터, 우리는 이제 각각(환원 일관성 행렬(the reduced consistency matrix) 의  요소는 모두 1)의 일관성을 가지는 부분집합들 중 가장 큰 일관성을 가지는 부분집합을 선택하려고 합니다. 이 문제는 인접 행렬  $M$ 을 이용한 [최대 도당 문제(Maximum Clique Problem)](http://en.wikipedia.org/wiki/Clique_problem)와 동일합니다. 도당(clique)들은 기본적으로 어떤 그래프의 부분집합이며, 모두 서로 연결된 노드들만을 포함합니다. 이를  시각화하는 가장 쉬운 방법은 사회망으로서 그래프를 생각하는 것이며, 각각이 모두 알고있는 사람들의 거대한 집합을 찾는 것으로  생각할 수 있습니다.

![img](visual_odometry_for_beginner_1.assets/215D8F44596E148E29)

이는 도당이 어떻게 보이는지입니다.

이 문제는 NP-완전 문제로 알려져 있습니다. 그렇기 때문에 어떠한 경우에도 최적의 해는 찾아질 수 없습니다. 그러므로 우리는 최적해에 가까운 도당을 얻기 위해 탐욕적인 경험 방법(Greedy heuristic) 을 이용합니다:

1.  최대 차수의 노드를 선택하고, 이 노드를 포함하는 도당을 초기화 합니다.

2.  기존의 도당으로부터, 도당에 연결된 모든 노드들의 부분집합 $v$ 를 정합니다.

3.  집합 $v$ 에서, $v$ 안의 다른 노드들과 가장 많이 연결되는 노드를 선택합니다.

위의 알고리즘은 아래 제가 코드로 작성한 두 함수에 의해 수행됩니다 :

```matlab
function cl = updateClique(potentialNodes, clique, M)

maxNumMatches = 0;
curr_max = 0;
for i = 1:length(potentialNodes)
    if(potentialNodes(i)==1)
        numMatches = 0;
        for j = 1:length(potentialNodes)
            if (potentialNodes(j) & M(i,j))
                numMatches = numMatches + 1;
            end
        end
        if (numMatches>=maxNumMatches)
            curr_max = i;
            maxNumMatches = numMatches;
        end
    end
end

if (maxNumMatches~=0)
    clique(length(clique)+1) = curr_max;
end

cl = clique;


function newSet = findPotentialNodes(clique, M)

newSet = M(:,clique(1));
if (size(clique)>1)  
    for i=2:length(clique)
        newSet = newSet & M(:,clique(i));
    end
end

for i=1:length(clique)
    newSet(clique(i)) = 0;
end
```

**$R$ 과 $t$ 의 계산(Computation of $R$ and $t$)**

 회전 행렬 $R$ 과 병진 벡터 $t$ 를 결정하기 위해서, 우리는 아래 따라오는 합을 최소화하는 Levenberg-Marquardt 비선형 최소 차승법을 이용합니다:

$$\epsilon=\sum_{F^t, F^{t+1}} (j_t-PTw_{t+1})^2+(j_{t+1}-PT^{-1}w_t)^2$$

$F^t, F^{t+1}$ : 시간 $t$ 와 $t+1$ 에서 좌측 이미지의 특징점들

$j_t, j_{t+1}$ : 특징점 $F^t, F^{t+1}$ 의 2D Homogeneous 좌표

$w_t, w_{t+1}$ : 특징점 $F^t, F^{t+1}$ 의 3D Homogeneous 좌표

$P$ : 좌측 카메라의 $3\times4$ 투사 행렬(Projection matrix)

$T$ : $4\times4$ Homogeneous 변환 행렬

MATLAB 에서의 최적화 도구는 lsqnonlin 함수를 통해 Levenberg-Marquardt 알고리즘을 바로 적용하며, 이 함수는  최소화되기 위해 필요한 vector objective 함수와 변형될 수 있는 파라미터들의 집합이 필요합니다.

여기에 MATLAB 으로 최소화 함수를 어떻게 작성하는지가 있습니다. 알고리즘의 해당 파트는 가장 비용이 많이 드는 부분입니다.

```matlab
function F = minimize(PAR, F1, F2, W1, W2, P1)
r = PAR(1:3);
t = PAR(4:6);

F = zeros(2*size(F1,1), 3);
reproj1 = zeros(size(F1,1), 3);
reproj2 = zeros(size(F1,1), 3);

dcm = angle2dcm( r(1), r(2), r(3), 'ZXZ' );
tran = [ horzcat(dcm, t); [0 0 0 1]];

for k = 1:size(F1,1)
    f1 = F1(k, :)';
    f1(3) = 1;
    w2 = W2(k, :)';
    w2(4) = 1;
    
    f2 = F2(k, :)';
    f2(3) = 1;
    w1 = W1(k, :)';
    w1(4) = 1;
    
    f1_repr = P1*(tran)*w2;
    f1_repr = f1_repr/f1_repr(3);
    f2_repr = P1*pinv(tran)*w1;
    f2_repr = f2_repr/f2_repr(3);
    
    reproj1(k, :) = (f1 - f1_repr);
    reproj2(k, :) = (f2 - f2_repr);    
end

F = [reproj1; reproj2];
```

**결과의 유효성 검증(Validation of results)**

$R$ 과 $t$ 의 특정 집합은 다음과 같은 조건을 만족할 경우 유효합니다:

1. 도당의 특징점의 숫자가 최소 8이다.

2. 재투사 에러(reprojection error) $\epsilon$ 이 어떤 기준점 이하이다.

위의 제약조건은 노이즈 데이터를 처리하는 데에 도움이 됩니다.

**중요한 "hack"**

여러분이 실제 세계 시퀀스에서 위 알고리즘을 수행한다면, 다소 큰 문제가 발생할 것입니다. 트럭이나 밴과 같은 거대한 이동체가 카메라의  시야 대부분을 차지하는 경우에는 장면의 강성(scene regidity) 가정이 깨집니다. 그러한 대이터를 다루기 위해서, 우리는 간단한 hack 을 소개합니다: 주된 움직임이 순방향일 경우 그냥 변환/회전 행렬을 수용합니다. 이는 KITTI 데이터 셋에서  상당한 결과의 향상을 가져오는 것으로 알려져 있습니다. 비록 여러분이 이 hack 이 대부분의 논문에 명시적으로 서술되어 있는 것을 찾지 못했더라도 말입니다.

## 참조

- https://snacky.tistory.com/96
# IMU 소스코드에 대한 이해

[소스](https://github.com/ccny-ros-pkg/imu_tools/blob/indigo/imu_complementary_filter/src/complementary_filter.cpp)

벡터 정규화
$$
n = \sqrt{x^2 + y^2 + z^2}
\\
x = \frac{x}{n}
\\
y = \frac{y}{n}
\\
z = \frac{y}{n}
$$

```c
void normalizeVector(double& x, double& y, double& z)
{
  double norm = sqrt(x*x + y*y + z*z);

  x /= norm;
  y /= norm;
  z /= norm;
}
```

사원수 정규화
$$
n = \sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}
\\
q_0 = \frac{q_0}{n}
\\
q_1 = \frac{q_1}{n}
\\
q_2 = \frac{q_2}{n}
\\
q_3 = \frac{q_3}{n}
$$

```
void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
  double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm;  
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}
```

역사원수
$$
\hat q_0 = q_0 \\
\hat q_1 = -q_1 \\
\hat q_2 = -q_2 \\
\hat q_3 = -q_3 \\
$$

```c
void invertQuaternion(
  double q0, double q1, double q2, double q3,
  double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv)
{
  // Assumes quaternion is normalized.
  q0_inv = q0;
  q1_inv = -q1;
  q2_inv = -q2;
  q3_inv = -q3;
}
```

사원수 스케일

```c
void scaleQuaternion(
  double gain,
  double& dq0, double& dq1, double& dq2, double& dq3)
{
	if (dq0 < 0.0)//0.9
  {
    // Slerp (Spherical linear interpolation):
    double angle = acos(dq0);
    double A = sin(angle*(1.0 - gain))/sin(angle);
    double B = sin(angle * gain)/sin(angle);
    dq0 = A + B * dq0;
    dq1 = B * dq1;
    dq2 = B * dq2;
    dq3 = B * dq3;
  }
  else
  {
    // Lerp (Linear interpolation):
    dq0 = (1.0 - gain) + gain * dq0;
    dq1 = gain * dq1;
    dq2 = gain * dq2;
    dq3 = gain * dq3;
  }

  normalizeQuaternion(dq0, dq1, dq2, dq3);  
}
```

사원수 곱

```c
void quaternionMultiplication(
  double p0, double p1, double p2, double p3,
  double q0, double q1, double q2, double q3,
  double& r0, double& r1, double& r2, double& r3)
{
  // r = p q
  r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
  r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
  r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
  r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}
```

사원수와 벡터의 곱

```c
void rotateVectorByQuaternion( 
  double x, double y, double z,
  double q0, double q1, double q2, double q3,
  double& vx, double& vy, double& vz)
{ 
  vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
  vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
  vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
}
```

```c
double ComplementaryFilter::getAdaptiveGain(double alpha, double ax, double ay, double az)
{
  double a_mag = sqrt(ax*ax + ay*ay + az*az);
  double error = fabs(a_mag - kGravity)/kGravity;
  double factor;
  double error1 = 0.1;
  double error2 = 0.2;
  double m = 1.0/(error1 - error2);
  double b = 1.0 - m*error1;
  if (error < error1)
    factor = 1.0;
  else if (error < error2)
    factor = m*error + b;
  else 
    factor = 0.0;
  //printf("FACTOR: %f \n", factor);
  return factor*alpha;
}
```







## 참조

[IMU Tool](https://github.com/ccny-ros-pkg/imu_tools/blob/indigo/imu_complementary_filter/src/complementary_filter.cpp)










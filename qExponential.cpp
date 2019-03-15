#include <iostream>
#include <Eigen/Dense>
#include <math.h>

//Quaternion Exponential
//implements simple 0th-order integration
//ref: quaternion kinematics, section 4.6.1 
using Eigen::Quaternionf;
using Eigen::Vector3f;
Quaternionf qExponential(float dt, Vector3f w)
{
  float wn = w.norm();
  Vector3f wN = w.normalized();
  Quaternionf qReturn;
  qReturn.w() = cos(wn * dt / 2);
  qReturn.vec() = wN * sin(wn * dt / 2);
  return qReturn;
}

int main()
{
  using Eigen::Quaternionf;
  using Eigen::Vector3f;
  float dt = 0.01;
  Vector3f w(1.0, 2.0, -3.0);
  Quaternionf qExp = qExponential(dt, w);
  std::cout << qExp.w() << std::endl << qExp.vec() << std::endl;
}
  

#include <iostream>
#include <Eigen/Dense>

//Quaternion Multiplication
using Eigen::Quaternionf;
using Eigen::Vector3f;

Quaternionf qMultiply(Quaternionf q1, Quaternionf q2)
{
  float w1 = q1.w();
  float w2 = q2.w();
  Vector3f v1 = q1.vec();
  Vector3f v2 = q2.vec();
  float wReturn = w1*w2 - v1(0)*v2(0) - v1(1)*v2(1) - v1(2)*v2(2);
  Vector3f vReturn;
  vReturn(0) = w1*v2(0) + v1(0)*w2 + v1(1)*v2(2) - v1(2)*v2(1);
  vReturn(1) = w1*v2(1) - v1(0)*v2(2) + v1(1)*w2 + v1(2)*v2(0);
  vReturn(2) = w1*v2(2) + v1(0)*v2(1) - v1(1)*v2(0) + v1(2)*w2;
  Quaternionf qReturn;
  qReturn.w() = wReturn; 
  qReturn.vec() = vReturn;
  return qReturn;
}

int main()
{
  float w1 = 2;
  float w2 = 7;		
  Vector3f v1(0.5, 0.2, 0.7);
  Vector3f v2(0.1, 0.9, 1.5); 
  Quaternionf q1;
  q1.w() = w1;
  q1.vec() = v1;
  Quaternionf q2;
  q2.w() = w2;
  q2.vec() = v2;
  Quaternionf qProduct = qMultiply(q1, q2);
  std::cout << qProduct.w() << std::endl << qProduct.vec() << std::endl;
}

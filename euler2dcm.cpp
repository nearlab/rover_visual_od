#include <iostream>
#include <Eigen/Dense>
#include <math.h>
//#include <unsupported/Eigen/MatrixFunctions>

//Euler angles to DCM
//e = [phi, theta, psi]
//3-1-2 rotation
//ref: Todd Humphreys euler2dcm MATLAB function, "Aerial Robotics," 2019
using Eigen::Matrix3f;
using Eigen::Vector3f;
Matrix3f euler2dcm(Vector3f e)
{
  std::cout << "Calculating trig values..." << std::endl;
  float cPhi = cos(e(0)); 
  float sPhi = sin(e(0));
  float cThe = cos(e(1)); 
  float sThe = sin(e(1));
  float cPsi = cos(e(2)); 
  float sPsi = sin(e(2));
  std::cout << "Building DCM..." << std::endl;
  Matrix3f DCM; 
  DCM << (cPhi*cThe - sPhi*sPsi*sThe), (cThe*sPsi + cPsi * sPhi*sThe), (-cPhi*sThe), 
         (-cPhi*sPsi),                                    (cPhi*cPsi),         sPhi,
         (cPsi*sThe + cThe*sPhi*sPsi), (sPsi*sThe - cPsi*cThe*sPhi),    (cPhi*cThe);
  return DCM;
}


int main()
{
  using Eigen::Vector3f; 
  using Eigen::Matrix3f;
  using Eigen::Quaternionf;
  /*Matrix3f A;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  std::cout << (A*5) << std::endl;*/
  Vector3f v(1.5708, -1.5708, 0.7854);
  Matrix3f DCM = euler2dcm(v);
  std::cout << DCM << std::endl;
}
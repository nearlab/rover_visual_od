//when you need a function, just find that function and include a header file
//also need to get eigen in your cmake file
//getting th, v values straight from IMU
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

//Global Variables
using Eigen::Vector3f;
Vector3f g(0,0,-9.81); //gravity


//Cross Product Equivalent
using Eigen::Matrix3f;
using Eigen::Vector3f;
Matrix3f crossProductEquivalent(Vector3f v)
{
  Matrix3f c;
  c << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  //std::cout << c << std::endl;
  return c;
}

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
  //std::cout << qReturn.w() << std::endl << qReturn.vec() << std::endl;
  return qReturn;
}

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

//Propagation

//Execute each time the IMU is sampled
Vector3f am; //accelerometer measurement (get from IMU)
Vector3f wm; //gyroscope measurement (get from IMU)
float dt; //(get from ROS) 

//Build Omega(w)
wmx = crossProductEquivalent(wm);
Matrix 4f Omega;
Omega << -wmx(0,0), -wmx(0,1), -wmx(0,2), wm(0),
	 -wmx(1,0), -wmx(1,1), -wmx(1,2), wm(1),
         -wmx(2,0), -wmx(2,1), -wmx(2,2), wm(2),
         -wm(0), -wm(1), -wm(2), 0;

//Measurements at time l-1
Vector3f amOld; //accelerometer measurement from last time
Vector3f wmOld; //gyroscope measurement from last time 


//Propagate state estimate

// Constants
using Eigen::Quaternionf;
using Eigen::Vector3f; 
using Eigen::Matrix3f
Matrix3f I3 = Matrix3f::Identity(3,3); 
Matrix3f O3 = Matrix3f::Zero();
float g = 9.81; 

//Propagate quaternion
using Eigen::Quaternionf;
using Eigen::Vector3f; 
using Eigen::Matrix3f
Quaternionf qHat;
Matrix3f RHat = qHat.toRotationMatrix();
Matrix3f RHatProp = (I - dt * wmx) * RHat; 
Quaternionf qHatExp = qExponential(wmOld, dt);
Quaternionf qHatProp = qMultiply(qHatExp, qHatExp);

//Propagate p, the position
Vector3f pHatProp = pHat + vHat*dt + RHat*RHatProp*(amOld - baHat)*dt^2 + 0.5*g*(dt^2)

//Propagate v, the velocity
Vector3f vHatProp = vHat + RHat*RHatProp*(amOld - baHat)*dt + g*dt; 

//Propagate bg and ba, the gyroscope and accelerometer biases
Vector3f bgHatProp = bgHat;
Vector3f baHatProp = baHat;

//Calculate IMU error state transition matrix
using Eigen::MatrixXf;
phipq = -crossProductEquivalent(pHatProp - pHat - vHat*dt - 0.5*g*dt^2);
phivq = -crossProductEquivalent(vHatProp - vHat - g*dt);
phigbg = RHat.transpose() * RHatProp * dt; 
phipbg = crossProductEquivalent(vHat - g*dt) * RHat.transpose() * RHatProp * dt;
phipa = RHat.transpose() * RHatProp * dt^2; 
phivbg = crossProductEquivalent(vHat - g*dt) * RHat.transpose() * RHatProp * dt; 
phiva = RHat.transpose() * RHatProp * dt; 
MatrixXf PhiProp(15, 15);
PhiProp << I3,    O3,      O3, phiqbg,    O3, 
	         phipq, I3, (dt*I3), phipbg, phipa, 
           phivq, O3,      I3, phivbg, phiva, 
	         O3,    O3,      O3,     I3,    O3, 
           O3,    O3,      O3,     O3,    I3;
}




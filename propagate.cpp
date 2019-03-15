#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <unsupported/Eigen/MatrixFunctions>

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

//Quaternion to DCM
//ref: mathworks Quat2DCM documentation
using Eigen::Quaternionf;
using Eigen::Matrix3f;
Matrix3f q2dcm(Quaternionf q)
{
  float q0 = q.w();
  float q1 = q.x(); 
  float q2 = q.y();
  float q3 = q.z();
  Matrix3f DCM;
  DCM << (q0*q0 + q1*q1 - q2*q2 - q3*q3), 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2), 
          2*(q1*q2 - q0*q3), (q0*q0 - q1*q1 + q2*q2 - q3*q3), 2*(q2*q3 + q0*q1), 
          2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), (q0*q0 - q1*q1 - q2*q2 + q3*q3);
  return DCM; 
}

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


//Estimate class
using Eigen::Quaternionf;
using Eigen::Vector3f;
class StateEstimate
{
  private:
    Quaternionf qHat;
    Vector3f pHat;
    Vector3f vHat;
    /*Vector3f dThetaHat;*/
    Vector3f bgHat;
    Vector3f baHat;
    float tk;
  
  public:
    StateEstimate(Quaternionf q, Vector3f p, Vector3f v, /*Vector3f dTheta,*/ Vector3f bg, Vector3f ba, float tk) {
      qHat = q;
      pHat = p; 
      vHat = v;
      /*dThetaHat = dTheta;*/
      bgHat = bg;
      baHat = ba;
    }

    Quaternionf getqHat(){return qHat;}
    void setqHat(Quaternionf q){this->qHat = q;}
    /*Vector3f getdThetaHat(){return dThetaHat;}
    void setdThetaHat(Vector3f dTheta){this->dThetaHat = dThetaHat;}*/
    Vector3f getpHat(){return pHat;}
    void setpHat(Vector3f p){this->pHat = p;}
    Vector3f getvHat(){return vHat;}
    void setvHat(Vector3f v){this->vHat = v;}
    Vector3f getbgHat(){return bgHat;}
    void setbgHat(Vector3f bg){this->bgHat = bg;}
    Vector3f getbaHat(){return baHat;}
    void setbaHat(Vector3f ba){this->baHat = ba;}
    void gettk(){return tk;}
    void settk(float t){this->tk = t;}

};

//Estimator state class
using Eigen::Vector3f;
using Eigen::Matrix3f; 
class EstimatorState
{
private: 
	Quaternionf qBar;
	Vector3f pBar;
	Vector3f vBar; 
	Vector3f e; 
	Vector3f bgBar; 
	Vector3f baBar; 

	Matrix3f Pp;
	Matrix3f Pv;
	Matrix3f Pe;
	Matrix3f Pba;
	Matrix3f Pbg;

	float tk;

public:
	EstimatorState(Quaternionf q, Vector3f p, Vector3f v, Vector3f ee, Vector3f bg, 
		Vector3f ba, Matrix3f pp, Matrix3f pv, Matrix3f pe, Matrix3f pba, Matrix3f pbg, float tk) {
        qBar = q;
        pBar = p;
        vBar = v;
        e = ee; 
        bgBar = bg; 
        baBar = ba; 
        Pe = pe; 
        Pp = pp;
        Pv = pv; 
        Pba = pba; 
        Pbg = pbg; 
	}

	Quaternionf getqBar(){return qBar;}
    void setqBar(Quaternionf q){this->qBar = q;}
    /*Vector3f getdThetaHat(){return dThetaHat;}
    void setdThetaHat(Vector3f dTheta){this->dThetaHat = dThetaHat;}*/
    Vector3f getpBar(){return pBar;}
    void setpBar(Vector3f p){this->pBar = p;}
    Vector3f getvBar(){return vBar;}
    void sete(Vector3f ee){this->e = ee;}
    Vector3f gete(){return e;}
    void setvBar(Vector3f v){this->vBar = v;}
    Vector3f getbgBar(){return bgBar;}
    void setbgBar(Vector3f bg){this->bgBar = bg;}
    Vector3f getbaBar(){return baBar;}
    void setbaBar(Vector3f ba){this->baBar = ba;}
    void gettk(){return tk;}
    void settk(float t){this->tk = t;}
}

//Measurement class
using Eigen::Vector3f; 
using Eigen::Matrix3f; 
class Measurement
{
private: 
	Vector3f wm;
	Vector3f am;

public: 
	Measurement(Vector3f a, Vector3f w) {
        am = a;
        wm = w; 
	}

	Vector3f getwm(){return wm;}
    Vector3f getam(){return am;}

}

//Error state class
/*using Eigen::Quaternionf;
using Eigen::Vector3f;
class ErrorState
{
  private: 
    Quaternionf qTilde;
    Vector3f pTilde;
    Vector3f vTilde;
    Vector3f bgTilde;
    Vector3f baTilde; 

  public: 
    ErrorState(Quaternionf qT, Vector3f pT, Vector3f vT, Vector3f bgT, Vector3f baT) {
      qTilde = qT;
      pTilde = pT; 

    }
}*/

//State Estimation (prop, update)
//EKF-based estimation 
//ref: "Aerial Robotics," 'Laboratory Exercise 3: Meas. Simulation and State Estimation,' 2019
void prop(StateEstimate S, EstimatorState E)
{
    
}

void update(StateEstimate S, EstimatorState E, Measurements M)
{
    //Convert measurements to I-frame
    Vector3f wm = M.getwm();
    Vector3f am = M.getam();
    Matrix3f RBIBark = q2dcm(E.getqBar());
    Vector3f wmI = (RBIBark.transpose())*wm;
    Vector3f amI = (RBIBark.transpose())*am; 

    //Push estimator state through measurement function 
    //(relationship bewteen measurements and dynamics)
    //do stuff with image features
    
    
}

//Propagate State Estimate
/*StateEstimate propState(StateEstimate S, Vector3f am, Vector3f wm, float dt)
{
  // Constants
  using Eigen::Quaternionf;
  using Eigen::Vector3f; 
  using Eigen::Matrix3f;
  Matrix3f I3 = Matrix3f::Identity(3,3); 
  Matrix3f O3 = Matrix3f::Zero();
  Matrix3f wmx = crossProductEquivalent(wm);

  //State values
  Quaternionf qHat = S.getqHat();
  Vector3f dThetaHat = S.getdThetaHat();
  Vector3f pHat = S.getpHat();
  Vector3f vHat = S.getvHat();
  Vector3f bgHat = S.getbgHat();
  Vector3f baHat = S.getbaHat();

  //Propagate quaternion
  using Eigen::Quaternionf;
  using Eigen::Vector3f; 
  using Eigen::Matrix3f;
  Quaternionf qExp = qExponential(dt, wm);
  Quaternionf qHatProp = qMultiply(qHat, qExp);

  //Calculate TIB, the DCM
  Matrix3f TIB = q2dcm(qHat);
  Matrix3f TBI = TIB.transpose();

  //Propagate p, the position
  Vector3f pHatProp = pHat 
                      + vHat*dt  
                      + 0.5*(TBI*crossProductEquivalent(am*(dt*dt)))*dThetaHat
                      - 0.5*TIB*baHat; 

  //Propagate v, the velocity
  Vector3f vHatProp = vHat 
  					  + (TBI*crossProductEquivalent((am-baHat)*dt))*dThetaHat
                      - TIB*dt*baHat;

  //Propagate dThetaHat, the 3x1 vector of angles of attitude error
  Vector3f dThetaHatProp = ((-crossProductEquivalent(wm)*dt).exp())*dThetaHat 
                  - bgHat;

  //Propagate bg and ba, the gyroscope and accelerometer biases
  Vector3f bgHatProp = bgHat;
  Vector3f baHatProp = baHat;

  //Return the propagated state
  StateEstimate Sprop(qHatProp, pHatProp, vHatProp, dThetaHatProp, bgHatProp, baHatProp);
  return Sprop;
}*/

//State Estimator
//EKF-based estimation 
//ref: "Aerial Robotics," 'Laboratory Exercise 3: Meas. Simulation and State Estimation,' 2019
StateEstimate stateEstimatorEKF(float tk, Vector3f am, Vector3f wm, )

int main()
{
  /*using Eigen::Vector3f; 
  using Eigen::Matrix3f;
  using Eigen::Quaternionf;
  Matrix3f A;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Vector3f b; 
  b << 1, 2, 3;
  Vector3f c;
  c << 4, 5, 6;
  std::cout << A*b << std::endl;
  std::cout << 0.5*A*b <<std::endl;
  float dt = 0.1;
  std::cout << 0.5*b*dt*dt <<std::endl;*/

  /*using Eigen::Vector3f; 
  using Eigen::Matrix3f;
  using Eigen::Quaternionf;
  Vector3f v(0.0, 3.1415, 6.2830);
  Matrix3f DCM = euler2dcm(v);
  std::cout << DCM << std::endl;*/

  Quaternionf q;
  Vector3f qv(0,0,0);
  float qw = 1;
  q.w() = qw;
  q.vec() = qv;
  Vector3f dTheta(0,0,0);
  Vector3f p(0,0,0);
  Vector3f v(0,0,0);
  Vector3f bg(0,0,0);
  Vector3f ba(0,0,0);
  StateEstimate S(q, p, v, dTheta, bg, ba);

  Vector3f am(0.2,0,-9.81);
  Vector3f wm(0.5*3.1415,0,0);
  float dt = 0.1;


  for ( int ii = 0; ii < 10; ii = ii + 1 ){
    StateEstimate Sprop = propState(S,am,wm,dt);
    std::cout << "qHatProp (w): " << Sprop.getqHat().w() << std::endl;
    std::cout << "qHatProp (v): " << std::endl << Sprop.getqHat().vec() << std::endl;
    std::cout << "dThetaHatProp: " << std::endl << Sprop.getdThetaHat() << std::endl;
    std::cout << "pHatProp: " << std::endl << Sprop.getpHat() << std::endl;
    std::cout << "vHatProp: " << std::endl << Sprop.getvHat() << std::endl;
    std::cout << "baHatProp: " << std::endl << Sprop.getbaHat() << std::endl;
    std::cout << "bgHatProp: " << std::endl << Sprop.getbgHat() << std::endl;
    std::cout << std::endl;
    S = Sprop;
  }


}

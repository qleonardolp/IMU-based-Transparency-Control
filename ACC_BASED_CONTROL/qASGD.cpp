//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
#include <string>

/*  -- Obtain attitude quaternion:
-- Quaternion-based Attitude estimation using ASGD algorithm
-- Refs:
-- [1]: Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm,
--      Wang, Li and Zhang, Zheng and Sun, Ping, International Journal of Advanced Robotic Systems, 2015
-- [2]: Estimation of IMU and MARG orientation using a gradient descent algorithm,
--      Madgwick, Sebastian OH and Harrison, Andrew JL and Vaidyanathan, Ravi, international Conference on Rehabilitation Robotics, 2011
-- [3]: "How to integrate Quaternions", Ashwin Narayan (www.ashwinnarayan.com/post/how-to-integrate-quaternions/)
*/

void removeYaw(Eigen::Vector4f* quat);
Eigen::Vector3f quatDelta2Euler(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m);
Eigen::Vector3f RelOmegaNED(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m, \
                            const Eigen::Vector3f* omg_r, const Eigen::Vector3f* omg_m);

void qASGD(ThrdStruct &data_struct)
{
  using namespace std;
  using namespace Eigen;

#if PRIORITY
  SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

  // Declarations
  float q0 = 0;
  float q1 = 0;
  float q2 = 0;
  float q3 = 0;
  float imus_data[18];
  float states_data[10];
  // Inicialização por segurança:
  for (int i = 0; i < 10; i++) states_data[i] = 0;
  float euler_k[3] = {0,0,0};
  float omega_k[3] = {0,0,0};
  const float Ts = data_struct.sampletime_;
  const float mi0 = 0.100;
  const float Beta = 1.1400;
  const float Rho = 0.00017;
  const Matrix4f H = Matrix4f::Identity();
  const Matrix4f R = Matrix4f::Identity() * 1e-5;

  Vector4f qASGD1_qk(1, 0, 0, 0);
  Vector4f qASGD2_qk(1, 0, 0, 0);
  Matrix4f qASGD1_Pk = Matrix4f::Identity();
  Matrix4f qASGD2_Pk = Matrix4f::Identity();
  FullPivLU<Matrix4f> Covar1;
  FullPivLU<Matrix4f> Covar2;
  Matrix4f Q1 = Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
  Matrix4f Q2 = Q1;

  Vector3f gyro1(0, 0, 0);
  Vector3f gyro2(0, 0, 0);
  Vector3f acc1(0, 0, 0);
  Vector3f acc2(0, 0, 0);
  Vector3f F_obj;
  Vector4f GradF;
  Vector4f z_k;
  Vector3f Zc;
  Matrix4f OmG;
  Matrix4f Psi;
  Matrix4f Kg1;
  Matrix4f Kg2;
  Matrix4f Qy;
  Matrix<float, 3, 4> Jq;
  Matrix<float, 4, 3> Xi;
  float omg_norm = gyro1.norm();
  float mi(0);

  bool isready_imu(false);
  do{ 
    {   // qASGD confere IMU:
      unique_lock<mutex> _(*data_struct.mtx_);
      isready_imu = *data_struct.param0A_;
    } 
  } while (!isready_imu);

  {   // qASGD avisa que esta pronto!
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0B_ = true;
  }

  looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
  // inicializa looptimer
  Timer.start();
  do
  {
    Timer.tik();
    { // sessao critica: minimo codigo necessario para pegar datavec_
      unique_lock<mutex> _(*data_struct.mtx_);
      memcpy(imus_data, *data_struct.datavec_, sizeof(imus_data));
    } // fim da sessao critica

    gyro1 << imus_data[0], imus_data[1], imus_data[2];
    acc1 << imus_data[3], imus_data[4], imus_data[5];
    gyro2 << imus_data[6], imus_data[7], imus_data[8];
    acc2 << imus_data[9], imus_data[10], imus_data[11];

    q0 = qASGD1_qk(0);
    q1 = qASGD1_qk(1);
    q2 = qASGD1_qk(2);
    q3 = qASGD1_qk(3);

    Zc << 2 * (q1 * q3 - q0 * q2),
      2 * (q2 * q3 + q0 * q1),
      (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
    F_obj = Zc - acc1.normalized(); // Eq.23

    Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
      2 * q1, 2 * q0, 2 * q3, 2 * q2,
      2 * q0, -2 * q1, -2 * q2, 2 * q3;

    GradF = Jq.transpose() * F_obj; // Eq.25

    omg_norm = gyro1.norm();
    mi = mi0 + Beta * Ts * omg_norm; // Eq.29

    z_k = qASGD1_qk - mi * GradF.normalized(); // Eq.24
    z_k.normalize();

    OmG << 0, -gyro1(0), -gyro1(1), -gyro1(2),
      gyro1(0), 0, gyro1(2), -gyro1(1),
      gyro1(1), -gyro1(2), 0, gyro1(0),
      gyro1(2), gyro1(1), -gyro1(0), 0;
    OmG = 0.5 * OmG;

    Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

    // Process noise covariance update (Eq. 19):
    Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
    Q1 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
    // Projection:
    qASGD1_qk = Psi * qASGD1_qk;
    qASGD1_Pk = Psi * qASGD1_Pk * Psi.transpose() + Q1;
    // Kalman Gain (H is Identity)
    Covar1 = FullPivLU<Matrix4f>(qASGD1_Pk + R);
    if (Covar1.isInvertible())
      Kg1 = qASGD1_Pk * Covar1.inverse();
    // Update (H is Identity)
    qASGD1_qk = qASGD1_qk + Kg1 * (z_k - qASGD1_qk);
    qASGD1_Pk = (Matrix4f::Identity() - Kg1) * qASGD1_Pk;
    qASGD1_qk.normalize();

    // Rotate the quaternion by a quaternion with -(yaw):
    removeYaw(&qASGD1_qk);


    q0 = qASGD2_qk(0);
    q1 = qASGD2_qk(1);
    q2 = qASGD2_qk(2);
    q3 = qASGD2_qk(3);

    Zc << 2 * (q1 * q3 - q0 * q2),
      2 * (q2 * q3 + q0 * q1),
      (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
    F_obj = Zc - acc2.normalized(); // Eq.23

    Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
      2 * q1, 2 * q0, 2 * q3, 2 * q2,
      2 * q0, -2 * q1, -2 * q2, 2 * q3;

    GradF = Jq.transpose() * F_obj; // Eq.25

    omg_norm = gyro2.norm();
    mi = mi0 + Beta * Ts * omg_norm; // Eq.29

    z_k = qASGD2_qk - mi * GradF.normalized(); // Eq.24
    z_k.normalize();

    OmG << 0, -gyro2(0), -gyro2(1), -gyro2(2),
      gyro2(0), 0, gyro2(2), -gyro2(1),
      gyro2(1), -gyro2(2), 0, gyro1(0),
      gyro2(2), gyro2(1), -gyro2(0), 0;
    OmG = 0.5 * OmG;

    Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

    // Process noise covariance update (Eq. 19):
    Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
    Q2 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
    // Projection:
    qASGD2_qk = Psi * qASGD2_qk;
    qASGD2_Pk = Psi * qASGD2_Pk * Psi.transpose() + Q2;
    // Kalman Gain (H is Identity)
    Covar2 = FullPivLU<Matrix4f>(qASGD2_Pk + R);
    if (Covar2.isInvertible())
      Kg2 = qASGD2_Pk * Covar2.inverse();
    // Update (H is Identity)
    qASGD2_qk = qASGD2_qk + Kg2 * (z_k - qASGD2_qk);
    qASGD2_Pk = (Matrix4f::Identity() - Kg2) * qASGD2_Pk;
    qASGD2_qk.normalize();

    // Rotate the quaternion by a quaternion with -(yaw):
    removeYaw(&qASGD2_qk);

    // Euler angles between IMUs:
    Vector3f euler_ned = quatDelta2Euler(&qASGD2_qk, &qASGD1_qk);
    // Relative Omega between IMUs:
    Vector3f omega_ned = RelOmegaNED(&qASGD1_qk, &qASGD2_qk, &gyro1, &gyro2);

    /*
    // por enquanto so para o joelho direito:
    hum_rgtknee_pos = vector[0]; v
    hum_rgtknee_vel = vector[1]; v
    hum_rgtknee_acc = vector[2]; v
    rbt_rgtknee_pos = vector[3]; x
    rbt_rgtknee_vel = vector[4]; x
    rbt_rgtknee_acc = vector[5]; x
    sea_rgtshank    = vector[6]; x
    inter_rgtshank  = vector[7]; x
    mtr_rgtknee_tau = vector[8]; x
    mtr_rgtknee_omg = vector[9]; x
    */

    // Finite Difference Acc approximation:

    euler_k[0] = euler_ned(0);
    float acc_euler = (euler_k[0] - 2*euler_k[1] + euler_k[2])/(Ts*Ts);
    euler_k[2] = euler_k[1]; 
    euler_k[1] = euler_k[0];


    omega_k[0] = omega_ned(0);
    float acc_omega = (3*omega_k[0] - 4*omega_k[1] + omega_k[2])/(2*Ts);
    omega_k[2] = omega_k[1]; 
    omega_k[1] = omega_k[0]; 

    states_data[0] = euler_ned(0);  // hum_rgtknee_pos
    states_data[1] = omega_ned(0);  // hum_rgtknee_vel
    states_data[2] = (acc_euler + acc_omega)/2; // hum_rgtknee_acc

    { // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      switch (data_struct.param39_)
      {
      case IMUBYPASS:
        *(*data_struct.datavecB_ + 0) = euler_ned(0);              // hum_rgtknee_pos
        *(*data_struct.datavecB_ + 2) = (acc_euler + acc_omega)/2; // hum_rgtknee_acc
        break;
      case READIMUS:
        *(*data_struct.datavecA_ + 0) = euler_ned(0);              // hum_rgtknee_pos
        *(*data_struct.datavecA_ + 1) = omega_ned(0);              // hum_rgtknee_vel
        *(*data_struct.datavecA_ + 2) = (acc_euler + acc_omega)/2; // hum_rgtknee_acc
      default:
        *(*data_struct.datavecB_ + 0) = euler_ned(0);              // hum_rgtknee_pos
        *(*data_struct.datavecB_ + 1) = omega_ned(0);              // hum_rgtknee_vel
        *(*data_struct.datavecB_ + 2) = (acc_euler + acc_omega)/2; // hum_rgtknee_acc
        break;
      }
    } // fim da sessao critica

    Timer.tak();
  } while (!Timer.end());

  {   
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0B_ = false;
  }
}

void removeYaw(Eigen::Vector4f* quat)
{
  float q0 = (*quat)(0);
  float q1 = (*quat)(1);
  float q2 = (*quat)(2);
  float q3 = (*quat)(3);
  float yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3, \
    q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
  Eigen::Matrix4f  Qy = Eigen::Matrix4f::Identity() * cosf(-yaw / 2);
  Qy(0, 3) = -sinf(-yaw / 2);
  Qy(1, 2) = Qy(0, 3);
  Qy(2, 1) = -Qy(0, 3);
  Qy(3, 0) = -Qy(0, 3);
  *quat = Qy * (*quat);
  quat->normalize();
}

Eigen::Vector3f quatDelta2Euler(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m)
{
  float qr0 = (*quat_r)(0);
  float qr1 = (*quat_r)(1);
  float qr2 = (*quat_r)(2);
  float qr3 = (*quat_r)(3);
  // q_m conjugate (*q_m):
  float qm0 =  (*quat_m)(0);
  float qm1 = -(*quat_m)(1);
  float qm2 = -(*quat_m)(2);
  float qm3 = -(*quat_m)(3);

  Eigen::Vector4f q;
  // quaternion product: q_r x *q_m:
  q(0) = qr0*qm0 - qr1*qm1 - qr2*qm2 - qr3*qm3;
  q(1) = qr0*qm1 + qr1*qm0 + qr2*qm3 - qr3*qm2;
  q(2) = qr0*qm2 - qr1*qm3 + qr2*qm0 + qr3*qm1;
  q(3) = qr0*qm3 + qr1*qm2 - qr2*qm1 + qr3*qm0;

  Eigen::Vector3f euler;
  euler(0) = atan2f(2*q(2)*q(3) + 2*q(0)*q(1), q(3)*q(3) - q(2)*q(2) - q(1)*q(1) + q(0)*q(0));
  euler(1) = -asinf(2*q(1)*q(3) - 2*q(0)*q(2));
  euler(2) = atan2f(2*q(1)*q(2) + 2*q(0)*q(3), q(1)*q(1) + q(0)*q(0) - q(3)*q(3) - q(2)*q(2));
  return euler;
}

Eigen::Vector3f RelOmegaNED(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m, \
                            const Eigen::Vector3f* omg_r, const Eigen::Vector3f* omg_m)
{
  using namespace Eigen;
  float q0 = (*quat_r)(0);
  float q1 = (*quat_r)(1);
  float q2 = (*quat_r)(2);
  float q3 = (*quat_r)(3);

  Matrix3f Rot = Matrix3f::Identity();

  Rot(0,0) = (q0*q0 + q1*q1 - q2*q2 - q3*q3);
  Rot(0,1) = 2*(q1*q2 - q0*q3);
  Rot(0,2) = 2*(q1*q3 + q0*q2);
  Rot(1,0) = 2*(q1*q2 + q0*q3);
  Rot(1,1) = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
  Rot(1,2) = 2*(q2*q3 - q0*q1);
  Rot(2,0) = 2*(q1*q3 - q0*q2);
  Rot(2,1) = 2*(q2*q3 + q0*q1);
  Rot(2,2) = (q0*q0 - q1*q1 - q2*q2 - q3*q3); 

  Vector3f Omega1 = Rot*(*omg_r);

  q0 = (*quat_m)(0);
  q1 = (*quat_m)(1);
  q2 = (*quat_m)(2);
  q3 = (*quat_m)(3);

  Rot(0,0) = (q0*q0 + q1*q1 - q2*q2 - q3*q3);
  Rot(0,1) = 2*(q1*q2 - q0*q3);
  Rot(0,2) = 2*(q1*q3 + q0*q2);
  Rot(1,0) = 2*(q1*q2 + q0*q3);
  Rot(1,1) = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
  Rot(1,2) = 2*(q2*q3 - q0*q1);
  Rot(2,0) = 2*(q1*q3 - q0*q2);
  Rot(2,1) = 2*(q2*q3 + q0*q1);
  Rot(2,2) = (q0*q0 - q1*q1 - q2*q2 - q3*q3); 

  Vector3f Omega2 = Rot*(*omg_m);

  return Omega2 - Omega1;
}


//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|
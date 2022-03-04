//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
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
Eigen::Vector4f qDelta(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m);
Eigen::Vector3f quatDelta2Euler(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m);
Eigen::Vector3f RelOmegaNED(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m, const Eigen::Vector3f* omg_r, const Eigen::Vector3f* omg_m);
Eigen::Vector3f RelVector(const Eigen::Vector4f rel_quat, const Eigen::Vector3f vec_r, const Eigen::Vector3f vec_m);
Eigen::Vector3f RelAngAcc(const Eigen::Vector4f rel_quat, const Eigen::Vector3f rel_ang_vel, const Eigen::Vector3f rel_linear_acc);

extern void rollBuffer(float buffer[10], const size_t length);


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
  float imus_data[DTVC_SZ];
  for (int i = 0; i < DTVC_SZ; i++) imus_data[i] = 0;
  float rad2deg = 180 / (M_PI);
  float euler_k[3] = {0,0,0};
  float omega_k[3] = {0,0,0};
  const float Ts = data_struct.sampletime_;
  const float mi0 = 0.36;
  const float Beta = 10.0;
  const Matrix4f H = Matrix4f::Identity();
  const Matrix4f R = Matrix4f::Identity() * 2.5e-5;

  Vector4f qASGD1_qk(1, 0, 0, 0);
  Vector4f qASGD2_qk(1, 0, 0, 0);
  Vector4f qASGD3_qk(1, 0, 0, 0);
  Quaternionf q12Off(0, 0, 0, 0);
  Quaternionf q23Off(0, 0, 0, 0);
  Matrix4f qASGD1_Pk = Matrix4f::Identity();
  Matrix4f qASGD2_Pk = Matrix4f::Identity();
  Matrix4f qASGD3_Pk = Matrix4f::Identity();
  FullPivLU<Matrix4f> Covar1;
  FullPivLU<Matrix4f> Covar2;
  FullPivLU<Matrix4f> Covar3;
  Matrix4f Q1 = Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
  Matrix4f Q2 = Q1;
  Matrix4f Q3 = Q1;

  Vector3f gyro1(0, 0, 0);
  Vector3f gyro2(0, 0, 0);
  Vector3f gyro3(0, 0, 0);
  Vector3f acc1(0, 0, 0);
  Vector3f acc2(0, 0, 0);
  Vector3f acc3(0, 0, 0);
  Vector3f F_obj;
  Vector4f GradF;
  Vector4f z_k;
  Vector3f Zc;
  Matrix4f OmG;
  Matrix4f Psi;
  Matrix4f Kg1;
  Matrix4f Kg2;
  Matrix4f Kg3;
  Matrix4f Qy;
  Matrix<float, 3, 4> Jq;
  Matrix<float, 4, 3> Xi;
  float omg_norm = gyro1.norm();
  float mi(0);

  bool isready_imu(false);
  bool aborting_imu(false);
  bool asgd_abort(false);
  do{ 
    {   // qASGD confere IMU:
      unique_lock<mutex> _(*data_struct.mtx_);
      isready_imu = *data_struct.param0A_;
      aborting_imu = *data_struct.param1A_;
      if (aborting_imu)
      {
          asgd_abort = *data_struct.param1B_ = true;
          break;
      }
    } 
  } while (!isready_imu);

  if (asgd_abort) return;

  {   // qASGD avisa que esta pronto!
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0B_ = true;
  }

  looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
  auto t_begin = Timer.micro_now();
  // inicializa looptimer
  Timer.start();
  do
  {
    Timer.tik();
    { // sessao critica:
      unique_lock<mutex> _(*data_struct.mtx_);
      memcpy(imus_data, *data_struct.datavec_, sizeof(imus_data));
    } // fim da sessao critica

    gyro1 << imus_data[0], imus_data[1], imus_data[2];
    acc1 << imus_data[3], imus_data[4], imus_data[5];
    gyro2 << imus_data[6], imus_data[7], imus_data[8];
    acc2 << imus_data[9], imus_data[10], imus_data[11];
    gyro3 << imus_data[12], imus_data[13], imus_data[14];
    acc3 << imus_data[15], imus_data[16], imus_data[17];

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


    // Foot:
    q0 = qASGD3_qk(0);
    q1 = qASGD3_qk(1);
    q2 = qASGD3_qk(2);
    q3 = qASGD3_qk(3);

    Zc << 2 * (q1 * q3 - q0 * q2),
        2 * (q2 * q3 + q0 * q1),
        (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
    F_obj = Zc - acc3.normalized(); // Eq.23

    Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
        2 * q1, 2 * q0, 2 * q3, 2 * q2,
        2 * q0, -2 * q1, -2 * q2, 2 * q3;

    GradF = Jq.transpose() * F_obj; // Eq.25

    omg_norm = gyro3.norm();
    mi = mi0 + Beta * Ts * omg_norm; // Eq.29

    z_k = qASGD3_qk - mi * GradF.normalized(); // Eq.24
    z_k.normalize();

    OmG << 0, -gyro3(0), -gyro3(1), -gyro3(2),
        gyro3(0), 0, gyro3(2), -gyro3(1),
        gyro3(1), -gyro3(2), 0, gyro3(0),
        gyro3(2), gyro3(1), -gyro3(0), 0;
    OmG = 0.5 * OmG;

    Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

    // Process noise covariance update (Eq. 19):
    Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
    Q3 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
    // Projection:
    qASGD3_qk = Psi * qASGD3_qk;
    qASGD3_Pk = Psi * qASGD3_Pk * Psi.transpose() + Q3;
    // Kalman Gain (H is Identity)
    Covar3 = FullPivLU<Matrix4f>(qASGD3_Pk + R);
    if (Covar3.isInvertible())
        Kg3 = qASGD3_Pk * Covar3.inverse();
    // Update (H is Identity)
    qASGD3_qk = qASGD3_qk + Kg3 * (z_k - qASGD3_qk);
    qASGD3_Pk = (Matrix4f::Identity() - Kg3) * qASGD3_Pk;
    qASGD3_qk.normalize();

    // Rotate the quaternion by a quaternion with -(yaw):
    removeYaw(&qASGD3_qk);

    // Euler angles between IMUs:
    Vector3f knee_euler = quatDelta2Euler(qASGD2_qk, qASGD1_qk);
    Vector3f ankle_euler = quatDelta2Euler(qASGD3_qk, qASGD2_qk);


    // Remove arbitrary IMU attitude:
    if (Timer.micro_now() - t_begin <  1*MILLION) // 1 segundos
    {
        float incrmnt = data_struct.sampletime_ / 1;
        q12Off.w() += (data_struct.sampletime_/2) * qDelta(qASGD2_qk, qASGD1_qk)(0);
        q12Off.x() += (data_struct.sampletime_/2) * qDelta(qASGD2_qk, qASGD1_qk)(1);
        q12Off.y() += (data_struct.sampletime_/2) * qDelta(qASGD2_qk, qASGD1_qk)(2);
        q12Off.z() += (data_struct.sampletime_/2) * qDelta(qASGD2_qk, qASGD1_qk)(3);

        q23Off.w() += (data_struct.sampletime_/2) * qDelta(qASGD3_qk, qASGD2_qk)(0);
        q23Off.x() += (data_struct.sampletime_/2) * qDelta(qASGD3_qk, qASGD2_qk)(1);
        q23Off.y() += (data_struct.sampletime_/2) * qDelta(qASGD3_qk, qASGD2_qk)(2);
        q23Off.z() += (data_struct.sampletime_/2) * qDelta(qASGD3_qk, qASGD2_qk)(3);
    }
    else
    {
        q12Off.normalize();
        q23Off.normalize();
        Vector4f q12(q12Off.w(), q12Off.x(), q12Off.y(), q12Off.z());
        Vector4f q23(q23Off.w(), q23Off.x(), q23Off.y(), q23Off.z());
        knee_euler  = quatDelta2Euler(qDelta(qASGD2_qk,q12), qASGD1_qk);
        ankle_euler = quatDelta2Euler(qDelta(qASGD3_qk,q23), qASGD2_qk);
    }

    // Relative Angular Velocity
    Vector3f knee_omega  = RelVector(qDelta(qASGD1_qk, qASGD2_qk), gyro1, gyro2);
    Vector3f ankle_omega = RelVector(qDelta(qASGD2_qk, qASGD3_qk), gyro2, gyro3);

    // Relative Acc (Linear) in IMU1 frame:
    Vector3f left_shank2thigh_acc = RelVector(qDelta(qASGD1_qk, qASGD2_qk), acc1, acc2);
    Vector3f left_foot2shank_acc = RelVector(qDelta(qASGD2_qk, qASGD3_qk), acc2, acc3);

    // IMU2 Relative Acc (Angular) in IMU2 frame:
    Vector3f left_shank2thigh_alpha = RelAngAcc( qDelta(qASGD2_qk, qASGD1_qk),\
                                                 knee_omega, left_shank2thigh_acc);

    Vector3f s2tL_alpha = Quaternionf(qASGD1_qk).toRotationMatrix() * left_shank2thigh_alpha;

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

    euler_k[0] = knee_euler(0);
    float acc_euler = (euler_k[0] - 2*euler_k[1] + euler_k[2])/(Ts*Ts);
    rollBuffer(euler_k, 3);


    omega_k[0] = knee_omega(0);
    float acc_omega = (3*omega_k[0] - 4*omega_k[1] + omega_k[2])/(2*Ts);
    omega_k[2] = omega_k[1]; 
    omega_k[1] = omega_k[0]; 

    { // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      switch (data_struct.param39_)
      {
      case IMUBYPASS:
        *(*data_struct.datavecB_ + 0) = knee_euler(0);              // hum_rgtknee_pos
        *(*data_struct.datavecB_ + 2) = (1.4*acc_euler + 0.6*acc_omega)/2; // hum_rgtknee_acc
        break;
      case READIMUS:
        *(*data_struct.datavecA_ + 0) = knee_euler(0);              // hum_rgtknee_pos
        *(*data_struct.datavecA_ + 1) = knee_omega(0);   // hum_rgtknee_vel
        *(*data_struct.datavecA_ + 2) = acc_omega;
        *(*data_struct.datavecA_ + 3) = ankle_euler(0); // esta "negativo" ->TODO
        *(*data_struct.datavecA_ + 4) = ankle_omega(0);
        break;
      default:
        *(*data_struct.datavecB_ + 0) = knee_euler(0);   // hum_rgtknee_pos
        *(*data_struct.datavecB_ + 1) = knee_omega(0);   // hum_rgtknee_vel
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

Eigen::Vector3f RelAngAcc(const Eigen::Vector4f rel_quat, const Eigen::Vector3f rel_ang_vel, const Eigen::Vector3f rel_linear_acc)
{
    using namespace Eigen;
    Matrix3f Rot = Quaternionf(rel_quat).toRotationMatrix();

    Vector3f linAccFrame2 = Rot.transpose() * rel_linear_acc;
    Vector3f angVelFrame2 = Rot.transpose() * rel_ang_vel;

    float omg_x = angVelFrame2(0);
    float omg_y = angVelFrame2(1);
    float omg_z = angVelFrame2(2);
    float acc_x = linAccFrame2(0);
    float acc_y = linAccFrame2(1);
    float acc_z = linAccFrame2(2);
    float alpha_x, alpha_y, alpha_z;

    // using centriptal and radial acc decompositon:
    float norm_zy = sqrt(acc_z * acc_z + acc_y * acc_y);
    float phi = atan2f(-acc_y, acc_z); // -y devido a orientacao adotada das imus nas pernas
    alpha_x = norm_zy * sinf(phi); // aproximando R=1

    float norm_xz = sqrt(acc_x * acc_x + acc_z * acc_z);
    phi = atan2f(acc_x, -acc_z);
    alpha_y = norm_xz * sinf(phi); // aproximando R=1

    float norm_xy = sqrt(acc_x * acc_x + acc_y * acc_y);
    phi = atan2f(acc_y, -acc_x);
    alpha_z = norm_xy * sinf(phi); // aproximando R=1

    return Vector3f(alpha_x, alpha_y, alpha_z);
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

Eigen::Vector4f qDelta(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m)
{
    float qr0 = quat_r(0);
    float qr1 = quat_r(1);
    float qr2 = quat_r(2);
    float qr3 = quat_r(3);
    // q_m conjugate (*q_m):
    float qm0 =  quat_m(0);
    float qm1 = -quat_m(1);
    float qm2 = -quat_m(2);
    float qm3 = -quat_m(3);

    Eigen::Vector4f q;
    // quaternion product: q_r x *q_m:
    q(0) = qr0 * qm0 - qr1 * qm1 - qr2 * qm2 - qr3 * qm3;
    q(1) = qr0 * qm1 + qr1 * qm0 + qr2 * qm3 - qr3 * qm2;
    q(2) = qr0 * qm2 - qr1 * qm3 + qr2 * qm0 + qr3 * qm1;
    q(3) = qr0 * qm3 + qr1 * qm2 - qr2 * qm1 + qr3 * qm0;
    return q;
}

Eigen::Vector3f quatDelta2Euler(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m)
{
  using namespace Eigen;
  // quaternion product: q_r x *q_m:
  Vector4f q = qDelta(quat_r, quat_m);
  Vector3f euler;
  euler(0) = atan2f(2*q(2)*q(3) + 2*q(0)*q(1), q(3)*q(3) - q(2)*q(2) - q(1)*q(1) + q(0)*q(0));
  euler(1) = -asinf(2*q(1)*q(3) - 2*q(0)*q(2));
  euler(2) = atan2f(2*q(1)*q(2) + 2*q(0)*q(3), q(1)*q(1) + q(0)*q(0) - q(3)*q(3) - q(2)*q(2));
  return euler;
}

Eigen::Vector3f RelOmegaNED(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m, \
                            const Eigen::Vector3f* omg_r, const Eigen::Vector3f* omg_m)
{
  using namespace Eigen;
  Vector3f Omega1 = Quaternionf(*quat_r).toRotationMatrix() * (*omg_r);
  Vector3f Omega2 = Quaternionf(*quat_m).toRotationMatrix() *(*omg_m);
  return Omega2 - Omega1;
}

Eigen::Vector3f RelVector(const Eigen::Vector4f rel_quat, const Eigen::Vector3f vec_r, const Eigen::Vector3f vec_m)
{
  return  (vec_m - Eigen::Quaternionf(rel_quat).toRotationMatrix() * vec_r);
}

//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|
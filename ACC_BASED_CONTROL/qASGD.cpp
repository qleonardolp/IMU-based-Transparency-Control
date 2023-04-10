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
#include <vector>

#define OFFSET_US int(0.260 * MILLION)
#define MI0       0.3600f
#define BETA      9.7000f

/*  -- Obtain attitude quaternion:
-- Quaternion-based Attitude estimation using ASGD algorithm
-- Refs:
-- [1]: Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm,
--      Wang, Li and Zhang, Zheng and Sun, Ping, International Journal of Advanced Robotic Systems, 2015
-- [2]: Estimation of IMU and MARG orientation using a gradient descent algorithm,
--      Madgwick, Sebastian OH and Harrison, Andrew JL and Vaidyanathan, Ravi, international Conference on Rehabilitation Robotics, 2011
-- [3]: "How to integrate Quaternions", Ashwin Narayan (www.ashwinnarayan.com/post/how-to-integrate-quaternions/)
*/

typedef struct asgd_struct{
    float sampletime_;
    int     exectime_;
    std::mutex* mtx_xsens_;
    std::mutex* mtx_kalman_;
    float *imudata[6];
    Eigen::Vector4f *quaternion;
    int id;
} AsgdStruct;

void removeYaw(Eigen::Vector4f* quat);
Eigen::Vector4f qDelta(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m);
Eigen::Vector3f quatDelta2Euler(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m);
Eigen::Vector3f RelOmegaNED(const Eigen::Vector4f* quat_r, const Eigen::Vector4f* quat_m, const Eigen::Vector3f* omg_r, const Eigen::Vector3f* omg_m);
Eigen::Vector3f RelVector(const Eigen::Vector4f rel_quat, const Eigen::Vector3f vec_r, const Eigen::Vector3f vec_m);
Eigen::Vector3f RelAngAcc(const Eigen::Vector4f rel_quat, const Eigen::Vector3f rel_ang_vel, const Eigen::Vector3f rel_linear_acc);
void qASGDKalman(const AsgdStruct &bind_struct);
extern void rollBuffer(float buffer[10], const size_t length);


void qASGD(ThrdStruct &data_struct)
{
  using namespace std;
  using namespace Eigen;

#if PRIORITY
  SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

  {
      unique_lock<mutex> _(*data_struct.mtx_);
      *data_struct.param0A_ = false; // not ready
      *data_struct.param1A_ = false;  // not aborting
      *data_struct.param3F_ = false;  // not finished
  }

  // Declarations

  float imus_data[DTVC_SZ] = {0};
  
  float data_block[NUMBER_OF_IMUS][IMU_DATA_SZ] = {0};

  Vector3f right_knee_elr;
  Vector3f  left_knee_elr;
  Quaternionf q12Off(1, 0, 0, 0);
  Quaternionf q23Off(1, 0, 0, 0);
  Quaternionf q34Off(1, 0, 0, 0);
  vector<Vector4f> qASGD;

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

  if (asgd_abort) {
      unique_lock<mutex> _(*data_struct.mtx_);
      *data_struct.param3F_ = true; // finished
      return;
  }

  {   // qASGD avisa que esta pronto!
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0B_ = true;
    cout << "-> qASGD Running!\n";
  }

  // cada subthread eh declarada e associada ah uma instancia da funct 'qASGDKalman'
  AsgdStruct asgdThreadsStructs[NUMBER_OF_IMUS];
  std::mutex kalman_mutex[NUMBER_OF_IMUS];
  std::vector<std::thread> asgd_threads;
  Vector4f attQuat[NUMBER_OF_IMUS];

  for (int i = 0; i < NUMBER_OF_IMUS; i++)
  {
      asgdThreadsStructs[i].sampletime_ = data_struct.sampletime_;
      asgdThreadsStructs[i].exectime_ = data_struct.exectime_;
      asgdThreadsStructs[i].quaternion = &attQuat[i];

      asgdThreadsStructs[i].mtx_xsens_  = data_struct.mtx_vector_[i]; // outside
      asgdThreadsStructs[i].mtx_kalman_ = &kalman_mutex[i];            // inside
      for (size_t k = 0; k < IMU_DATA_SZ; k++)
      {
          size_t idx = i * IMU_DATA_SZ + k;
          asgdThreadsStructs[i].imudata[k] = data_struct.datavec_[idx];
      }

      asgdThreadsStructs[i].id = i+1;

      asgd_threads.push_back(std::thread(qASGDKalman, asgdThreadsStructs[i]));
      this_thread::sleep_for(chrono::microseconds(100));

      qASGD.push_back(Eigen::Vector4f(1,0,0,0));
  }

  looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
  int sampleT_us = data_struct.sampletime_ * MILLION;
  auto t_begin = Timer.micro_now();
  // inicializa looptimer
  Timer.start();
  do
  {
    auto begin_timestamp = chrono::steady_clock::now();

    for (size_t i = 0; i < NUMBER_OF_IMUS; i++) {
        // Lendo quaternion de cada thread:
        unique_lock<mutex> _(kalman_mutex[i]);
        qASGD[i] = attQuat[i];
    }

    // Knees Euler angles:
    right_knee_elr = quatDelta2Euler(qASGD[1], qASGD[0]);
    left_knee_elr = quatDelta2Euler(qASGD[3], qASGD[2]);


    // Remove arbitrary IMU attitude:
    auto elapsedTime = Timer.micro_now() - t_begin;
    // Time average:
    if (elapsedTime < OFFSET_US) //
    {
        float incrmnt = (data_struct.sampletime_ * MILLION) / OFFSET_US;
        q12Off.w() += incrmnt * qDelta(qASGD[1], qASGD[0])(0);
        q12Off.x() += incrmnt * qDelta(qASGD[1], qASGD[0])(1);
        q12Off.y() += incrmnt * qDelta(qASGD[1], qASGD[0])(2);
        q12Off.z() += incrmnt * qDelta(qASGD[1], qASGD[0])(3);

        q34Off.w() += incrmnt * qDelta(qASGD[3], qASGD[2])(0);
        q34Off.x() += incrmnt * qDelta(qASGD[3], qASGD[2])(1);
        q34Off.y() += incrmnt * qDelta(qASGD[3], qASGD[2])(2);
        q34Off.z() += incrmnt * qDelta(qASGD[3], qASGD[2])(3);

    }
    if (elapsedTime < (OFFSET_US + static_cast<long long>(7 * data_struct.sampletime_ * MILLION)) && elapsedTime >= OFFSET_US) {
        q12Off.normalize();
        q34Off.normalize();

    } else {   // Attitude without arbitrary IMU orientation:
        Vector4f q12(q12Off.w(), q12Off.x(), q12Off.y(), q12Off.z());
        Vector4f q34(q34Off.w(), q34Off.x(), q34Off.y(), q34Off.z());
        right_knee_elr = quatDelta2Euler(qDelta(qASGD[1], q12), qASGD[0]);
        left_knee_elr = quatDelta2Euler(qDelta(qASGD[3], q34), qASGD[2]);
    }

    // Relative Angular Velocity
    //Vector3f right_knee_vel = RelVector(qDelta(qASGD1_qk, qASGD2_qk), gyro1, gyro2);
    //Vector3f  left_knee_vel = RelVector(qDelta(qASGD3_qk, qASGD4_qk), gyro3, gyro4);


    { // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      switch (data_struct.param39_)
      {
      case OPMODE::IMU_BYPASS_CONTROL:
        *(*data_struct.datavecB_ + 0) = R2D*right_knee_elr(0);   // hum_rgtknee_pos
        *(*data_struct.datavecB_ + 1) = R2D*left_knee_elr(0);    // hum_rgtknee_vel
        *(*data_struct.datavecB_ + 2) = 0;       // hum_rgtknee_acc
        break;
      default:
        *(*data_struct.datavecA_ + 0) = R2D*right_knee_elr(0);   // hum_rgtknee_pos
        *(*data_struct.datavecA_ + 1) = (0);                     // hum_rgtknee_vel
        *(*data_struct.datavecA_ + 2) = (0);                     // hum_rgtknee_acc
        *(*data_struct.datavecA_ + 3) = R2D*left_knee_elr(0);    // hum_lftknee_pos 
        *(*data_struct.datavecA_ + 4) = (0);                     // hum_lftknee_vel

        *(*data_struct.datavecB_ + 0) = R2D*right_knee_elr(0);   // hum_rgtknee_pos
        *(*data_struct.datavecB_ + 1) = R2D*left_knee_elr(0);   // hum_rgtknee_vel
        *(*data_struct.datavecB_ + 2) = 0;                   // hum_rgtknee_acc
        *(*data_struct.datavecB_ + 3) = (0);    // hum_lftknee_pos 
        *(*data_struct.datavecB_ + 4) = (0);    // hum_lftknee_vel
        break;
      }
    } // fim da sessao critica

    this_thread::sleep_until(begin_timestamp + chrono::microseconds(sampleT_us));
  } while (!Timer.end());

  // Joining asgd threads:
  for (auto& it : asgd_threads) {
      cout << "qASGD subthread " << it.get_id() << " jointed" << endl;
      this_thread::sleep_for(chrono::milliseconds(80));
      it.join();
  }

  {   
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0B_ = false;
    *data_struct.param3F_ = true;
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

void qASGDKalman(const AsgdStruct& bind_struct)
{
    using namespace std;
    using namespace Eigen;
    // Declarations
    float q0 = 1;
    float q1 = 0;
    float q2 = 0;
    float q3 = 0;
    Vector4f qk(q0, q1, q2, q3);
    const Matrix4f H = Matrix4f::Identity();
    const Matrix4f R = Matrix4f::Identity() * 2.5e-5;

    FullPivLU<Matrix4f> Ck; // Covariance Matrix
    Matrix4f Pk = Matrix4f::Identity();
    Matrix4f Qk = Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
    Matrix4f KG; // Kalman Gain
    Vector3f gyro(0, 0, 0);
    Vector3f acc(0, 0, 0);
    Vector3f F_obj;
    Vector4f GradF;
    Vector4f z_k;
    Vector3f Zc;
    Matrix4f OmG;
    Matrix4f Psi;
    Matrix<float, 3, 4> Jq;
    Matrix<float, 4, 3> Xi;
    float omg_norm = gyro.norm();
    float mi(0);

    float Ts = bind_struct.sampletime_;
    int sampleT_us = Ts * MILLION;

    looptimer Timer(Ts, bind_struct.exectime_);
    auto t_begin = Timer.micro_now();
    // inicializa looptimer
    cout << "\n-> qASGD[" << bind_struct.id << "] Running!\n";
    Timer.start();
    do
    {
        //Timer.tik();
        auto begin_timestamp = chrono::steady_clock::now();

        { // sessao critica:
            unique_lock<mutex> _(*bind_struct.mtx_xsens_);
            gyro << *bind_struct.imudata[0], *bind_struct.imudata[1], *bind_struct.imudata[2];
            acc << *bind_struct.imudata[3], *bind_struct.imudata[4], *bind_struct.imudata[5];                      
        } // fim da sessao critica (ext)

        // ASGD iteration:
        Zc << 2 * (q1 * q3 - q0 * q2),
            2 * (q2 * q3 + q0 * q1),
            (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
        F_obj = Zc - acc.normalized(); // Eq.23

        Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
            2 * q1, 2 * q0, 2 * q3, 2 * q2,
            2 * q0, -2 * q1, -2 * q2, 2 * q3;

        GradF = Jq.transpose() * F_obj; // Eq.25

        omg_norm = gyro.norm();
        mi = MI0 + BETA * Ts * omg_norm; // Eq.29

        z_k = qk - mi * GradF.normalized(); // Eq.24
        z_k.normalize();

        OmG << 0, -gyro(0), -gyro(1), -gyro(2),
            gyro(0), 0, gyro(2), -gyro(1),
            gyro(1), -gyro(2), 0, gyro(0),
            gyro(2),  gyro(1), -gyro(0), 0;
        OmG = 0.5 * OmG;

        Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * H + 0.5 * Ts * OmG; // Using H as 'I_44'

        // Process noise covariance update (Eq. 19):
        Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
        Qk = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
        // Projection:
        qk = Psi * qk;
        Pk = Psi * Pk * Psi.transpose() + Qk;
        // Kalman Gain (H is Identity)
        Ck = FullPivLU<Matrix4f>(Pk + R);
        if (Ck.isInvertible())
            KG = Pk * Ck.inverse();
        // Update (H is Identity)
        qk = qk + KG * (z_k - qk);
        Pk = (Matrix4f::Identity() - KG) * Pk;
        qk.normalize();

        // Rotate the quaternion by a quaternion with -(yaw):
        removeYaw(&qk);

        {// Write at the output quaternion:
            unique_lock<mutex> _(*bind_struct.mtx_kalman_);
            *bind_struct.quaternion << qk(0), qk(1), qk(2), qk(3);
        }

        //Timer.tak();
        this_thread::sleep_until( begin_timestamp + chrono::microseconds(sampleT_us) );

    } while (!Timer.end());
}

//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|
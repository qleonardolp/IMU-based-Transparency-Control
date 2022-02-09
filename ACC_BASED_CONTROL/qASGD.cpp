//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/

#ifdef _WIN32
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#else
#include <windows.h>
#endif
#include "XsensEpos.h"
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>

/*  -- Obtain attitude quaternion:
    -- Quaternion-based Attitude estimation using ASGD algorithm
    -- Refs:
    -- [1]: Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm,
    --      Wang, Li and Zhang, Zheng and Sun, Ping, International Journal of Advanced Robotic Systems, 2015
    -- [2]: Estimation of IMU and MARG orientation using a gradient descent algorithm,
    --      Madgwick, Sebastian OH and Harrison, Andrew JL and Vaidyanathan, Ravi, international Conference on Rehabilitation Robotics, 2011
    -- [3]: "How to integrate Quaternions", Ashwin Narayan (www.ashwinnarayan.com/post/how-to-integrate-quaternions/)
*/

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
    Matrix4f Q1 = Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
    Matrix4f Q2 = Q1;

    Vector3f gyro1(0, 0, 0);
    Vector3f gyro2(0, 0, 0);
    Vector3f acc1(0, 0, 0);
    Vector3f acc2(0, 0, 0);
    Vector3f F_obj;
    Vector4f GradF;
    Vector3f Zc;
    Matrix4f OmG;
    Matrix4f Psi;
    Matrix4f Kg;
    Matrix4f Qy;
    Matrix<float, 3, 4> Jq;
    Matrix<float, 4, 3> Xi;
    float omg_norm = gyro1.norm();
    float mi(0);

    //do {    } while (!imu_isready);
    asgd_isready = true;

    looptimer Timer(data_struct.sampletime_);
    llint exec_time_micros = data_struct.exectime_ * MILLION;
    llint t_begin = Timer.micro_now();
    do
    {
        Timer.tik();
        { // sessao critica: minimo codigo necessario para pegar datavec_
            unique_lock<mutex> _(*data_struct.mtx_);
            memcpy(imus_data, *data_struct.datavec_, 18*sizeof(float));
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

        Vector4f z_k = qASGD1_qk - mi * GradF.normalized(); // Eq.24
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
        FullPivLU<Matrix4f> TotalCovariance(qASGD1_Pk + R);
        if (TotalCovariance.isInvertible())
            Kg = qASGD1_Pk * TotalCovariance.inverse();
        // Update (H is Identity)
        qASGD1_qk = qASGD1_qk + Kg * (z_k - qASGD1_qk);
        qASGD1_Pk = (Matrix4f::Identity() - Kg) * qASGD1_Pk;
        qASGD1_qk.normalize();

        // Remove Yaw: Rotate the quaternion by a quaternion with -(yaw):
        q0 = qASGD1_qk(0);
        q1 = qASGD1_qk(1);
        q2 = qASGD1_qk(2);
        q3 = qASGD1_qk(3);
        float yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
        Qy = Matrix4f::Identity() * cosf(-yaw / 2);
        Qy(0, 3) = -sinf(-yaw / 2);
        Qy(1, 2) = Qy(0, 3);
        Qy(2, 1) = -Qy(0, 3);
        Qy(3, 0) = -Qy(0, 3);
        qASGD1_qk = Qy * qASGD1_qk;
        qASGD1_qk.normalize();

        Timer.tak();
    } while (Timer.micro_now() - t_begin <= exec_time_micros);

    // ending stuff...
}
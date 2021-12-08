///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2019     ///
// leonardo.felipe.santos@usp.br	_____ ___  ___   //
// github/bitbucket qleonardolp		| |  | . \/   \  //
////////////////////////////////	| |   \ \   |_|  //
////////////////////////////////	\_'_/\_`_/__|    //
///////////////////////////////////////////////////////

/*
Copyright 2019-2023 Leonardo Felipe Lima Santos dos Santos

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "qASGD_KF.h"

void qASGDKF::Recorder()
{
	logger = fopen(getLogfilename(), "a");
	if (logger != NULL)
	{
    timestamp = (float)1e-6*duration_cast<microseconds>(system_clock::now() - timestamp_begin).count();
    /*
		fprintf(logger, "%5.6f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n",\
						timestamp, qASGD1_qk[0], qASGD1_qk[1], qASGD1_qk[2], qASGD1_qk[3], \
								   qASGD2_qk[0], qASGD2_qk[1], qASGD2_qk[2], qASGD2_qk[3]);
    */
    Vector3f euler = quat2euler(1)*(180 / MY_PI);
    fprintf(logger, "%5.6f,%5.5f,%5.5f,%5.5f\n", timestamp, euler(0), euler(1), euler(2));
		fclose(logger);
	}
}

float qASGDKF::constrain_float(float val, float min, float max)
{
	if (_isnan(val)) return (min + max)/2;

	if (val < min) return min;

	if(val > max) return max;
	
	return val;
}

void qASGDKF::updateqASGD1Kalman(Vector3f gyro, Vector3f acc, float Dt)
{
	//Vector4f a_b(0, acc.normalized()(0), acc.normalized()(1), acc.normalized()(2));

	float q0 = qASGD1_qk(0);
	float q1 = qASGD1_qk(1);
	float q2 = qASGD1_qk(2);
	float q3 = qASGD1_qk(3);

	/* 
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
	*/

	Vector3f F_obj = Vector3f::Zero();
	//F_obj = Rot.transpose()*Vector3f(0,0,1) - acc.normalized();

	// De maneira simplificada podemos fazer apenas:
	// g = [0 0 1]^T, portanto so nos interessa a ultima coluna de C^T:
	Vector3f Z_c = Vector3f::Zero();
	Z_c << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 - q3*q3);
	F_obj = Z_c - acc.normalized();	// Eq.23

	Matrix<float,3,4> Jq;
	Jq << -2*q2, 2*q3, -2*q0, 2*q1, 
		   2*q1, 2*q0,  2*q3, 2*q2, 
		   2*q0,-2*q1, -2*q2, 2*q3;
	
	Vector4f GradF;
	GradF = Jq.transpose()*F_obj;	// Eq.25

	float Ts = 0.0000;
	float omg_norm = gyro.norm();

#if FIXED_DT
	Ts = DELTA_T;
#else
	Ts = constrain_float(Dt, 1.00e-6, 0.020);
#endif

	float mi = mi0 + Beta*Ts*omg_norm; // Eq.29

	Vector4f z_k;
	z_k = qASGD1_qk - mi*GradF.normalized(); // Eq.24

	Matrix4f Q = Matrix4f::Identity()*5.476e-6;	// Usar Eq. 19...
	Matrix4f R = Matrix4f::Identity()*5.476e-6;
	Matrix4f H = Matrix4f::Identity();

	Matrix4f OmG = Matrix4f::Zero();
	OmG << 0, -gyro(0), -gyro(1), -gyro(2),
		 gyro(0), 0, gyro(2), -gyro(1),
		 gyro(1), -gyro(2), 0, gyro(0),
		 gyro(2), gyro(1), -gyro(0), 0;

	OmG = 0.5*OmG;

	Matrix4f Psi;
	Psi = (1 - ((omg_norm*Ts)*(omg_norm*Ts))/8)*Matrix4f::Identity() + 0.5*Ts*OmG;

	// Process noise covariance update (Eq. 19):
	Matrix<float,4,3> Xi;
	Xi << q0, q3, -q2,
	     -q3, q0,  q1,
		  q2, -q1, q0,
		 -q1, -q2, -q3; 

	Q = 0.5*Ts*Xi*(Matrix3f::Identity()*5.476e-6)*Xi.transpose();

	// Projection:
	qASGD1_qk = Psi*qASGD1_qk;
	qASGD1_Pk = Psi*qASGD1_Pk*Psi.transpose() + Q;

	// Kalman Gain
	Matrix4f Kg;
	FullPivLU<Matrix4f> TotalCovariance(H * qASGD1_Pk * H.transpose() + R);
	if (TotalCovariance.isInvertible()){
		Kg = qASGD1_Pk * H.transpose() * TotalCovariance.inverse();
	}
	// Update
	qASGD1_qk = qASGD1_qk + Kg * (z_k - H*qASGD1_qk);
	qASGD1_Pk = (Matrix4f::Identity() - Kg*H)*qASGD1_Pk;

}

void qASGDKF::updateqASGD2Kalman(Vector3f gyro, Vector3f acc, float Dt)
{
	float q0 = qASGD2_qk(0);
	float q1 = qASGD2_qk(1);
	float q2 = qASGD2_qk(2);
	float q3 = qASGD2_qk(3);

	Vector3f F_obj = Vector3f::Zero();
	Vector3f Z_c = Vector3f::Zero();
	// De maneira simplificada podemos fazer apenas:
	// g = [0 0 1]^T, portanto so nos interessa a ultima coluna de C^T:
	Z_c << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 - q3*q3);
	F_obj = Z_c - acc.normalized();	// Eq.23

	Matrix<float,3,4> Jq;
	Jq << -2*q2, 2*q3, -2*q0, 2*q1, 
		   2*q1, 2*q0,  2*q3, 2*q2, 
		   2*q0,-2*q1, -2*q2, 2*q3;
	
	Vector4f GradF;
	GradF = Jq.transpose()*F_obj;	// Eq.25

	float Ts = 0.0000;
	float omg_norm = gyro.norm();

#if FIXED_DT
	Ts = DELTA_T;
#else
	Ts = constrain_float(Dt, 1.00e-6, 0.020);
#endif

	float mi = mi0 + Beta*Ts*omg_norm; // Eq.29

	Vector4f z_k;
	z_k = qASGD2_qk - mi*GradF.normalized(); // Eq.24

	Matrix4f Q = Matrix4f::Identity()*5.476e-6;	// Usar Eq. 19...
	Matrix4f R = Matrix4f::Identity()*5.476e-6;
	Matrix4f H = Matrix4f::Identity();

	Matrix4f OmG = Matrix4f::Zero();
	OmG << 0, -gyro(0), -gyro(1), -gyro(2),
		 gyro(0), 0, gyro(2), -gyro(1),
		 gyro(1), -gyro(2), 0, gyro(0),
		 gyro(2), gyro(1), -gyro(0), 0;

	OmG = 0.5*OmG;

	Matrix4f Psi;
	Psi = (1 - ((omg_norm*Ts)*(omg_norm*Ts))/8)*Matrix4f::Identity() + 0.5*Ts*OmG;

	// Process noise covariance update (Eq. 19):
	Matrix<float,4,3> Xi;
	Xi << q0, q3, -q2,
	     -q3, q0,  q1,
		  q2, -q1, q0,
		 -q1, -q2, -q3; 

	Q = 0.5*Ts*Xi*(Matrix3f::Identity()*5.476e-6)*Xi.transpose();

	// Projection:
	qASGD2_qk = Psi*qASGD2_qk;
	qASGD2_Pk = Psi*qASGD2_Pk*Psi.transpose() + Q;

	// Kalman Gain
	Matrix4f Kg;
	FullPivLU<Matrix4f> TotalCovariance(H * qASGD2_Pk * H.transpose() + R);
	if (TotalCovariance.isInvertible()){
		Kg = qASGD2_Pk * H.transpose() * TotalCovariance.inverse();
	}
	// Update
	qASGD2_qk = qASGD2_qk + Kg * (z_k - H*qASGD2_qk);
	qASGD2_Pk = (Matrix4f::Identity() - Kg*H)*qASGD2_Pk;
}

Vector3f qASGDKF::quat2euler(Vector4f* quat)
{
	float q0 = (*quat)(0);
	float q1 = (*quat)(1);
	float q2 = (*quat)(2);
	float q3 = (*quat)(3);

	Vector3f euler;

	euler(0) = atan2f(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
	euler(1) = -asinf(2*q1*q3 - 2*q0*q2);
	euler(2) = atan2f(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);

	return euler;
}

Vector3f qASGDKF::quat2euler(int id)
{
	float q0, q1, q2, q3;

	switch (id)
	{
	case 1:
		q0 = qASGD1_qk[0];
		q1 = qASGD1_qk[1];
		q2 = qASGD1_qk[2];
		q3 = qASGD1_qk[3];
		break;
	case 2:
		q0 = qASGD2_qk[0];
		q1 = qASGD2_qk[1];
		q2 = qASGD2_qk[2];
		q3 = qASGD2_qk[3];
		break;
	default:
		q0 = qASGD1_qk[0];
		q1 = qASGD1_qk[1];
		q2 = qASGD1_qk[2];
		q3 = qASGD1_qk[3];
		break;
	}

	Vector3f euler;

	euler(0) = atan2f(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
	euler(1) = -asinf(2*q1*q3 - 2*q0*q2);
	euler(2) = atan2f(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);

	return euler;
}


Vector3f qASGDKF::quatDelta2euler(Vector4f* quat_r, Vector4f* quat_m)
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

	Vector3f euler;
	Vector4f q;

	// quaternion product: q_r x *q_m:
	q(0) = qr0*qm0 - qr1*qm1 - qr2*qm2 - qr3*qm3;
	q(1) = qr0*qm1 + qr1*qm0 + qr2*qm3 - qr3*qm2;
	q(2) = qr0*qm2 - qr1*qm3 + qr2*qm0 + qr3*qm1;
	q(3) = qr0*qm3 + qr1*qm2 - qr2*qm1 + qr3*qm0;

	euler(0) = atan2f(2*q(2)*q(3) + 2*q(0)*q(1), q(3)*q(3) - q(2)*q(2) - q(1)*q(1) + q(0)*q(0));
	euler(1) = -asinf(2*q(1)*q(3) - 2*q(0)*q(2));
	euler(2) = atan2f(2*q(1)*q(2) + 2*q(0)*q(3), q(1)*q(1) + q(0)*q(0) - q(3)*q(3) - q(2)*q(2));

	return euler;
}

void qASGDKF::GainScan()
{
	FILE* pFile = fopen("ASGDparam.txt", "rt");

	if (pFile != NULL)
	{
		fscanf(pFile, "mi0 %f\nbeta %f\n", &mi0, &Beta);
		fclose(pFile);
	}
}
///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2022     ///
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

#include "qASGD.h"

void qASGDKF::Recorder()
{
	if(logging.load()){
		logger = fopen(getLogfilename(), "a");
		if (logger != NULL)
		{
		timestamp = (float)1e-6*duration_cast<microseconds>(system_clock::now() - timestamp_begin).count();

		Vector3f euler1 = quat2euler(1)*(180 / MY_PI);
		Vector3f euler2 = quat2euler(2)*(180 / MY_PI);
    	Vector3f DelAng = quatDelta2euler()*(180 / MY_PI);
    	Vector3f RelVelocity = RelOmegaNED()*(180 / MY_PI);
		fprintf(logger, "%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n", \
				timestamp, euler1(0), euler1(1), euler2(0), euler2(1), DelAng(0), RelVelocity(0));
			fclose(logger);
		}
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
	/*
	-- Quaternion-based Attitude estimation using ASGD algorithm:
	-- [1]: Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm (2015)
	-- [2]: Estimation of IMU and MARG orientation using a gradient descent algorithm (2011)
	-- [3]: "How to integrate Quaternions", Ashwin Narayan (www.ashwinnarayan.com/post/how-to-integrate-quaternions/)
	*/

	float q0 = qASGD1_qk(0);
	float q1 = qASGD1_qk(1);
	float q2 = qASGD1_qk(2);
	float q3 = qASGD1_qk(3);
	gyro1 = gyro;

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

	// De maneira simplificada podemos fazer apenas:
	// g = [0 0 1]^T, portanto so nos interessa a ultima coluna de C^T:
	Vector3f Zc = Vector3f::Zero();
	Zc << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 - q3*q3);
	Vector3f F_obj = Zc - acc.normalized();	// Eq.23

	Matrix<float,3,4> Jq;
	Jq << -2*q2, 2*q3, -2*q0, 2*q1, 
		   2*q1, 2*q0,  2*q3, 2*q2, 
		   2*q0,-2*q1, -2*q2, 2*q3;
	
	Vector4f GradF = Jq.transpose()*F_obj;	// Eq.25

	float Ts = 0.0000;
	float omg_norm = gyro.norm();

#if FIXED_DT
	Ts = delta_t;
#else
	Ts = constrain_float(Dt, 1.00e-6, 0.050);
#endif

	float mi = mi0 + Beta*Ts*omg_norm; // Eq.29

	Vector4f z_k = qASGD1_qk - mi*GradF.normalized(); // Eq.24
	z_k.normalize();

	Matrix4f OmG = Matrix4f::Zero();
	OmG << 0, -gyro(0), -gyro(1), -gyro(2),
		 gyro(0), 0, gyro(2), -gyro(1),
		 gyro(1), -gyro(2), 0, gyro(0),
		 gyro(2), gyro(1), -gyro(0), 0;

	OmG = 0.5*OmG;

	Matrix4f Psi = (1 - ((omg_norm*Ts)*(omg_norm*Ts))/8)*Matrix4f::Identity() + 0.5*Ts*OmG;

	// Process noise covariance update (Eq. 19):
	Matrix<float,4,3> Xi;
	Xi << q0, q3, -q2,
	     -q3, q0,  q1,
		  q2, -q1, q0,
		 -q1, -q2, -q3; 

	Q1 = 0.5*Ts*Xi*(Matrix3f::Identity()*5.476e-6)*Xi.transpose();

	// Projection:
	qASGD1_qk = Psi*qASGD1_qk;
	qASGD1_Pk = Psi*qASGD1_Pk*Psi.transpose() + Q1;

	// Kalman Gain (H is Identity)
	Matrix4f Kg;
	FullPivLU<Matrix4f> TotalCovariance(qASGD1_Pk + R);
	if (TotalCovariance.isInvertible()){
		Kg = qASGD1_Pk * TotalCovariance.inverse();
	}
	// Update (H is Identity)
	qASGD1_qk = qASGD1_qk + Kg * (z_k - qASGD1_qk);
	qASGD1_Pk = (Matrix4f::Identity() - Kg)*qASGD1_Pk;
	qASGD1_qk.normalize();

	// Remove Yaw: Rotate the quaternion by a quaternion with -(yaw):
	q0 = qASGD1_qk(0);
	q1 = qASGD1_qk(1);
	q2 = qASGD1_qk(2);
	q3 = qASGD1_qk(3);

	float yaw = atan2f(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
	Matrix4f Qy = Matrix4f::Identity()*cosf(-yaw/2);
	Qy(0,3) = -sinf(-yaw/2);
	Qy(1,2) =  Qy(0,3);
	Qy(2,1) = -Qy(0,3);
	Qy(3,0) = -Qy(0,3);

	qASGD1_qk = Qy*qASGD1_qk;
	qASGD1_qk.normalize();
}

void qASGDKF::updateqASGD2Kalman(Vector3f gyro, Vector3f acc, float Dt)
{
	float q0 = qASGD2_qk(0);
	float q1 = qASGD2_qk(1);
	float q2 = qASGD2_qk(2);
	float q3 = qASGD2_qk(3);
	gyro2 = gyro;

	// De maneira simplificada podemos fazer apenas:
	// g = [0 0 1]^T, portanto so nos interessa a ultima coluna de C^T:
	Vector3f Zc = Vector3f::Zero();
	Zc << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 - q3*q3);
	Vector3f F_obj = Zc - acc.normalized();	// Eq.23

	Matrix<float,3,4> Jq;
	Jq << -2*q2, 2*q3, -2*q0, 2*q1, 
		   2*q1, 2*q0,  2*q3, 2*q2, 
		   2*q0,-2*q1, -2*q2, 2*q3;
	
	Vector4f GradF = Jq.transpose()*F_obj;	// Eq.25

	float Ts = 0.0000;
	float omg_norm = gyro.norm();

#if FIXED_DT
	Ts = delta_t;
#else
	Ts = constrain_float(Dt, 1.00e-6, 0.050);
#endif

	float mi = mi0 + Beta*Ts*omg_norm; // Eq.29

	Vector4f z_k = qASGD2_qk - mi*GradF.normalized(); // Eq.24
	z_k.normalize();

	Matrix4f OmG = Matrix4f::Zero();
	OmG << 0, -gyro(0), -gyro(1), -gyro(2),
		 gyro(0), 0, gyro(2), -gyro(1),
		 gyro(1), -gyro(2), 0, gyro(0),
		 gyro(2), gyro(1), -gyro(0), 0;

	OmG = 0.5*OmG;

	Matrix4f Psi = (1 - ((omg_norm*Ts)*(omg_norm*Ts))/8)*Matrix4f::Identity() + 0.5*Ts*OmG;

	// Process noise covariance update (Eq. 19):
	Matrix<float,4,3> Xi;
	Xi << q0, q3, -q2,
	     -q3, q0,  q1,
		  q2, -q1, q0,
		 -q1, -q2, -q3; 

	Q2 = 0.5*Ts*Xi*(Matrix3f::Identity()*5.476e-6)*Xi.transpose();

	// Projection:
	qASGD2_qk = Psi*qASGD2_qk;
	qASGD2_Pk = Psi*qASGD2_Pk*Psi.transpose() + Q2;

	// Kalman Gain (H is Identity)
	Matrix4f Kg;
	FullPivLU<Matrix4f> TotalCovariance(qASGD2_Pk + R);
	if (TotalCovariance.isInvertible()){
		Kg = qASGD2_Pk * TotalCovariance.inverse();
	}
	// Update (H is Identity)
	qASGD2_qk = qASGD2_qk + Kg * (z_k - qASGD2_qk);
	qASGD2_Pk = (Matrix4f::Identity() - Kg)*qASGD2_Pk;
	qASGD2_qk.normalize();

	// Remove Yaw: Rotate the quaternion by a quaternion with -(yaw):
	q0 = qASGD2_qk(0);
	q1 = qASGD2_qk(1);
	q2 = qASGD2_qk(2);
	q3 = qASGD2_qk(3);

	float yaw = atan2f(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
	Matrix4f Qy = Matrix4f::Identity()*cosf(-yaw/2);
	Qy(0,3) = -sinf(-yaw/2);
	Qy(1,2) =  Qy(0,3);
	Qy(2,1) = -Qy(0,3);
	Qy(3,0) = -Qy(0,3);

	qASGD2_qk = Qy*qASGD2_qk;
	qASGD2_qk.normalize();
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


Vector3f qASGDKF::quatDelta2euler()
{
	float qr0 = qASGD2_qk[0];
	float qr1 = qASGD2_qk[1];
	float qr2 = qASGD2_qk[2];
	float qr3 = qASGD2_qk[3];
	// q_m conjugate (*q_m):
	float qm0 =  qASGD1_qk[0];
	float qm1 = -qASGD1_qk[1];
	float qm2 = -qASGD1_qk[2];
	float qm3 = -qASGD1_qk[3];

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

Vector3f qASGDKF::RelOmegaNED()
{
	float q0 = qASGD1_qk(0);
	float q1 = qASGD1_qk(1);
	float q2 = qASGD1_qk(2);
	float q3 = qASGD1_qk(3);

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

	Vector3f Omega1 = Rot*gyro1;

	q0 = qASGD2_qk(0);
	q1 = qASGD2_qk(1);
	q2 = qASGD2_qk(2);
	q3 = qASGD2_qk(3);

	Rot(0,0) = (q0*q0 + q1*q1 - q2*q2 - q3*q3);
	Rot(0,1) = 2*(q1*q2 - q0*q3);
	Rot(0,2) = 2*(q1*q3 + q0*q2);
	Rot(1,0) = 2*(q1*q2 + q0*q3);
	Rot(1,1) = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
	Rot(1,2) = 2*(q2*q3 - q0*q1);
	Rot(2,0) = 2*(q1*q3 - q0*q2);
	Rot(2,1) = 2*(q2*q3 + q0*q1);
	Rot(2,2) = (q0*q0 - q1*q1 - q2*q2 - q3*q3); 

	Vector3f Omega2 = Rot*gyro2;

	return Omega2 - Omega1;

}


void qASGDKF::GainScan()
{
	FILE* pFile = fopen("ASGDparam.txt", "rt");

	if (pFile != NULL)
	{
		fscanf(pFile, "mi0 %f\nbeta %f\nrho %f\n", &mi0, &Beta, &Rho);
		fclose(pFile);
	}
}
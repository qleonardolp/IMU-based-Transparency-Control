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

/*
-- Obtain attitude quaternion:
    -- Quaternion-based Attitude estimation using ASGD algorithm
    -- Refs: 
    -- [1]: Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm, 
    --      Wang, Li and Zhang, Zheng and Sun, Ping, International Journal of Advanced Robotic Systems, 2015
    -- [2]: Estimation of IMU and MARG orientation using a gradient descent algorithm,
    --      Madgwick, Sebastian OH and Harrison, Andrew JL and Vaidyanathan, Ravi, international Conference on Rehabilitation Robotics, 2011
    -- [3]: "How to integrate Quaternions", Ashwin Narayan (www.ashwinnarayan.com/post/how-to-integrate-quaternions/)
*/

#ifndef QASGD_H
#define QASGD_H

#include <stdio.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <atomic>
#include <chrono>

#include <Eigen/LU>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

// CONSTANTS
#ifndef M_PI
  #define M_PI 3.141592653f
#endif
#define		RADS2RPM	(30/M_PI)		// rad/s to rpm 
#define		RPM2RADS	(M_PI/30)		// rpm to rad/s
#define		FIXED_DT	1

using namespace Eigen;
using namespace std::chrono;

class qASGD
{
public:
	// Class Constructor
	qASGD(int seconds, float xsens_rate) : log_seconds(seconds)
	{
		imu_rate = xsens_rate;		// [Hz]	IMUs update rate
		delta_t  = 1/xsens_rate;	// [s]

		logging = false;
		//Criar arguivo de log com cabecalho:
		if (log_seconds > 0)
		{
			timestamp_begin = system_clock::now();
			logging = true;
			time_t rawtime;
			struct tm* timeinfo;
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			char header_timestamp[30];
			strftime(header_timestamp, sizeof(header_timestamp), "%Y-%m-%d-%H-%M-%S", timeinfo);
			char logfilename[40];
			strftime(logfilename, sizeof(logfilename), "./data/asgd-%Y-%m-%d-%H-%M-%S.txt", timeinfo);
      		strcpy(logger_filename, logfilename);

			logger = fopen(getLogfilename(), "wt");
			if (logger != NULL)
			{	
				// printing the header into the file first line
				fprintf(logger, "qASGD: [%s]\n\n", header_timestamp);
				fclose(logger);
			}
		}

		// qASGD-KF for IMUs AHRS:
		qASGD1_Pk.setIdentity();
		qASGD2_Pk.setIdentity();
		qASGD1_qk << 1,0,0,0;
		qASGD2_qk << 1,0,0,0;

		H = Matrix4f::Identity();
		R = Matrix4f::Identity()*1e-5;
		Q1 = Matrix4f::Identity()*5.476e-6;	// Usar Eq. 19...
		Q2 = Q1;

		mi0 = 0.100;
		Beta = 1.1400;
		Rho = 0.00017;
	}

	// Update method for qASGD AHRS Kalman Filer
	void updateqASGD1Kalman(Vector3f gyro, Vector3f acc, float Dt);
	void updateqASGD2Kalman(Vector3f gyro, Vector3f acc, float Dt);

	// Convert Quaternion in Euler Angles (rad)
	Vector3f quat2euler(Vector4f* quat);
	Vector3f quat2euler(int id);

	// Convert Difference Quaternion in Euler Angles (rad)
	Vector3f quatDelta2euler();

	// Angular relative velocity vector in Earth Frame (NED):
	Vector3f RelOmegaNED();

	// Saturation method
	float constrain_float(float val, float min, float max);

	// Log method:
	void Recorder();

	// Read parameters from file:
	void GainScan();

	// Class Destructor
	~qASGD(){	}

private:
	int log_seconds;
	FILE* logger;
	std::atomic<bool> logging;
	char logger_filename[40];
	float timestamp;
	system_clock::time_point timestamp_begin;
	int imu_rate;
	float delta_t;

	// qASGD Kalman for AHRS: 
	Vector4f qASGD1_qk, qASGD2_qk;
	Matrix4f qASGD1_Pk, qASGD2_Pk;
	Matrix4f Q1, Q2, R, H;
	float mi0, Beta, Rho;

	Vector3f acc1, acc2;
	Vector3f gyro1, gyro2;

	// Get the log file name
	char* getLogfilename(){return logger_filename;}
};

#endif // !QASGD_H

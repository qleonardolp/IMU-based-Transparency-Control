///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2021     ///
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

#ifndef QASGDKF_H
#define QASGDKF_H

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

#define		MY_PI		3.141592653f	// Pi value
#define		RADS2RPM	(30/MY_PI)		// rad/s to rpm 
#define		RPM2RADS	(MY_PI/30)		// rpm to rad/s

#define		IMU_RATE	120.0f		// [Hz]	IMUs update rate
#define		DELTA_T		(1/IMU_RATE) 	// [s]
#define		FIXED_DT	0
#define		MI0			0.010
#define		BETA		10.00

using namespace Eigen;
using namespace std::chrono;

class qASGDKF
{
public:
	// Class Constructor
	qASGDKF(int seconds) : log_seconds(seconds)
	{
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

		mi0 = MI0;
		Beta = BETA;
	}

	// Update method for qASGD AHRS Kalman Filer
	void updateqASGD1Kalman(Vector3f gyro, Vector3f acc, float Dt);
	void updateqASGD2Kalman(Vector3f gyro, Vector3f acc, float Dt);

	// Convert Quaternion in Euler Angles (rad)
	Vector3f quat2euler(Vector4f* quat);
	Vector3f quat2euler(int id);

	// Convert Difference Quaternion in Euler Angles (rad)
	Vector3f quatDelta2euler(Vector4f* quat_r, Vector4f* quat_m);

	// Saturation method
	float constrain_float(float val, float min, float max);

	// Log method:
	void Recorder();

	// Read parameters from file:
	void GainScan();

	// Class Destructor
	~qASGDKF(){	}

private:
	int log_seconds;
	FILE* logger;
	std::atomic<bool> logging;
	char logger_filename[40];
	float timestamp;
	system_clock::time_point timestamp_begin;

	// qASGD Kalman for AHRS: 
	// (WANG, Li; ZHANG, Zheng; SUN, Ping. Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm.)
	Vector4f qASGD1_qk;
	Matrix4f qASGD1_Pk;
	Vector4f qASGD2_qk;
	Matrix4f qASGD2_Pk;

	float mi0;
	float Beta;

	// Get the log file name
	char* getLogfilename(){return logger_filename;}
};

#endif // !QASGDKF_H

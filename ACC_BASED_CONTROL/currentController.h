///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2019     ///
// leonardo.felipe.santos@usp.br	_____ ___  ___   //
// github/bitbucket qleonardolp		| |  | \ \/   \  //
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

#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <thread>

#include <Eigen/LU>
#include <Eigen/Core>

#define		SGVECT_SIZE		11				// Size of the window vector for Savitsky-Golay smoothing and derivative

// CONSTANTS

#define		GEAR_RATIO		150.0		    // Redução do Sistema
#define		ENCODER_IN		4096		    // Resolução do encoder do motor
#define		ENCODER_OUT		2048		    // Resolução do encoder de saída
#define		STIFFNESS		104.0			// Constante da mola SEA [N.m/rad]

//	Torque Constant: 0.0603 N.m/A = 60.3 N.m/mA
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A

#define		CURRENT_MAX		3.1000		// Max corrente nominal no motor Maxon RE40 [A]
#define		VOLTAGE_MAX		21.600		// Max tensão de saída Vcc = 0.9*24V fornecida pela EPOS 24/5
#define		TORQUE_CONST	60.300		// Constante de torque do motor RE40	[N.m/mA]
#define		SPEED_CONST		158.00		// Constante de velocidade do motor RE40 [rpm/V]

#define     GRAVITY         9.8066      // [m/s^2]
#define     INERTIA_EXO     0.0655 * 4  // [Kg.m^2], +- 0.0006, estimado em 2019-08-21
#define		MTW_DIST_LIMB	0.2500		// [m]
#define		MTW_DIST_EXO	0.0700		// [m]
#define		L_CG			0.3500		// [m]

// According to W. M. Dos Santos and A. A. G. Siqueira in 10.1109/BIOROB.2014.6913851 (DOI)
#define		J_EQ			0.4700		// [Kg.m^2]
#define		B_EQ			60.000		// [N.m s/rad]

// Feedforward-Feedback PI acc-based controller:
#define		K_FF			1.0000		// [dimensionless]
#define     KP_A			4.5600      // [Kg.m^2]
#define     KI_A			5.7900      // [Kg.m^2/s]

// Feedback PD force (SEA) controller:
#define     KP_F			1.5310      // [dimensionless]
#define     KD_F			0.0200      // [s]


#define     RATE            125.0      // [Hz]	Use the control loop rate running
#define		DELTA_T			(1/RATE)   // [s]

// Low Pass Filtering
#define     LPF_FC          5.000      // [Hz] Low Pass Filter Frequency Cutoff
#define		MY_PI			3.141592653	// Pi value
#define		LPF_SMF         ( (2*MY_PI / RATE) / (2*MY_PI / RATE + 1 / LPF_FC) )    // Low Pass Filter Smoothing Factor

typedef Eigen::Matrix<float, 5, 1> Vector5f;
typedef Eigen::Matrix<float, 5, 5> Matrix5f;


class accBasedControl
{
public:
	// constructor
	accBasedControl(EPOS_NETWORK* epos, AXIS* eixo_in, AXIS* eixo_out, char control_mode, int seconds)
	{

		m_epos = epos;
		m_eixo_in = eixo_in;
		m_eixo_out = eixo_out;

		pos0_out = -m_eixo_out->PDOgetActualPosition();
		pos0_in = m_eixo_in->PDOgetActualPosition();

		if (control_mode == 'c' || control_mode == 'k')
		{
			m_eixo_in->VCS_SetOperationMode(CURRENT_MODE);    // For FiniteDiff and CurrentControlKF functions
		}
		if (control_mode == 's')
		{
			m_eixo_in->VCS_SetOperationMode(VELOCITY_MODE);   // For OmegaControl function
		}

		// Current Control
		K_ff = K_FF;
		Kp_A = KP_A;
		Ki_A = KI_A;
		Kp_F = KP_F;
		Kd_F = KD_F;
		Amplifier = 100000; // initialized with a safe value

		// Speed Control
		Kff_V = 1.000;
		Kp_V = 0.100;
		Ki_V = 0.100;
		Kd_V = 0.100;
		Amp_V = 50;			// initialized with a safe value

		// Kalman Filter
		Amp_kf = 1;

		vel_hum = 0;
		vel_exo = 0;
		setpoint = 0;
		setpoint_filt = 0;
		accbased_comp = 0;
		torque_sea = 0;

		for (size_t i = 0; i < SGVECT_SIZE; ++i)
		{
			velhumVec[i] = 0;
			velexoVec[i] = 0;
			theta_l_vec[i] = 0;
			theta_c_vec[i] = 0;
			torqueSeaVec[i] = 0;
			torqueAccVec[i] = 0;	
		}

		if (seconds != 0)
		{
			logging = true;

			time_t rawtime;
			struct tm* timeinfo;
			time(&rawtime);
			timeinfo = localtime(&rawtime);

			strftime(logger_filename, 40, "./data/%Y-%m-%d-%H-%M-%S.txt", timeinfo);
			logger = fopen(logger_filename, "wt");
			if (logger != NULL)
			{
				// printing the header on the first line of the file
				if (control_mode == 'c' || control_mode == 'k')
				{
					fprintf(logger, "SetPt[mA]  I_m[mA] theta_l[deg]  theta_c[deg]  T_acc[N.m]  acc_hum[rad/s2]  acc_exo[rad/s2]  vel_hum[rad/s]  vel_exo[rad/s]  vel_motor[rad/s]\n");
				}
				if (control_mode == 's')
				{
					fprintf(logger, "acc_hum[rad/s2]  acc_exo[rad/s2]  vel_hum[rad/s]  vel_exo[rad/s]  vel_motor[rpm]  actual_Vel[rpm]  Voltage[V]\n");
				}
				fclose(logger);
			}
		}
		else
		{
			logging = false;
		}


		//		KALMAN FILTER SETUP		//

		x_k = Vector5f::Zero();
		z_k = Eigen::Vector3f::Zero();

		KG = Eigen::Matrix<float, 5, 3>::Zero();

		Fk = Matrix5f::Zero(5, 5);	// filling Fx matrix with zeros, once there are many zeros on it
		Fk(0, 0) = 1;	Fk(0, 1) = DELTA_T;
		Fk(1, 2) = -(B_EQ / (J_EQ + INERTIA_EXO));
		Fk(2, 2) = 1;	Fk(2, 3) = DELTA_T;
		Fk(3, 2) = -(B_EQ / (J_EQ + INERTIA_EXO));
		Fk(3, 4) = -(1 / (J_EQ + INERTIA_EXO));
		Fk(4, 0) = -STIFFNESS*DELTA_T; Fk(4, 2) = STIFFNESS*DELTA_T; Fk(4, 4) = 1;

		Bk = Vector5f::Zero(5, 1);
		Bk(1, 0) = GEAR_RATIO / (J_EQ + INERTIA_EXO);
		Bk(3, 0) = GEAR_RATIO / (J_EQ + INERTIA_EXO);

		Pk = 0.05 * Matrix5f::Identity();		// ???
		Qk = 0.1 * Matrix5f::Identity();		// ???

		Hk = Eigen::Matrix<float, 3, 5>::Zero();
		Hk(0, 0) = 1;	Hk(1, 2) = 1;	Hk(2, 4) = 1;
		Rk = 0.01 * Eigen::Matrix3f::Identity();
		//Rk(1, 0) = 2 * RATE * 0.01;		// error propagation from vel_hum to acc_hum
		//Rk(3, 2) = 2 * RATE * 0.01;		// error propagation from vel_exo to acc_exo

	}

	// Savitsky-Golay Smoothing and First Derivative based on the last 11 points
	void savitskygolay(float window[], float newest_value, float* first_derivative);

	// numdiff, controlling through the EPOS current control
	void FiniteDiff(float velHum, float velExo);

	// acc-gravity
	//void Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z);

	// Controlling through the EPOS motor speed control
	void OmegaControl(float velHum, float velExo);

	// Controlling through the EPOS current control using Kalman Filter for state estimation
	void CurrentControlKF(float velHum, float velExo);

	void GainScan_Current();

	void GainScan_CurrentKF();

	void GainScan_Velocity();

	void Recorder_Current();

	void Recorder_Velocity();

	void UpdateCtrlWord_Current();

	void UpdateCtrlWord_CurrentKF();

	void UpdateCtrlWord_Velocity();

	// destructor
	~accBasedControl()
	{
		m_eixo_in->PDOsetCurrentSetpoint(0);
		m_eixo_in->WritePDO01();
	}

	std::string ctrl_word;

private:

	EPOS_NETWORK* m_epos;
	AXIS* m_eixo_in;
	AXIS* m_eixo_out;

	FILE* logger;
	char logger_filename[40];
	bool logging;

	FILE* gains_values;

	int pos0_out;
	int pos0_in;

	//		GAINS		//
	// Current Control
	int Amplifier;
	float K_ff;
	float Kp_A;
	float Ki_A;
	float Kp_F;
	float Kd_F;

	// Speed Control
	int Amp_V;          // [dimensionless]
	float Kff_V;        // [dimensionless]
	float Kp_V;         // [dimensionless]
	float Ki_V;         // [1/s]
	float Kd_V;         // [s]

	//	 STATE VARIABLES	//

	float acc_hum;			// [rad/s^2]
	float acc_exo;			// [rad/s^2]
	float vel_hum;			// [rad/s]
	float vel_exo;			// [rad/s]

	float theta_l;			// [rad]
	float theta_c;			// [rad]

	//		STATE VECTORS				//

	float velhumVec[SGVECT_SIZE];		// [rad/s]
	float velexoVec[SGVECT_SIZE];		// [rad/s]
	float torqueSeaVec[SGVECT_SIZE];	// [N.m]
	float torqueAccVec[SGVECT_SIZE];	// [N.m]
	float theta_l_vec[SGVECT_SIZE];		// [rad]
	float theta_c_vec[SGVECT_SIZE];		// [rad]


	float setpoint;			// [mA]
	float setpoint_filt;	// [mA]


	float torque_sea;		// [N.m]
	float d_torque_sea;		// [N.m/s]
	float accbased_comp;	// [N.m]
	float d_accbased_comp;	// [N.m/s]
	float grav_comp;		// [N.m]

	float vel_leg;			// [rpm]
	float vel_motor;		// [rpm]
	float voltage;			// [V]

	int actualCurrent;		// [mA]
	int actualVelocity;		// [rpm]

	//		KALMAN FILTER		//

	Vector5f x_k;			// State Vector
	Eigen::Vector3f z_k;	// Sensor reading Vector
	Matrix5f Pk;			// State Covariance Matrix
	Matrix5f Fk;			// Prediction Matrix
	Vector5f Bk;			// Control Matrix* (is a vector but called matrix)
	Matrix5f Qk;			// Process noise Covariance

	Eigen::Matrix<float, 3, 5> Hk;			// Sensor Expectations Matrix
	Eigen::Matrix3f Rk;						// Sensor noise Covariance
	Eigen::Matrix<float, 5, 3> KG;			// Kalman Gain Matrix

	float Amp_kf;

	std::string kf_error;

};

#endif // !CURRENT_CONTROL_H

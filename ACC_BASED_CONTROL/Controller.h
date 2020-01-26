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
#define		LPF_SMF         ( DELTA_T / (DELTA_T + 1 /(2*MY_PI*LPF_FC)) )    // Low Pass Filter Smoothing Factor

#define		STATE_DIM		5
#define		SENSOR_DIM		3

// Filtered Derivative using Gain and Feedback Integrator
#define		CUTOFF			1.6000



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

		switch (control_mode)
		{
		case 'c':
			m_eixo_in->VCS_SetOperationMode(CURRENT_MODE); // For FiniteDiff
			break;
		case 'k':
			m_eixo_in->VCS_SetOperationMode(CURRENT_MODE); // For CurrentControlKF
			break;
		case 's':
			m_eixo_in->VCS_SetOperationMode(VELOCITY_MODE); // For OmegaControl
			break;
		case 'p':
			m_eixo_in->VCS_SetOperationMode(POSITION_MODE); // For FFPosition 
			break;
		default:
			break;
		}

		// Current Control
		K_ff = K_FF;	Kp_A = KP_A;	Ki_A = KI_A;	Kp_F = KP_F;	Kd_F = KD_F;
		Amplifier = 100000; // initialized with a safe value

		// Speed Control
		Kff_V = 225.000;	Kp_V = 0.018;
		Ki_V = 0.000;	Kd_V = 0.000;
		Amp_V = 1;			// initialized with a safe value

		vel_motor = 0;
		vel_motor_filt = 0;

		// Position Control
		Amp_P = 0;	Kff_P = 0;	Kp_P = 0;	Ki_P = 0;	Kd_P = 0;

		vel_hum = 0;		vel_exo = 0;
		setpoint = 0;		setpoint_filt = 0;
		accbased_comp = 0;	torque_sea = 0;

		accHum_R = 0;	accHum_T = 0;
		accExo_R = 0;	accExo_T = 0;

		theta_m = 0;
	
		IntegratorHum = 0;
		IntegratorExo = 0;
		IntAccMotor = 0;
		IntAccHum = 0;
		IntAccExo = 0;
		diffCutoff = CUTOFF;
		
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
				// printing the header into the file first line
				if (control_mode == 'c' || control_mode == 'k')
				{
					fprintf(logger, "SetPt[mA]  I_m[mA] theta_l[deg]  theta_c[deg]  T_acc[N.m]  acc_hum[rad/s2]  acc_exo[rad/s2]  vel_hum[rad/s]  vel_exo[rad/s]  vel_motor[rad/s]\n");
				}
				if (control_mode == 's')
				{
					fprintf(logger, "acc_hum[rad/s2]  acc_exo[rad/s2]  vel_hum[rad/s]  vel_exo[rad/s]  vel_motor[rad/s]  acc_motor[rad/s2]  actual_Vel[rad/s]\n");
				}
				if (control_mode == 'p')
				{
					fprintf(logger, "acc_hum[rad/s2]  acc_exo[rad/s2]  vel_hum[rad/s]  vel_exo[rad/s]  theta_m[deg]  theta_c[deg]  theta_l[deg]\n");
				}
				fclose(logger);
			}
		}
		else
		{
			logging = false;
		}


		//		KALMAN FILTER SETUP		//

		Amp_kf = 1;

		x_k.setZero();
		z_k.setZero();

		KG.setZero();

		Fk.setZero();
		Fk(0, 0) = 1;	Fk(0, 1) = DELTA_T;
		Fk(1, 2) = -(B_EQ / (J_EQ + INERTIA_EXO));
		Fk(2, 2) = 1;	Fk(2, 3) = DELTA_T;
		Fk(3, 2) = -(B_EQ / (J_EQ + INERTIA_EXO));
		Fk(3, 4) = -(1 / (J_EQ + INERTIA_EXO));
		Fk(4, 0) = -STIFFNESS*DELTA_T; Fk(4, 2) = STIFFNESS*DELTA_T; Fk(4, 4) = 1;

		Bk.setZero();
		Bk(1, 0) = GEAR_RATIO / (J_EQ + INERTIA_EXO);
		Bk(3, 0) = GEAR_RATIO / (J_EQ + INERTIA_EXO);

		// How to define my uncertainties ?	//
		Pk.setZero();
		Pk(0, 0) = 0.00002; Pk(1, 1) = 0.010; Pk(2, 2) = 0.00002; Pk(3, 3) = 0.010; Pk(4, 4) = 0.050;

		Qk.setZero();
		Qk(0, 0) = 0.01; Qk(1, 1) = 0.0001; Qk(2, 2) = 0.01; Qk(3, 3) = 0.0001; Qk(4, 4) = 0.01;

		//Qk(4, 4) = 0.010; // mechanical slack in the knee joint

		//	----------------------------	//

		Hk.setZero();
		Hk(0, 0) = 1;	Hk(1, 2) = 1;	Hk(2, 4) = 1;
		Rk.setZero();
		Rk(0, 0) = 0.00002;	Rk(1, 1) = 0.00002;	Rk(2, 2) = 0.107;	// sensors noise covariances

	}

	// Savitsky-Golay Smoothing and First Derivative based on the last 11 points
	void savitskygolay(float window[], float newest_value, float* first_derivative);

	// numdiff, controlling through the EPOS current control
	void FiniteDiff(float velHum, float velExo);

	// Controlling through the EPOS motor speed control
	void OmegaControl(float velHum, float velExo);

	// Controlling through the EPOS current control using Kalman Filter
	void CurrentControlKF(float velHum, float velExo);

	// Controlling through the EPOS position control, assuming T_l = T_ff
	void FFPosition(float velHum, float velExo);

	// Controlling through the EPOS position control, using the IMUs acceleration with gravity:
	void accBasedControl::FFPositionGrav(float accHumT, float accHumR, float accExoT, float accExoR, float velHum, float velExo);
	
	void GainScan_Current();

	void GainScan_CurrentKF();

	void GainScan_Velocity();

	void GainScan_Position();

	void Recorder_Current();

	void Recorder_Velocity();

	void Recorder_Position();

	void UpdateCtrlWord_Current();

	void UpdateCtrlWord_CurrentKF();

	void UpdateCtrlWord_Velocity();

	void UpdateCtrlWord_Position();

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
	float Amp_V;          // [dimensionless]
	float Kff_V;        // [dimensionless]
	float Kp_V;         // [dimensionless]
	float Ki_V;         // [1/s]
	float Kd_V;         // [s]

	// Position Control

	float Amp_P;          // [dimensionless]
	float Kff_P;        // [dimensionless]
	float Kp_P;         // []
	float Ki_P;         // []
	float Kd_P;         // []

	//	 STATE VARIABLES	//

	float acc_hum;			// [rad/s^2]
	float acc_exo;			// [rad/s^2]
	float vel_hum;			// [rad/s]
	float vel_exo;			// [rad/s]
	float jerk_hum;			// [rad/s^3]
	float jerk_exo;			// [rad/s^3]

	float theta_l;			// [rad]
	float theta_c;			// [rad]

	//		STATE MEMORY VECTORS        //

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

	float accHum_R;			// [m/s²]
	float accHum_T;			// [m/s²]
	float accExo_R;			// [m/s²]
	float accExo_T;			// [m/s²]

	float vel_leg;			// [rpm]
	float acc_motor;		// [rad/s^2]
	float vel_motor;		// [rad/s]
	float vel_motor_filt;
	float voltage;			// [V]

	float theta_m;       // [encoder pulses]

	int actualCurrent;		// [mA]
	int actualVelocity;		// [rpm]
	
	float diffCutoff;
	float IntegratorHum;
	float IntegratorExo;
	float IntAccMotor;
	float IntAccHum;
	float IntAccExo;

	//		KALMAN FILTER		//

	Eigen::Matrix<float, STATE_DIM, 1> x_k;			// State Vector
	Eigen::Matrix<float, SENSOR_DIM, 1> z_k;		// Sensor reading Vector
	Eigen::Matrix<float, STATE_DIM, STATE_DIM> Pk;			// State Covariance Matrix
	Eigen::Matrix<float, STATE_DIM, STATE_DIM> Fk;			// Prediction Matrix
	Eigen::Matrix<float, STATE_DIM, 1> Bk;					// Control Matrix* (is a vector but called matrix)
	Eigen::Matrix<float, STATE_DIM, STATE_DIM> Qk;			// Process noise Covariance
	Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM> Rk;		// Sensor noise Covariance
	Eigen::Matrix<float, SENSOR_DIM, STATE_DIM> Hk;			// Sensor Expectations Matrix
	Eigen::Matrix<float, STATE_DIM, SENSOR_DIM> KG;			// Kalman Gain Matrix

	float Amp_kf;

	std::string kf_error;

};

#endif // !CURRENT_CONTROL_H

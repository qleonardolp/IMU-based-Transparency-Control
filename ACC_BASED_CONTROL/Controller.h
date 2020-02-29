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

#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <Eigen/LU>
#include <Eigen/Core>

#define		SGVECT_SIZE		11				// Size of the window vector for Savitsky-Golay smoothing and derivative

// CONSTANTS

#define		GEAR_RATIO		150.0f		    // Redução do Sistema
#define		ENCODER_IN		4096		    // Resolução do encoder do motor
#define		ENCODER_OUT		2048		    // Resolução do encoder de saída
#define		STIFFNESS		104.0f			// Constante da mola SEA [N.m/rad]
#define		MY_PI			3.141592653f	// Pi value
#define		RADS2RPM		(30/MY_PI)		// rad/s to rpm 
#define		RPM2RADS		(MY_PI/30)		// rpm to rad/s

//	Torque Constant: 0.0603 N.m/A
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A

#define		CURRENT_MAX		3.1400f		// Max corrente nominal no motor Maxon RE40 [A]
#define		VOLTAGE_MAX		21.600f		// Max tensão de saída Vcc = 0.9*24V fornecida pela EPOS 24/5
#define		TORQUE_CONST	0.0603f		// Constante de torque do motor RE40	[N.m/A]
#define		SPEED_CONST		158.00f		// Constante de velocidade do motor RE40 [rpm/V]

#define     GRAVITY         9.8066f		// [m/s^2]
#define     INERTIA_EXO     0.2620f		// [Kg.m^2], 0.0655 +- 0.0006, estimado em 2019-08-21, estou superdimensionando para 4x
#define		LOWERLEGMASS	3.6480f		// [Kg] Pesar no labDim para saber valor real!
#define		MTW_DIST_LIMB	0.2500f		// [m]
#define		MTW_DIST_EXO	0.0700f		// [m]
#define		L_CG			0.4500f		// [m]

// According to W. M. Dos Santos and A. A. G. Siqueira in 10.1109/BIOROB.2014.6913851 (DOI)
#define		J_EQ			0.4700f		// [Kg.m^2]
#define		B_EQ			60.000f		// [N.m s/rad]

// Feedforward-Feedback PI acc-based controller:
#define		K_FF			1.0000f		// [dimensionless]
#define     KP_A			4.5600f     // [Kg.m^2]
#define     KI_A			5.7900f     // [Kg.m^2/s]

// Feedback PD force (SEA) controller:
#define     KP_F			1.5310f     // [dimensionless]
#define     KD_F			0.0200f     // [s]


#define     RATE            125.0f		// [Hz]	Use the control loop rate running
#define		DELTA_T			(float) 1/RATE  // [s]

// -> Low Pass Filtering <- //
#define     LPF_FC          3.000f		// [Hz] Low Pass Filter Frequency Cutoff
// Low Pass Filter Smoothing Factor
#define		LPF_SMF         (float) ( DELTA_T / (DELTA_T + 1 /(2*MY_PI*LPF_FC)) )
// ------------------------ //

// Filtered Derivative using Gain and Feedback Integrator
#define		CUTOFF			20.000f

// High Pass Filter for Gyroscopes Bias
#define		HPF_FC			0.7f	// [Hz]
#define		HPF_SMF			(float) (1 / (2*MY_PI*HPF_FC*DELTA_T + 1) )

// Kalman Filter Dimensions
#define		STATE_DIM		5
#define		SENSOR_DIM		3

using namespace Eigen;

class accBasedControl
{
public:
	// Class Constructor
	accBasedControl(EPOS_NETWORK* epos, AXIS* eixo_in, AXIS* eixo_out, char control_mode, int seconds)
	{
		m_epos = epos;
		m_eixo_in = eixo_in;
		m_eixo_out = eixo_out;
		m_seconds = seconds;
		m_control_mode = control_mode;

		pos0_out = -m_eixo_out->PDOgetActualPosition();
		pos0_in = m_eixo_in->PDOgetActualPosition();

		switch (control_mode)
		{
		case 'c':
		case 'k':
		case 'u':
			m_eixo_in->VCS_SetOperationMode(CURRENT_MODE); // For FiniteDiff, CurrentControlKF, CACurrent
			break;
		case 's':
		case 'a':
			m_eixo_in->VCS_SetOperationMode(VELOCITY_MODE); // For OmegaControl or CAdmittanceControl
			break;
		default:
			break;
		}

		// Current Control
		K_ff = K_FF;	Kp_A = KP_A;	Ki_A = KI_A;	Kp_F = KP_F;	Kd_F = KD_F;
		Amplifier = 100; // initialized with a safe value

		for (size_t i = 0; i < SGVECT_SIZE; ++i)
		{
			velhumVec[i] = 0.000f; velexoVec[i] = 0.000f;
			theta_l_vec[i] = 0.000f; theta_c_vec[i] = 0.000f;
			torqueSeaVec[i] = 0.000f; torqueAccVec[i] = 0.000f;
		}

		if (seconds > 0)
		{

			time_t rawtime;
			struct tm* timeinfo;
			time(&rawtime);
			timeinfo = localtime(&rawtime);

			strftime(logger_filename, 40, "./data/%Y-%m-%d-%H-%M-%S.txt", timeinfo);
			logger = fopen(logger_filename, "wt");
			if (logger != NULL)
			{
				// printing the header into the file first line
				if (control_mode == 's')
				{
					fprintf(logger, "time[s]  vel_hum[rad/s]  vel_exo[rad/s]  acc_hum[rad/s2]  acc_exo[rad/s2]  jerk_hum[rad/s3]  jerk_exo[rad/s3]  vel_motor[rad/s]  actual_Vel[rad/s]  exo_Vel[rad/s]  T_Sea[N.m]  theta_c[rad]  theta_l[rad]\n");
				}
				else if (control_mode == 'a')
				{
					fprintf(logger, "time[s]  vel_hum[rad/s]  vel_adm[rad/s]  vel_motor[rad/s]  actual_Vel[rad/s]  T_Sea[N.m]\n");
				}
				else if (control_mode == 'u')
				{
					fprintf(logger, "time[s]  vel_hum[rad/s]  vel_adm[rad/s]  vel_motor[rad/s]  SetPt[mA]  I_m[mA]  T_Sea[N.m]\n");
				}
				fclose(logger);
			}
		}

		//		KALMAN FILTER SETUP		//
		float Dt = 0.001f;

		if (control_mode == 's'){}
		else if (control_mode == 'k')
		{	
			CACu_xk.setZero();	CACu_zk.setZero(); CACu_KG.setZero();

			CACu_Fk.setZero();
			// Row 1
			CACu_Fk(0, 0) = 1; 
			// Row 2
			CACu_Fk(1, 1) = damping_A / (damping_A + stiffness_d*Dt);
			CACu_Fk(1, 6) = -(1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_A / Dt);
			// Row 3
			CACu_Fk(2, 2) = 1;
			// Row 4
			CACu_Fk(3, 2) = Dt; CACu_Fk(3, 3) = 1;
			// Row 5
			CACu_Fk(4, 4) = 1;
			// Row 6
			CACu_Fk(5, 3) = STIFFNESS; CACu_Fk(5, 4) = -STIFFNESS; CACu_Fk(5, 6) = Dt;
			// Row 7
			CACu_Fk(6, 6) = 1;

			CACu_Bk.setZero();
			CACu_Bk(2, 0) = Dt; CACu_Bk(3, 0) = 0.500f*Dt*Dt;

			CACu_Hk.setZero();
			CACu_Hk(0, 0) = 1;	CACu_Hk(1, 2) = 1;	CACu_Hk(2, 3) = 1;	CACu_Hk(3, 4) = 1;

			CACu_Pk.setZero();
			CACu_Pk(0, 0) = 0.0002;	CACu_Pk(1, 1) = 0.0009;	CACu_Pk(2, 2) = 0.0004;	CACu_Pk(3, 3) = 0.0002;	
			CACu_Pk(4, 4) = 0.0001;	CACu_Pk(5, 5) = STIFFNESS*0.0003; CACu_Pk(6, 6) = STIFFNESS*0.0010;

			CACu_Qk.setIdentity();
			CACu_Qk = 0.001*CACu_Qk;

			CACu_Rk.setZero();
			CACu_Rk(0, 0) = 0.002;	CACu_Rk(1, 1) = 0.005;	CACu_Rk(2, 2) = 0.001;	CACu_Rk(3, 3) = 0.001;
		}
		/*
		Amp_kf = 1;
		x_k.setZero(); z_k.setZero(); KG.setZero();

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

		Hk.setZero();
		Hk(0, 0) = 1;	Hk(1, 2) = 1;	Hk(2, 4) = 1;
		Rk.setZero();
		Rk(0, 0) = 0.00002;	Rk(1, 1) = 0.00002;	Rk(2, 2) = 0.107;	// sensors noise covariances
		*/
	}

	// numdiff, controlling through the EPOS current control (Deprecated)
	void FiniteDiff(float &velHum, float &velExo, std::condition_variable &cv, std::mutex &m){}

	// Controlling through the EPOS motor speed control
	void OmegaControl(float &velHum, float &velExo, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin);

	// Controlling through the EPOS motor speed control assisted by Kalman Filter estimation
	void OmegaControlKF(float &velHum, float &velExo, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin);

	// Collocated Admittance Controller using q' and tau_e, according to A. Calanca, R. Muradore and P. Fiorini
	void CAdmittanceControl(float &velHum, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin);

	// Collocated Admittance Controller using q and tau_e and tau_m
	void CACurrent(float &velHum, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin);

	// Collocated Admittance Controller using q and tau_e and tau_m
	void CACurrentKF(float &velHum, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin);

	// Savitsky-Golay Smoothing and First Derivative based on the last 11 points
	void SavitskyGolay(float window[], float newest_value, float* first_derivative);

	// GainScan are the methods to scan the file with the values to update the gains in runtime
	void GainScan_Current();
	void GainScan_CurrentKF();
	void GainScan_Velocity();
	void GainScan_CAC();
	void GainScan_CACu();

	// Recorder are the methods to log the desired variables in a .txt file
	void Recorder_Current();
	void Recorder_Velocity();
	void Recorder_CAC();
	void Recorder_CACu();

	// Update the 'Control Word' to show info at the console screen
	void UpdateCtrlWord_Current();
	void UpdateCtrlWord_CurrentKF();
	void UpdateCtrlWord_Velocity();
	void UpdateCtrlWord_Admittance();

	// Stop the control_t thread loop, allowing .join at the main
	void StopCtrlThread(){ Run = false; }

	// Class Destructor
	~accBasedControl()
	{
		switch (m_control_mode)
		{
		case 'c': case 'k': case 'u':
			m_eixo_in->PDOsetCurrentSetpoint(0);
			m_eixo_in->WritePDO01();
			break;
		case 's': case 'a':
			m_eixo_in->PDOsetVelocitySetpoint(0);
			m_eixo_in->WritePDO02();
			break;
		default:
			break;
		}
	}

	std::string ctrl_word;

private:

	static EPOS_NETWORK* m_epos;
	static AXIS* m_eixo_in;
	static AXIS* m_eixo_out;
	static float m_seconds;
	char m_control_mode;
	static int pos0_out;
	static int pos0_in;

	FILE* logger;
	char logger_filename[40];
	static float timestamp;

	FILE* gains_values;

	static std::atomic<bool> Run;
	static float control_t_Dt;
	static std::chrono::system_clock::time_point control_t_begin;

	//		GAINS		//
	// Current Control
	int Amplifier; float K_ff; float Kp_A; float Ki_A; float Kp_F; float Kd_F;

	// Speed Control
	static float Kff_V; static float Kp_V; 
	static float Ki_V; static float Kd_V;

	// |-> Admittance Control <---
	// |
	// |- Inner Control:
	static float Ki_adm;		// in the reference is the P [N.m s/rad]
	static float Kp_adm;		// in the reference is the D [N.m/rad]
	static float torque_m;		// output from 'Cp(s)', actually a Cv(s) controller
	static float IntInnerC;		// Integrator of the input in Cv(s)
	static float IntTorqueM;	// Integrator of the Torque to the Motor, torque_m -> 1/(J_EQ*s) -> vel_motor
	static int   resetInt;
	// |- Admittance Control:
	static float torque_ref;	// Reference Torque for A(s).  ? Compensate the lower leg exo mass and dynamics ?
	static float Adm_In;		// A(s) Input Derivative = d/dt(torque_ref - torque_sea)
	static float IntAdm_In;		// A(s) Input integrator
	static float stiffness_d;	// k_d	  [N.m/rad]
	static float damping_A;		// d_{dm} [N.m s/rad]
	static float vel_adm;		// [rad/s] output from A(s), the velocity required to guarantee the desired Admittance
	static float vel_adm_last;	// [rad/s]
	static float kd_min;
	static float kd_max;
	// |-------------------------


	//	 STATE VARIABLES	//

	static float vel_hum;
	static float vel_exo;
	static float acc_hum;
	static float acc_exo;
	static float jerk_hum;
	static float jerk_exo;
	static float theta_l;
	static float theta_c;

	static float vel_hum_last;
	static float vel_exo_last;

	//	Aux Variables		//
	static float setpoint;			// [A]
	static float setpoint_filt;		// [A]
	static int   actualCurrent;		// [mA]

	static float torque_sea;		// [N.m]
	static float d_torque_sea;		// [N.m/s]
	static float accbased_comp;		// [N.m]
	static float d_accbased_comp;	// [N.m/s]
	static float grav_comp;			// [N.m]

	static float vel_leg;			// [rpm ?]
	static float acc_motor;			// [rad/s^2]
	static float vel_motor;			// [rad/s]
	static float vel_motor_filt;	// [rad/s]
	static float voltage;			// [V]

	static float theta_m;			// [encoder pulses]
	static int actualVelocity;		// [rpm]
	static int exoVelocity;			// [rpm]

	static float diffCutoff;
	static float IntegratorHum;
	static float IntegratorExo;
	static float IntAccMotor;
	static float IntAccHum;
	static float IntAccExo;

	//		STATE MEMORY VECTORS        //

	float velhumVec[SGVECT_SIZE];		// [rad/s]
	float velexoVec[SGVECT_SIZE];		// [rad/s]
	float torqueSeaVec[SGVECT_SIZE];	// [N.m]
	float torqueAccVec[SGVECT_SIZE];	// [N.m]
	float theta_l_vec[SGVECT_SIZE];		// [rad]
	float theta_c_vec[SGVECT_SIZE];		// [rad]

	//		KALMAN FILTER				//
	/*
	float Amp_kf;
	std::string kf_error;
	Eigen::Matrix<float, STATE_DIM, 1> x_k;					// State Vector
	Eigen::Matrix<float, SENSOR_DIM, 1> z_k;				// Sensor reading Vector
	Eigen::Matrix<float, STATE_DIM, STATE_DIM> Pk;			// State Covariance Matrix
	Eigen::Matrix<float, STATE_DIM, STATE_DIM> Fk;			// Prediction Matrix
	Eigen::Matrix<float, STATE_DIM, 1> Bk;					// Control Matrix* (is a vector but called matrix)
	Eigen::Matrix<float, STATE_DIM, STATE_DIM> Qk;			// Process noise Covariance
	Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM> Rk;		// Sensor noise Covariance
	Eigen::Matrix<float, SENSOR_DIM, STATE_DIM> Hk;			// Sensor Expectations Matrix
	Eigen::Matrix<float, STATE_DIM, SENSOR_DIM> KG;			// Kalman Gain Matrix
	*/

	//		OmegaControl KF				//

	static Matrix<float, 9, 1> Ome_xk;	// State Vector				[vel_h acc_h jerk_h vel_e acc_e jerk_e vel_m theta_c theta_l]
	static Matrix<float, 5, 1> Ome_zk;	// Sensor reading Vector	[vel_hum vel_exo vel_motor theta_c theta_l]
	static Matrix<float, 9, 9> Ome_Pk;	// State Covariance Matrix
	static Matrix<float, 9, 9> Ome_Fk;	// Prediction Matrix
	static Matrix<float, 9, 1> Ome_Bk;	// Control Matrix (is a vector but called matrix)
	static Matrix<float, 9, 9> Ome_Qk;	// Process noise Covariance
	static Matrix<float, 5, 5> Ome_Rk;	// Sensor noise Covariance
	static Matrix<float, 5, 9> Ome_Hk;	// Sensor Expectations Matrix
	static Matrix<float, 9, 5> Ome_KG;	// Kalman Gain Matrix

	//		CACu Kalman Filter			//

	static Matrix<float, 7, 1> CACu_xk;	// State Vector				[vel_hum vel_adm vel_motor theta_c theta_l torque_sea d_torque_sea]
	static Matrix<float, 4, 1> CACu_zk;	// Sensor reading Vector	[vel_hum vel_motor theta_c theta_l]
	static Matrix<float, 7, 7> CACu_Pk;	// State Covariance Matrix
	static Matrix<float, 7, 7> CACu_Fk;	// Prediction Matrix
	static Matrix<float, 7, 1> CACu_Bk;	// Control Matrix (is a vector but called matrix)
	static Matrix<float, 7, 7> CACu_Qk;	// Process noise Covariance
	static Matrix<float, 4, 4> CACu_Rk;	// Sensor noise Covariance
	static Matrix<float, 4, 7> CACu_Hk;	// Sensor Expectations Matrix
	static Matrix<float, 7, 4> CACu_KG;	// Kalman Gain Matrix
};

// Speed Control [s]
float accBasedControl::Kp_V = 0;
float accBasedControl::Ki_V = 0;
float accBasedControl::Kd_V = 0;
float accBasedControl::Kff_V = 0;

//		CACu Kalman Filter						//
Matrix<float, 7, 1> accBasedControl::CACu_xk;	// State Vector				[vel_hum vel_adm vel_motor theta_c theta_l torque_sea d_torque_sea]
Matrix<float, 4, 1> accBasedControl::CACu_zk;	// Sensor reading Vector	[vel_hum vel_motor theta_c theta_l]
Matrix<float, 7, 7> accBasedControl::CACu_Pk;	// State Covariance Matrix
Matrix<float, 7, 7> accBasedControl::CACu_Fk;	// Prediction Matrix
Matrix<float, 7, 1> accBasedControl::CACu_Bk;	// Control Matrix (is a vector but called matrix)
Matrix<float, 7, 7> accBasedControl::CACu_Qk;	// Process noise Covariance
Matrix<float, 4, 4> accBasedControl::CACu_Rk;	// Sensor noise Covariance
Matrix<float, 4, 7> accBasedControl::CACu_Hk;	// Sensor Expectations Matrix
Matrix<float, 7, 4> accBasedControl::CACu_KG;	// Kalman Gain Matrix

#endif // !CURRENT_CONTROL_H

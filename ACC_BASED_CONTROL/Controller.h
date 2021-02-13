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

#define	UDP_ENABLE		(true)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>

#pragma comment(lib, "Ws2_32.lib")

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <stdio.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <Eigen/LU>
#include <Eigen/Core>


#if UDP_ENABLE
#define 	UDP_PORT 		"2324"
#endif

#define		SGVECT_SIZE		11				// Size of the window vector for Savitsky-Golay smoothing and derivative

// CONSTANTS

#define		GEAR_RATIO		150.0f		    // Redu��o do Sistema
#define		ENCODER_IN		4096		    // Resolu��o do encoder do motor
#define		ENCODER_OUT		2048		    // Resolu��o do encoder de sa�da
#define		STIFFNESS		104.0f			// Constante da mola SEA [N.m/rad]
#define		MY_PI			3.141592653f	// Pi value
#define		RADS2RPM		(30/MY_PI)		// rad/s to rpm 
#define		RPM2RADS		(MY_PI/30)		// rpm to rad/s

//	Torque Constant: 0.0603 N.m/A
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A

#define		CURRENT_MAX		3.1400f		// Max corrente nominal no motor Maxon RE40 [A]
#define		VOLTAGE_MAX		21.600f		// Max tens�o de sa�da Vcc = 0.9*24V fornecida pela EPOS 24/5
#define		TORQUE_CONST	0.0603f		// Constante de torque do motor RE40	[N.m/A]
#define		SPEED_CONST		158.00f		// Constante de velocidade do motor RE40 [rpm/V]

#define     GRAVITY         9.8066f		// [m/s^2]
#define     INERTIA_EXO     0.2620f		// [Kg.m^2], 0.0655 +- 0.0006, estimado em 2019-08-21, estou superdimensionando para 4x
#define		LOWERLEGMASS	4.7421f		// [Kg] Definido pelo fit usando o torque SEA em 2020-12-19, ver exo_mass_measurement.m para mais detalhes
#define		MTW_DIST_LIMB	0.2500f		// [m]
#define		MTW_DIST_EXO	0.0700f		// [m]
#define		L_CG			0.4320f		// [m]

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

#define     C_RATE    1000.0f
#define     C_DT      (float) 1/C_RATE

#define     RATE            125.0f		// [Hz]	IMUs update rate
#define		DELTA_T			(float) 1/RATE  	// [s]

// -> Low Pass Filtering <- //
#define     LPF_FC          5.000f		// [Hz] Low Pass Filter Frequency Cutoff
// Low Pass Filter Smoothing Factor
#define		LPF_SMF         (float) ( DELTA_T / (DELTA_T + 1 /(2*MY_PI*LPF_FC)) )
// ------------------------ //

// Filtered Derivative using Gain and Feedback Integrator
#define		CUTOFF			20.000f

// High Pass Filter for Gyroscopes Bias
#define		HPF_FC			0.7f	// [Hz]
#define		HPF_SMF			(float) (1 / (2*MY_PI*HPF_FC*DELTA_T + 1) )

// EKF dimensions
#define EKF_STATE_DIM  5
#define EKF_SENSOR_DIM 5
#define EKF_CTRL_DIM   2
#define IMU_DELAY 	   8				// N Samples of Ts=0.001s

using namespace Eigen;

typedef Matrix<float, EKF_STATE_DIM, EKF_STATE_DIM> StateSzMtx;
typedef Matrix<float, EKF_SENSOR_DIM, EKF_SENSOR_DIM> SensorSzMtx;


class accBasedControl
{
public:
	// Class Constructor
	accBasedControl(EPOS_NETWORK* epos, AXIS* eixo_in, AXIS* eixo_out, char control_mode, int seconds) : 
		m_control_mode(control_mode)
	{
		m_epos = epos;
		m_eixo_in = eixo_in;
		m_eixo_out = eixo_out;
		m_seconds = seconds;

#if UDP_ENABLE
		// Initialize Winsock
		ListenSocket = INVALID_SOCKET;
		int iResult;
		struct addrinfo *result = NULL, *ptr = NULL, hints;
		iResult = WSAStartup(MAKEWORD(2,2), &wsaData);

		if (iResult != 0) {
    		printf("WSAStartup failed: %d\n", iResult);
		} else {

			ZeroMemory(&hints, sizeof (hints));
			hints.ai_family = AF_INET;
			hints.ai_socktype = SOCK_STREAM;
			hints.ai_protocol = IPPROTO_UDP;
			hints.ai_flags = AI_PASSIVE;

			// Resolve the local address and port to be used by the server
			iResult = getaddrinfo(NULL, UDP_PORT, &hints, &result);
			if (iResult != 0) {
				printf("getaddrinfo failed: %d\n", iResult);
				WSACleanup();
			} else {
				// Create a SOCKET for the server to listen for client connections
				ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
				if (ListenSocket == INVALID_SOCKET) {
					printf("Error at socket(): %ld\n", WSAGetLastError());
					freeaddrinfo(result);
					WSACleanup();
				} else {
					// Setup the TCP listening socket
					iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
					if (iResult == SOCKET_ERROR) {
						printf("bind failed with error: %d\n", WSAGetLastError());
						freeaddrinfo(result);
						closesocket(ListenSocket);
						WSACleanup();
					} else {
						freeaddrinfo(result);
						
						if ( listen( ListenSocket, SOMAXCONN ) == SOCKET_ERROR ) {
							printf( "Listen failed with error: %ld\n", WSAGetLastError() );
							closesocket(ListenSocket);
							WSACleanup();
						} else {
							ClientSocket = INVALID_SOCKET;
							// Accept a client socket
							ClientSocket = accept(ListenSocket, NULL, NULL);
							if (ClientSocket == INVALID_SOCKET) {
								printf("accept failed: %d\n", WSAGetLastError());
								closesocket(ListenSocket);
								WSACleanup();
							}
						}
					}
				}
			}
		}
#endif

		pos0_out = -m_eixo_out->PDOgetActualPosition();
		pos0_in = m_eixo_in->PDOgetActualPosition();

		switch (control_mode)
		{
		case 'p':
			m_eixo_in->VCS_SetOperationMode(POSITION_MODE);	// For accBasedPosition
		case 'k':
		case 'u':
			m_eixo_in->VCS_SetOperationMode(CURRENT_MODE); // For CACurrent, CACurrentKF
			break;
		case 's':
		case 'a':
			m_eixo_in->VCS_SetOperationMode(VELOCITY_MODE); // For OmegaControl or CAdmittanceControl
			break;
		default:
			break;
		}

		for (size_t i = 0; i < SGVECT_SIZE; ++i)
		{
			velhumVec[i] = 0.000f; velexoVec[i] = 0.000f;
			theta_l_vec[i] = 0.000f; theta_c_vec[i] = 0.000f;
			torqueSeaVec[i] = 0.000f; torqueAccVec[i] = 0.000f;
		}

		if (seconds > 0)
		{
			timestamp_begin = std::chrono::steady_clock::now();
			logging = true;
			time_t rawtime;
			struct tm* timeinfo;
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			strftime(header_timestamp, sizeof(header_timestamp), "%Y-%m-%d-%H-%M-%S", timeinfo);
			char logfilename[40];
			strftime(logfilename, sizeof(logfilename), "./data/%Y-%m-%d-%H-%M-%S.txt", timeinfo);
      		strcpy(logger_filename, logfilename);

			logger = fopen(getLogfilename(), "wt");
			if (logger != NULL)
			{
				// printing the header into the file first line
				if (control_mode == 'p'){
					fprintf(logger, "accBasedPosition [%s]\ntime[s]  acc_hum[rad / s2]  vel_hum[rad / s]  vel_exo[rad / s]  T_Sea[N.m]  theta_m[rad]\n", header_timestamp);
				}
				else if (control_mode == 's'){
					fprintf(logger, "CAdmittanceControl [%s]\ntime[s]  acc_hum[rad/s2]  vel_hum[rad/s]  vel_exo[rad/s]  vel_adm[rad/s]  theta_c[rad]  theta_l[rad]  InvDyn[N.m]  AccBsd[N.m]  vel_motor[rad/s]\n", header_timestamp);
				}
				else if (control_mode == 'a'){
					fprintf(logger, "CAdmittanceControlKF [%s]\ntime[s]  vel_hum[rad/s]  vel_adm[rad/s]  vel_motor[rad/s]  T_Sea[N.m]\n", header_timestamp);
				}
				else if (control_mode == 'u'){
					fprintf(logger, "CACurrent [%s]\ntime[s]  vel_hum[rad/s]  vel_adm[rad/s]  vel_motor[rad/s]  SetPt[mA]  I_m[mA]  T_Sea[N.m]  dT_Sea[N.m/s]\n", header_timestamp);
				}
				else if (control_mode == 'k'){
					fprintf(logger, "CACurrentKF [%s]\ntime[s]  vel_hum[rad/s]  vel_adm[rad/s]  vel_motor[rad/s]  SetPt[mA]  I_m[mA]  T_Sea[N.m]\n", header_timestamp);
				}
				fclose(logger);
			}
		}

		//		KALMAN FILTER SETUP		//
		float Dt = 0.0010f;
		CAC_xk.setZero();
		CAC_zk.setZero();
		CAC_KG.setZero();

		CAC_Fk.setZero();
		// Row 1
		CAC_Fk(0, 0) = 1;
		// Row 2
		CAC_Fk(1, 1) = 1;
		// Row 3
		CAC_Fk(2, 1) = Dt;
		CAC_Fk(2, 2) = 1;
		// Row 4
		CAC_Fk(3, 3) = 1;
		// Row 5
		CAC_Fk(4, 2) = STIFFNESS;
		CAC_Fk(4, 3) = -STIFFNESS;

		CAC_Bk.setZero();
		CAC_Bk(1, 0) = Dt;
		CAC_Bk(2, 0) = 0.500f * Dt * Dt;

		CAC_Hk.setZero();
		CAC_Hk(0, 0) = 1;
		CAC_Hk(1, 1) = 1;
		CAC_Hk(2, 2) = 1;
		CAC_Hk(3, 3) = 1;

		CAC_Pk.setZero();
		CAC_Pk(0, 0) = pow(0.0023400, 2); // devpad = devpad(vel_hum measured*)
		CAC_Pk(1, 1) = pow(0.0059438, 2); // devpad = devpad(vel_motor) + Dt*devpad(torque_m/J_EQ)
		CAC_Pk(2, 2) = pow(0.0000154, 2); // devpad = devpad(theta_c) + Dt*devpad(vel_motor) + 0.5*Dt^2*devpad(torque_m/J_EQ)
		CAC_Pk(3, 3) = pow(0.0030700, 2); // 2*pi/2048
		CAC_Pk(4, 4) = pow(0.3203400, 2); // devpad = Ksea*(devpad(theta_c) + devpad(theta_l))

		CAC_Rk.setZero();
		CAC_Rk(0, 0) = pow(0.0023400, 2); // MTw Noise x sqrt(Bandwidth) in rad/s, check MTw Technical Specs
		CAC_Rk(1, 1) = pow(0.0048800, 2); // Considering 7 rpm devpad: 7 * RPM2RADS / GEAR_RATIO
		CAC_Rk(2, 2) = pow(0.0000100, 2); // 2*pi/(4096*GEAR_RATIO)
		CAC_Rk(3, 3) = pow(0.0030700, 2); // 2*pi/2048

		// additional uncertainty from the environment
		CAC_Qk.setZero();
		CAC_Qk(0, 0) = pow(0.0000234, 2);
		CAC_Qk(1, 1) = pow(0.0010638, 2); // Dt*devpad(torque_m/J_EQ)
		CAC_Qk(2, 2) = pow(0.0000054, 2); // Dt*devpad(vel_motor) + 0.5*Dt^2*devpad(torque_m/J_EQ)
		CAC_Qk(3, 3) = pow(0.0000307, 2);
		CAC_Qk(4, 4) = pow(0.0032034, 2);

		//		EKF SETUP		//
		int_stiffness = STIFFNESS/50;
		theta_l = 0;

		ekf_xk.setZero();
		ekf_uk.setZero();
		ekf_zk.setZero();
		ekf_KG.setZero();

		ekf_Gk.setZero();
		ekf_Gk(1, 2) = 1;
		ekf_Gk(2, 1) = -(int_stiffness + STIFFNESS - LOWERLEGMASS*GRAVITY*L_CG*cos(theta_l))/INERTIA_EXO;
		ekf_Gk(2, 3) = STIFFNESS/INERTIA_EXO;
		ekf_Gk(3, EKF_STATE_DIM-1) = 1;
		ekf_Gk(EKF_STATE_DIM-1, 1)  = STIFFNESS/J_EQ;
		ekf_Gk(EKF_STATE_DIM-1, 3)  = -STIFFNESS/J_EQ;
		ekf_Gk(EKF_STATE_DIM-1, EKF_STATE_DIM-1)  = -B_EQ/J_EQ;

		ekf_Bk.setZero();
		ekf_Bk(0, 0) = 1;
		ekf_Bk(EKF_STATE_DIM-1, EKF_CTRL_DIM-1) = GEAR_RATIO*TORQUE_CONST/J_EQ;

		ekf_Hk.setZero();
		ekf_Hk(1, 1) = 1;
		ekf_Hk(2, 2) = 1;
		ekf_Hk(3, 3) = GEAR_RATIO;
		ekf_Hk(EKF_SENSOR_DIM-1, EKF_STATE_DIM-1) = GEAR_RATIO;
		
		ekf_Dk.setZero();
		ekf_Dk(0, 0) = 1;

		ekf_Pk = 0.01 * Matrix<float,EKF_STATE_DIM,EKF_STATE_DIM>::Identity();

		ekf_Rk.setZero();
		ekf_Rk(0, 0) = pow(0.0023400, 2); // MTw Noise x sqrt(Bandwidth) in rad/s, check MTw Technical Specs
		ekf_Rk(1, 1) = pow(0.0030700, 2); // 2*pi/2048
		ekf_Rk(2, 2) = pow(0.0023400, 2); // MTw Noise x sqrt(Bandwidth) in rad/s, check MTw Technical Specs
		ekf_Rk(3, 3) = pow(0.0015339, 2); // 2*pi/4096
		ekf_Rk(EKF_SENSOR_DIM-1, EKF_SENSOR_DIM-1) = pow(0.1047197, 2); // Considering 1 rpm devpad: 1 * RPM2RADS

		ekf_Qk.setZero();
		ekf_Qk(0, 0) = pow(0.0000234, 2);
		ekf_Qk(1, 1) = pow(0.0000234, 2);
		ekf_Qk(2, 2) = pow(0.0032000, 2); // ????
		ekf_Qk(3, 3) = pow(0.0000766, 2); // 2*pi/4096 (5%)
		ekf_Qk(EKF_STATE_DIM-1, EKF_STATE_DIM-1) = pow(0.0032034, 2); // ???
	}

	// EKF main loop
	void ekfUpdate(float velHum, float posExo, float velExo, float posAct, float velAct, float mCurrent);

	// EKF Log
	void ekfLogger();

	// Controlling tau_m using the Human acceleration feedforward
	void accBasedController(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m);

	// Controlling through the EPOS Position control
	void accBasedPosition(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m);

	// Controlling through the EPOS motor speed control
	void OmegaControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m);

	// Controlling through the EPOS motor speed control assisted by Kalman Filter estimation
	void OmegaControlKF(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m);

	// Collocated Admittance Controller using q' and tau_e, according to A. Calanca, R. Muradore and P. Fiorini
	void CAdmittanceControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m);

	// Collocated Admittance Controller using q' and tau_e, according to A. Calanca, R. Muradore and P. Fiorini
	void CAdmittanceControlKF(float &velHum, std::condition_variable &cv, std::mutex &m);

	// Collocated Admittance Controller using q and tau_e and tau_m
	void CACurrent(float &velHum, std::condition_variable &cv, std::mutex &m);

	// Collocated Admittance Controller using q and tau_e and tau_m
	void CACurrentKF(float &velHum, std::condition_variable &cv, std::mutex &m);

	float getPu(){ return J_EQ*stiffness_d / STIFFNESS + (Kp_adm / STIFFNESS - 1)*damping_d; }

	float getIu(){ return (Ki_adm*damping_d + Kp_adm*stiffness_d - stiffness_d*STIFFNESS) / STIFFNESS; }

	float getI2u(){ return Ki_adm*stiffness_d / STIFFNESS; }

	float getDu(){ return J_EQ*damping_d / STIFFNESS - J_EQ*(1 - stiffness_d / STIFFNESS); }

	// The method to scan the file with the values to update the gains in runtime
	void GainScan();

	// The method to log the desired variables in a text file
	void Recorder();

	// Update the 'Control Word' to show info at the console screen
	void UpdateControlStatus();

	// 'Control Word' to show info at the console screen
	std::string ctrl_word;

	// Savitsky-Golay Smoothing and First Derivative based on the last 11 points
	void SavitskyGolay(float window[], float newest_value, float* first_derivative);

	// Stop the control_t thread loop, allowing .join at the main
	void StopCtrlThread(){ Run = false; }

	// Get the log file name
	char* getLogfilename(){return logger_filename;}

	// Set a closer beginning timestamp for the log running under the Controller thread
	void set_timestamp_begin(std::chrono::system_clock::time_point begin){ timestamp_begin = begin; }

	// Logging logic reproduced at the end of every control method
	void Run_Logger();

	// Set Current setpoint on EPOS within safe limits
	void SetEposCurrentLimited(float current_stp);

	// Set Current setpoint on EPOS within safe limits
	void SetEposVelocityLimited(float speed_stp);

	// Integrator update method with anti-windup and saturation
	float update_i(float error, float Ki, bool limit, float* integrator);

	// Saturation method
	float constrain_float(float val, float min, float max);
	float constrain_float(float val, float constrain);

	// Class Destructor
	~accBasedControl()
	{
		switch (m_control_mode)
		{
		case 'p': case 'k': case 'u':
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

private:

#if UDP_ENABLE
	WSADATA wsaData;
	SOCKET ListenSocket;
	SOCKET ClientSocket;
#endif

	static EPOS_NETWORK* m_epos;
	static AXIS* m_eixo_in;
	static AXIS* m_eixo_out;
	static float m_seconds;
	char m_control_mode;
	static int pos0_out;
	static int pos0_in;

	FILE* logger;
	static std::atomic<bool> logging;
	char logger_filename[40];
	char header_timestamp[30];
	static float timestamp;
	static std::chrono::system_clock::time_point timestamp_begin;

	FILE* gains_values;

	static std::atomic<bool> Run;
	static float control_t_Dt;
	static std::chrono::system_clock::time_point control_t_begin;

	//		GAINS		//

	// Speed Control / Position Control
	static float Kff_V; static float Kp_V; 
	static float Ki_V; static float Kd_V;
	static float Kp_acc, Ki_acc;

	// |-> Admittance Control <---
	// |
	// |- Inner Control:
	static float Ki_adm;		// in the reference is the P [N.m s/rad]
	static float Kp_adm;		// in the reference is the D [N.m/rad]
	static float torque_m;		// output from 'Cp(s)', actually a Cv(s) controller
	static float IntInnerC;		// Integrator of the input in Cv(s)
	static float vel_inner;		// Input velocity to the Inner Loop Controller. It's the k-1 value to be used in the Trapezoidal Integration IntInnerC += (v_k + v_k-1)/2*dt
	static float IntTorqueM;	// Integrator of the Torque to the Motor, torque_m -> 1/(J_EQ*s) -> vel_motor
	static int   resetInt;
	// |- Admittance Control:
	static float torque_ref;	// Reference Torque for A(s).  ? Compensate the lower leg exo mass and dynamics ?
	static float Adm_In;		// A(s) Input Derivative = d/dt(torque_ref - torque_sea)
	static float IntAdm_In;		// A(s) Input integrator
	static float stiffness_d;	// k_d	  [N.m/rad]
	static float damping_d;		// d_{dm} [N.m s/rad]
	static float k_bar;			// 1 - k_d/K
	static float vel_adm;		// [rad/s] output from A(s), the velocity required to guarantee the desired Admittance
	static float vel_adm_last;	// [rad/s]
	static float kd_min;
	static float kd_max;
	// |- Acc-based Admittance Control*:
	static float torque_u;
	static float IntTsea;
	static float Int2Tsea;
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
  	static float torque_sea_last;	// [N.m]
	static float d_torque_sea;		// [N.m/s]
	static float accbased_comp;		// [N.m]
	static float d_accbased_comp;	// [N.m/s]
	static float grav_comp;			// [N.m]
	static float des_tsea_last;		// [N.m]
	static float int_stiffness;		// [N.m/rad]

	static float vel_leg;			// [rpm ?]
	static float acc_motor;			// [rad/s^2]
	static float vel_motor;			// [rad/s]
	static float vel_motor_filt;	// [rad/s]
	static float voltage;			// [V]

	static float theta_m;			// [rad]
	static float theta_m_last;
	static int actualVelocity;		// [rpm]
	static int exoVelocity;			// [rpm]

	static float diffCutoff;
	static float IntegratorHum;
	static float IntegratorExo;
	static float IntAccMotor;
	static float IntAccHum;
	static float IntAccExo;
	static uint8_t downsample;

	//		STATE MEMORY VECTORS        //

	float velhumVec[SGVECT_SIZE];		// [rad/s]
	float velexoVec[SGVECT_SIZE];		// [rad/s]
	float torqueSeaVec[SGVECT_SIZE];	// [N.m]
	float torqueAccVec[SGVECT_SIZE];	// [N.m]
	float theta_l_vec[SGVECT_SIZE];		// [rad]
	float theta_c_vec[SGVECT_SIZE];		// [rad]

	//		Kalman Filter		//

	static Matrix<float, 5, 1> CAC_xk;	// State Vector				[vel_hum vel_motor theta_c theta_l torque_sea]
	static Matrix<float, 4, 1> CAC_zk;	// Sensor reading Vector	[vel_hum vel_motor theta_c theta_l]
	static Matrix<float, 5, 5> CAC_Pk;	// State Covariance Matrix
	static Matrix<float, 5, 5> CAC_Fk;	// Prediction Matrix
	static Matrix<float, 5, 1> CAC_Bk;	// Control Matrix (is a vector but called matrix)
	static Matrix<float, 1, 1> CAC_uk; // Control Vector
	static Matrix<float, 5, 5> CAC_Qk;	// Process noise Covariance
	static Matrix<float, 4, 4> CAC_Rk;	// Sensor noise Covariance
	static Matrix<float, 4, 5> CAC_Hk;	// Sensor Expectations Matrix
	static Matrix<float, 5, 4> CAC_KG;	// Kalman Gain Matrix

//		Extended Kalman Filter		//

	static StateSzMtx ekf_Gk;										// State transition Jacobian
	static StateSzMtx ekf_Pk;										// State Covariance Matrix
	static StateSzMtx ekf_Qk;	 									// Process noise Covariance
	static SensorSzMtx ekf_Rk; 										// Sensor noise Covariance
	static Matrix<float, EKF_STATE_DIM, 1> ekf_xk;					// State Vector				[x_h x_e \dot{x_e} x_a \dot{x_a}]
	static Matrix<float, EKF_SENSOR_DIM, 1> ekf_zk;					// Sensor reading Vector	[\dot{x_h} x_e \dot{x_e} x_a \dot{x_a}]
	static Matrix<float, EKF_CTRL_DIM, 1> ekf_uk; 				 	// Control Vector
	static Matrix<float, EKF_STATE_DIM, EKF_CTRL_DIM> 	ekf_Bk;		// Control Matrix
	static Matrix<float, EKF_SENSOR_DIM, EKF_STATE_DIM> ekf_Hk;		// Sensor expectations Jacobian
	static Matrix<float, EKF_STATE_DIM, EKF_SENSOR_DIM> ekf_KG;		// Kalman Gain Matrix
	static Matrix<float, EKF_SENSOR_DIM, EKF_CTRL_DIM>	ekf_Dk;		// Feedforward Matrix

	static float ekfPosHum;
	static float ekfPosExo;
	static float ekfVelExo;
	static float ekfPosAct;
	static float ekfVelAct;
	static uint8_t ekf_skip;

	FILE *ekfLogFile;
};

#endif // !CURRENT_CONTROL_H

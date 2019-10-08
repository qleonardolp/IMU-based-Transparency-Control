///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2019     ///
// leonardo.felipe.santos@usp.br	_____ ___  ___   //
// github/bitbucket qleonardolp		| |  | \ \/   \  //
////////////////////////////////	| |   \ \   |_|  //
////////////////////////////////	\_'_/\_`_/__|    //
///////////////////////////////////////////////////////

#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <stdio.h>
#include <time.h>

// CONSTANTES

#define		GEAR_RATIO		150.0		    // Redução do Sistema
#define		ENCODER_IN		4096		    // Resolução do encoder do motor
#define		ENCODER_OUT		2048		    // Resolução do encoder de saída
#define		STIFFNESS		104.0			// Constante da mola SEA [N.m/rad]

//	Torque Constant: 0.0603 N.m/A = 60.3 N.m/mA
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A

#define		CURRENT_MAX		3.1000		// Max corrente nominal no motor Maxon RE40 [A]
#define		TORQUE_CONST	60.300		// Constante de torque do motor RE40	[N.m/mA]
#define		SPEED_CONST		158.00		// Constante de velocidade do motor RE40 [rpm/V]

#define     GRAVITY         9.8066      // [m/s^2]
#define     INERTIA_EXO     0.0655      // [Kg.m^2], +- 0.0006, estimado em 2019-08-21
#define		MTW_DIST_LIMB	0.2500		// [m]
#define		MTW_DIST_EXO	0.0700		// [m]
#define		L_CG			0.3500		// [m]

// Feedforward-Feedback PI acc-based controller:
#define		K_FF			1.0000		// [dimensionless]
#define     KP_A			4.5600      // [Kg.m^2]
#define     KI_A			5.7900      // [Kg.m^2/s]

// Feedback PD force (SEA) controller:
#define     KP_F			1.5310      // [dimensionless]
#define     KD_F			0.0200      // [s]


#define     RATE            142.86      // [Hz]	Use the control loop rate running
#define     LPF_FC          5.000      // [Hz] Low Pass Filter Frequency Cutoff
#define		MY_PI			3.141592653	// Pi value
#define		LPF_SMF         ( (2*MY_PI / RATE) / (2*MY_PI / RATE + 1 / LPF_FC) )    // Low Pass Filter Smoothing Factor


class accBasedControl
{
public:
	// constructor
	accBasedControl(EPOS_NETWORK* epos, AXIS* eixo_in, AXIS* eixo_out, int seconds)
	{

		m_epos = epos;
		m_eixo_in = eixo_in;
		m_eixo_out = eixo_out;

		pos0_out = -m_eixo_out->PDOgetActualPosition();
		pos0_in = m_eixo_in->PDOgetActualPosition();

		K_ff = K_FF;
		Kp_A = KP_A;
		Ki_A = KI_A;
		Kp_F = KP_F;
		Kd_F = KD_F;
		Amplifier = 100000; // initialized with a safe value

		vel_hum = 0;
		vel_exo = 0;

		for (size_t i = 0; i < 6; ++i)
		{
			velhumVec[i] = 0;
			velexoVec[i] = 0;
			torqueSeaVec[i] = 0;
		}

		torque_sea = 0;
		setpoint = 0;
		setpoint_filt = 0;

		if (seconds != 0)
		{
			logging = true;
			log_count = seconds*RATE;

			time_t rawtime;
			struct tm* timeinfo;
			time(&rawtime);
			timeinfo = localtime(&rawtime);

			strftime(logger_filename, 40, "./data/%Y-%m-%d-%H-%M-%S.txt", timeinfo);
			logger = fopen(logger_filename, "wt");
			if (logger != NULL)
			{
				//printing the first line, the header:
				fprintf(logger, "SetPt[mA]  I_m[mA] theta_l[deg]  theta_c[deg]  T_sea[N.m]  T_acc[N.m]  K_ff  Kp_A  Ki_A  Kp_F  Kd_F  Amp\n");
				fclose(logger);
			}
		}
		else
		{
			logging = false;
			log_count = 0;
		}

	}

	// numdiff
	void FiniteDiff(float velHum, float velExo);

	// acc-gravity
	void Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z);

	void OmegaControl(float velHum, float velExo);

	void Gains_Scan();

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
	int log_count;
	bool logging;

	FILE* gains_values;
	int Amplifier;
	float K_ff;
	float Kp_A;
	float Ki_A;
	float Kp_F;
	float Kd_F;

	float acc_hum;			// [rad/s^2]
	float acc_exo;			// [rad/s^2]
	float vel_hum;			// [rad/s]
	float vel_exo;			// [rad/s]

	float velhumVec[6];		// [rad/s]
	float velexoVec[6];		// [rad/s]
	float torqueSeaVec[6];	// [N.m]

	float setpoint;			// [mA]
	float setpoint_filt;

	float theta_l;			// [rad]
	float theta_c;			// [rad]

	float torque_sea;		// [N.m]
	float d_torque_sea;		// [N.m/s]
	float accbased_comp;	// [N.m]
	float grav_comp;		// [N.m]

	int pos0_out;
	int pos0_in;
};

#endif // !CURRENT_CONTROL_H

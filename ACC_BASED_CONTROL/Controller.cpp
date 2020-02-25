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


#include "Controller.h"

using namespace std;
using namespace chrono;

// accBasedControl Paramount Variables
EPOS_NETWORK* accBasedControl::m_epos;
AXIS* accBasedControl::m_eixo_in;
AXIS* accBasedControl::m_eixo_out;
float accBasedControl::m_seconds;
int   accBasedControl::pos0_out;
int   accBasedControl::pos0_in;
atomic<bool> accBasedControl::logging(false);	// just for safety
system_clock::time_point accBasedControl::control_t_begin;
float accBasedControl::control_t_Dt = 0.008333;	// initialized with the old value from 120 Hz
float accBasedControl::timestamp = 0.000000000f;
atomic<bool> accBasedControl::Run(true);

// Speed Control [s]
float accBasedControl::Kp_V = 0;
float accBasedControl::Ki_V = 0;
float accBasedControl::Kd_V = 0;
float accBasedControl::Kff_V = 0;

// Adimittance Control [a,u] Variables
float accBasedControl::Ki_adm = 0;
float accBasedControl::Kp_adm = 0;
float accBasedControl::torque_m;
float accBasedControl::IntInnerC = 0;
float accBasedControl::IntTorqueM = 0;
int   accBasedControl::resetInt = 0;
float accBasedControl::torque_ref = 0;
float accBasedControl::Adm_In;
float accBasedControl::IntAdm_In = 0;
float accBasedControl::stiffness_d = STIFFNESS / 2;
float accBasedControl::damping_A = 0.001;
float accBasedControl::vel_adm = 0;
float accBasedControl::vel_adm_last = 0;
float accBasedControl::kd_min = damping_A*(Ki_adm / Kp_adm - damping_A / (J_EQ*(1 - stiffness_d / STIFFNESS)) - Kp_adm / J_EQ);
float accBasedControl::kd_max = STIFFNESS;

// State Variables
float accBasedControl::vel_hum = 0;			// [rad/s]
float accBasedControl::vel_exo = 0;			// [rad/s]
float accBasedControl::acc_hum = 0;			// [rad/s^2]
float accBasedControl::acc_exo = 0;			// [rad/s^2]
float accBasedControl::jerk_hum = 0;		// [rad/s^3]
float accBasedControl::jerk_exo = 0;		// [rad/s^3]
float accBasedControl::theta_l = 0;			// [rad]
float accBasedControl::theta_c = 0;			// [rad]
float accBasedControl::vel_hum_last = 0;
float accBasedControl::vel_exo_last = 0;

// Auxiliar Variables
float accBasedControl::setpoint = 0;		// [A]
float accBasedControl::setpoint_filt = 0;	// [A]
int   accBasedControl::actualCurrent = 0;	// [mA] Read from the EPOS
float accBasedControl::accbased_comp = 0;	// [N.m]
float accBasedControl::d_accbased_comp = 0;	// [N.m/s]
float accBasedControl::torque_sea = 0;		// [N.m]
float accBasedControl::d_torque_sea = 0;	// [N.m/s]
float accBasedControl::grav_comp = 0;		// [N.m]
float accBasedControl::vel_leg = 0;			// [rpm ?]
float accBasedControl::acc_motor = 0;		// [rad/s^2]
float accBasedControl::vel_motor = 0;		// [rad/s]
float accBasedControl::vel_motor_filt = 0;	// [rad/s]
float accBasedControl::voltage = 0;			// [V]
float accBasedControl::theta_m = 0;			// [encoder pulses]
int	  accBasedControl::actualVelocity = 0;	// [rpm]
int   accBasedControl::exoVelocity = 0;		// [rpm]
float accBasedControl::diffCutoff = CUTOFF;	// [Hz] ?
float accBasedControl::IntegratorHum = 0;
float accBasedControl::IntegratorExo = 0;
float accBasedControl::IntAccMotor = 0;
float accBasedControl::IntAccHum = 0;
float accBasedControl::IntAccExo = 0;


// Control Functions //

void accBasedControl::OmegaControl(float &velHum, float &velExo, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin)
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(1500));

		m_epos->sync();	// CAN Synchronization

		vel_hum = HPF_SMF*vel_hum + HPF_SMF*(velHum - vel_hum_last);	// HPF on gyroscopes
		vel_hum_last = velHum;
		vel_exo = HPF_SMF*vel_exo + HPF_SMF*(velExo - vel_exo_last);	// HPF on gyroscopes
		vel_exo_last = velExo;

		// velocities derivative using a Gain and a Discrete Integrator in loop
		acc_exo = 24 * (vel_exo - IntegratorExo);
		IntegratorExo += acc_exo*DELTA_T;
		acc_hum = 24 * (vel_hum - IntegratorHum);
		IntegratorHum += acc_hum*DELTA_T;

		// accelerations derivative using a Gain and a Discrete Integrator in loop
		jerk_hum = 15 * (acc_hum - IntAccHum);
		IntAccHum += jerk_hum*DELTA_T;
		jerk_exo = 15 * (acc_exo - IntAccExo);
		IntAccExo += jerk_exo*DELTA_T;

		// SEA Torque:

		m_eixo_out->ReadPDO01();
		theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();
		theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		torque_sea += LPF_SMF*(STIFFNESS*(theta_c - theta_l) - torque_sea);

		// Jerk Feedforward:
		m_eixo_out->ReadPDO02();
		exoVelocity = m_eixo_out->PDOgetActualVelocity(); //  [rpm] ???
		vel_motor = vel_exo + (Kff_V*INERTIA_EXO*jerk_hum + Kp_V*(jerk_hum - jerk_exo) + Ki_V*(acc_hum - acc_exo)) / STIFFNESS;

		vel_motor = RADS2RPM * GEAR_RATIO * vel_motor;
		vel_motor_filt += LPF_SMF*(vel_motor - vel_motor_filt);

		voltage = abs(vel_motor_filt / SPEED_CONST);

		if ((voltage > 0.100) && (voltage <= VOLTAGE_MAX))
		{
			m_eixo_in->PDOsetVelocitySetpoint((int)vel_motor_filt);
		}
		else if (voltage > VOLTAGE_MAX)	// upper saturation
		{
			if (vel_motor_filt < 0)
				m_eixo_in->PDOsetVelocitySetpoint(-(int)(SPEED_CONST * VOLTAGE_MAX));
			else
				m_eixo_in->PDOsetVelocitySetpoint((int)(SPEED_CONST * VOLTAGE_MAX));
		}
		else                            // lower saturation
		{
			m_eixo_in->PDOsetVelocitySetpoint(0);
		}
		m_eixo_in->WritePDO02();

		m_eixo_in->ReadPDO02();
		actualVelocity = m_eixo_in->PDOgetActualVelocity();  //  [rpm]

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float) duration_cast<microseconds>(control_t_end - control_t_begin).count();
    control_t_Dt = 1e-6*control_t_Dt;

		timestamp = (float)duration_cast<microseconds>(control_t_end - begin).count();
    timestamp = 1e-6*timestamp;
		if (timestamp < m_seconds) Recorder_Velocity();
	}
}


void accBasedControl::CAdmittanceControl(float &velHum, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin)
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(1500));

		resetInt++;	// counter to periodically reset the Inner Loop Integrator 
		m_epos->sync();	// CAN Synchronization

		vel_hum = HPF_SMF*vel_hum + HPF_SMF*(velHum - vel_hum_last);	// HPF on gyro
		vel_hum_last = velHum;

		// SEA Torque:
		m_eixo_out->ReadPDO01();
		theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();
		theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		torque_sea += LPF_SMF*(STIFFNESS*(theta_c - theta_l) - torque_sea);
		d_torque_sea = 13 * (torque_sea - IntAdm_In);
		IntAdm_In += d_torque_sea*control_t_Dt;

		// Outer Admittance Control loop: the discrete realization relies on the derivative of tau_e (check my own red notebook)
		// Here, the reference torque is zero!
		vel_adm = damping_A / (damping_A + stiffness_d*control_t_Dt) * vel_adm +
			(1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_A / control_t_Dt) * (-d_torque_sea);
		// testar sem as duas linhas abaixo
		vel_adm += LPF_SMF*(vel_adm - vel_adm_last);
		vel_adm_last = vel_adm;

		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Integration for the Inner Control (PI)
		IntInnerC += (vel_hum + vel_adm - vel_motor)*control_t_Dt;
		// Integration reset:
		if (resetInt == 1700)
			IntInnerC = (vel_hum + vel_adm - vel_motor)*control_t_Dt;

		// Inner Control Loop (PI):
		torque_m = Kp_adm*(vel_hum + vel_adm - vel_motor) + Ki_adm*IntInnerC;

		// Motor Dynamics:
		torque_ref += LPF_SMF*(L_CG*LOWERLEGMASS*GRAVITY*sinf(theta_l) - torque_ref);
		torque_m = torque_m - torque_sea;
		// torque_m -> 1/(J_EQ*s) -> vel_motor:
		IntTorqueM += (torque_m / J_EQ)*control_t_Dt;
		// Integration reset:
		if (resetInt == 2000)
		{
			IntTorqueM = (torque_m / J_EQ)*control_t_Dt;
			resetInt = 0;
		}

		vel_motor = RADS2RPM*GEAR_RATIO*IntTorqueM;

		voltage = abs(vel_motor / SPEED_CONST);

		if ((voltage > 0.100) && (voltage <= VOLTAGE_MAX))
		{
			m_eixo_in->PDOsetVelocitySetpoint((int)vel_motor);
		}
		else if (voltage > VOLTAGE_MAX)	// upper saturation
		{
			if (vel_motor < 0)
				m_eixo_in->PDOsetVelocitySetpoint(-(int)(SPEED_CONST * VOLTAGE_MAX));
			else
				m_eixo_in->PDOsetVelocitySetpoint((int)(SPEED_CONST * VOLTAGE_MAX));
		}
		else                            // lower saturation
		{
			m_eixo_in->PDOsetVelocitySetpoint(0);
		}
		m_eixo_in->WritePDO02();

		m_eixo_in->ReadPDO02();
		actualVelocity = m_eixo_in->PDOgetActualVelocity();  //  [rpm]

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float) duration_cast<microseconds>(control_t_end - control_t_begin).count();
    control_t_Dt = 1e-6*control_t_Dt;

		timestamp = (float)duration_cast<microseconds>(control_t_end - begin).count();
    timestamp = 1e-6*timestamp;
		if (timestamp < m_seconds) Recorder_CAC();
	}
}

void accBasedControl::CACurrent(float &velHum, std::condition_variable &cv, std::mutex &m, std::chrono::system_clock::time_point &begin)
{
	while (Run.load())
	{
		unique_lock<std::mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(1500));

		resetInt++;	// counter to periodically reset the Inner Loop Integrator 
		m_epos->sync();	// CAN Synchronization

		vel_hum = HPF_SMF*vel_hum + HPF_SMF*(velHum - vel_hum_last);	// HPF on gyro
		vel_hum_last = velHum;

		// SEA Torque:
		m_eixo_out->ReadPDO01();
		theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();
		theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		torque_sea += LPF_SMF*(STIFFNESS*(theta_c - theta_l) - torque_sea);
		d_torque_sea = 13 * (torque_sea - IntAdm_In);
		IntAdm_In += d_torque_sea*control_t_Dt;

		// Outer Admittance Control loop: the discrete realization relies on the derivative of tau_e (check my own red notebook)
		// Here, the reference torque is the torque required to sustain the Exo lower leg mass
		vel_adm = damping_A / (damping_A + stiffness_d*control_t_Dt) * vel_adm +
			(1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_A / control_t_Dt) * (-d_torque_sea);
		// testar sem as duas linhas abaixo
		vel_adm += LPF_SMF*(vel_adm - vel_adm_last);
		vel_adm_last = vel_adm;

		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Integration for the Inner Control (PI)
		IntInnerC += (vel_hum + vel_adm - vel_motor)*control_t_Dt;
		// Integration reset:
		if (resetInt == 2000)
		{
			IntInnerC = (vel_hum + vel_adm - vel_motor)*control_t_Dt;
			resetInt = 0;
		}
		// Inner Control Loop (PI):
		torque_m = Kp_adm*(vel_hum + vel_adm - vel_motor) + Ki_adm*IntInnerC;
		torque_ref += LPF_SMF*(L_CG*LOWERLEGMASS*GRAVITY*sinf(theta_l) - torque_ref);

		setpoint = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_m; // now in Ampere!
		setpoint_filt += LPF_SMF * (setpoint - setpoint_filt);

		if (abs(setpoint_filt) < CURRENT_MAX)
		{
			if ((theta_l >= -1.5708) && (theta_l <= 0.5236)) //(caminhando)
				m_eixo_in->PDOsetCurrentSetpoint((int)(setpoint_filt * 1000));	// esse argumento é em mA !!!
			else
				m_eixo_in->PDOsetCurrentSetpoint(0);
		}
		else
		{
			if (setpoint_filt < 0)
				m_eixo_in->PDOsetCurrentSetpoint(-(int)(CURRENT_MAX * 1000));
			else
				m_eixo_in->PDOsetCurrentSetpoint((int)(CURRENT_MAX * 1000));
		}
		m_eixo_in->WritePDO01();

		m_eixo_in->ReadPDO01();
		actualCurrent = m_eixo_in->PDOgetActualCurrent();

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float) duration_cast<microseconds>(control_t_end - control_t_begin).count();
    control_t_Dt = 1e-6*control_t_Dt;

		timestamp = (float)duration_cast<microseconds>(control_t_end - begin).count();
    timestamp = 1e-6*timestamp;
		if (timestamp < m_seconds) Recorder_CACu();
	}
}

void accBasedControl::GainScan_Current()
{
	gains_values = fopen("gainsCurrent.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "K_FF %f\nKP_A %f\nKI_A %f\nKP_F %f\nKD_F %f\nAMP %d\n", &K_ff, &Kp_A, &Ki_A, &Kp_F, &Kd_F, &Amplifier);
		fclose(gains_values);
	}
}

void accBasedControl::GainScan_CurrentKF()
{
	gains_values = fopen("gainsCurrentKF.txt", "rt");

	if (gains_values != NULL)
	{
		//fscanf(gains_values, "AMP %d\n", &Amp_kf);
		fclose(gains_values);
	}
}

void accBasedControl::GainScan_Velocity()
{
	gains_values = fopen("gainsSpeed.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "KFF_V %f\nKP_V %f\nKI_V %f\nKD_V %f\n", &Kff_V, &Kp_V, &Ki_V, &Kd_V);
		fclose(gains_values);
	}
}

void accBasedControl::GainScan_CAC()
{
	gains_values = fopen("gainsCAC.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "KP %f\nKI %f\nSTF %f\nDAM %f\n", &Kp_adm, &Ki_adm, &stiffness_d, &damping_A);
		fclose(gains_values);
	}
}

void accBasedControl::GainScan_CACu()
{
	gains_values = fopen("gainsCACu.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "KP %f\nKI %f\nSTF %f\nDAM %f\n", &Kp_adm, &Ki_adm, &stiffness_d, &damping_A);
		fclose(gains_values);
	}
}

void accBasedControl::UpdateCtrlWord_Current()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+5.3f", 1000 * setpoint_filt);
	ctrl_word = " setpt: " + (std::string) numbers_str;
	sprintf(numbers_str, "%+5d", actualCurrent);
	ctrl_word += " [" + (std::string) numbers_str + " mA]\n";
	sprintf(numbers_str, "%+5.3f", theta_l * (180 / MY_PI));
	ctrl_word += " theta_l: " + (std::string) numbers_str + " deg";
	sprintf(numbers_str, "%+5.3f", theta_c * (180 / MY_PI));
	ctrl_word += " theta_c: " + (std::string) numbers_str + " deg\n";
	sprintf(numbers_str, "%+5.3f", torque_sea);
	ctrl_word += " T_sea: " + (std::string) numbers_str + " N.m";
	sprintf(numbers_str, "%+5.3f", accbased_comp);
	ctrl_word += " T_acc: " + (std::string) numbers_str + " N.m\n";
	sprintf(numbers_str, "%5.3f", K_ff);
	ctrl_word += " K_ff: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Kp_A);
	ctrl_word += " Kp_A: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Ki_A);
	ctrl_word += " Ki_A: " + (std::string) numbers_str + "\n";
	sprintf(numbers_str, "%5.3f", Kp_F);
	ctrl_word += " Kp_F: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Kd_F);
	ctrl_word += " Kd_F: " + (std::string) numbers_str + "\n";
	sprintf(numbers_str, "%5d", Amplifier);
	ctrl_word += " Amplifier: " + (std::string) numbers_str + "\n";
	sprintf(numbers_str, "%+5.3f", RADS2RPM*vel_motor);
	ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm\n";
}

void accBasedControl::UpdateCtrlWord_CurrentKF()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+.3f", 1000 * setpoint_filt);
	ctrl_word = " setpt: " + (std::string) numbers_str;
	sprintf(numbers_str, "%+5d", actualCurrent);
	ctrl_word += " [" + (std::string) numbers_str + " mA]\n";
	sprintf(numbers_str, "%+5.3f", theta_l * (180 / MY_PI));
	ctrl_word += " theta_l: " + (std::string) numbers_str + " deg";
	sprintf(numbers_str, "%+5.3f", theta_c * (180 / MY_PI));
	ctrl_word += " theta_c: " + (std::string) numbers_str + " deg\n";
	sprintf(numbers_str, "%+.3f", torque_sea);
	ctrl_word += " T_sea: " + (std::string) numbers_str + " N.m";
	sprintf(numbers_str, "%+.3f", accbased_comp);
	ctrl_word += " T_acc: " + (std::string) numbers_str + " N.m\n";
	//sprintf(numbers_str, "%.3f", Amp_kf);
	//ctrl_word += " Amplifier: " + (std::string) numbers_str + "\n";
	sprintf(numbers_str, "%+5.3f", RADS2RPM*vel_motor);
	ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm\n";
	//ctrl_word += " " + kf_error + "\n";
}


void accBasedControl::UpdateCtrlWord_Velocity()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+5.3f", vel_motor_filt);
	ctrl_word = " vel_motor: " + (std::string) numbers_str + " rpm";
	sprintf(numbers_str, "%+4d", actualVelocity);
	ctrl_word += " [" + (std::string) numbers_str + " rpm	";
	sprintf(numbers_str, "%+2.3f", abs(actualVelocity / SPEED_CONST));
	ctrl_word += (std::string) numbers_str + " V]\n";
	sprintf(numbers_str, "%5.3f", Kff_V);
	ctrl_word += " Kff_V: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Kp_V);
	ctrl_word += " Kp_V: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Ki_V);
	ctrl_word += " Ki_V: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Kd_V);
	ctrl_word += " Kd_V: " + (std::string) numbers_str + "\n";
	ctrl_word += " T_Sea: " + std::to_string(torque_sea) + " N.m\n";
}

void accBasedControl::UpdateCtrlWord_Admittance()
{
	char numbers_str[20];
	ctrl_word = " COLLOCATED ADMITTANCE CONTROLLER\n";
	if (m_control_mode == 'a')
	{
		sprintf(numbers_str, "%+5.3f", vel_motor);
		ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm";
		sprintf(numbers_str, "%+4d", actualVelocity);
		ctrl_word += " [" + (std::string) numbers_str + " rpm	";
		sprintf(numbers_str, "%+2.3f", abs(actualVelocity / SPEED_CONST));
		ctrl_word += (std::string) numbers_str + " V]\n";
	}
	else if (m_control_mode == 'u')
	{
		ctrl_word += " Torque: " + std::to_string(torque_m) + " N.m";
		ctrl_word += " [ " + std::to_string(1000 * setpoint_filt) + " ";
		ctrl_word += std::to_string(actualCurrent) + " mA]\n";
	}
	sprintf(numbers_str, "%5.3f", Kp_adm);
	ctrl_word += " Kp: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Ki_adm);
	ctrl_word += " Ki: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", stiffness_d);
	ctrl_word += " STF: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", damping_A);
	ctrl_word += " DAM: " + (std::string) numbers_str + "\n";
	ctrl_word += " T_Sea: " + std::to_string(torque_sea);
	ctrl_word += " | T_ref: " + std::to_string(torque_ref) + " [N.m]\n";
	ctrl_word += " -> Passivity Constraints <-\n ";

	kd_min = damping_A*(Ki_adm / Kp_adm - damping_A / (J_EQ*(1 - stiffness_d / STIFFNESS)) - Kp_adm / J_EQ);

	ctrl_word += std::to_string(kd_min) + " < kd < " + std::to_string(kd_max) + "\n";
	sprintf(numbers_str, "%.8f", control_t_Dt);
	ctrl_word += " Control Dt: " + (std::string) numbers_str + " sec\n";
}


void accBasedControl::Recorder_Current()
{
	logger = fopen(logger_filename, "a");
	if (logger != NULL)
	{
		fprintf(logger, "%5.3f  %5d   %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",
			setpoint_filt, actualCurrent, theta_l * (180 / MY_PI), theta_c * (180 / MY_PI), accbased_comp, acc_hum, acc_exo, vel_hum, vel_exo, vel_motor);
		fclose(logger);
	}
}

void accBasedControl::Recorder_Velocity()
{
	logger = fopen(logger_filename, "a");
	if (logger != NULL)
	{
		fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, jerk_hum, jerk_exo, RPM2RADS*vel_motor_filt, RPM2RADS*actualVelocity, RPM2RADS*exoVelocity, torque_sea, theta_c, theta_l);
		// everything logged in standard units (SI)
		fclose(logger);
	}
}

void accBasedControl::Recorder_CAC()
{
	logger = fopen(logger_filename, "a");
	if (logger != NULL)
	{
		fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",
			timestamp, vel_hum, vel_adm, RPM2RADS*vel_motor, RPM2RADS*(float)actualVelocity, torque_sea);
		// everything logged in standard units (SI)
		fclose(logger);
	}
}

void accBasedControl::Recorder_CACu()
{
	logger = fopen(logger_filename, "a");
	if (logger != NULL)
	{
		fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5d  %5.3f\n",
			timestamp, vel_hum, vel_adm, vel_motor, 1000 * setpoint_filt, actualCurrent, torque_sea);
		// everything logged in standard units (SI)
		fclose(logger);
	}
}

void accBasedControl::savitskygolay(float window[], float newest_value, float* first_derivative)
{
	// shifting values in 1 position, updating the window with one sample:
	for (int i = 10; i > 0; --i)
	{
		window[i] = window[i - 1];
	}

	// *Savitsky-Golay Smoothing Coefficients, 4th order fit with 11 pts and +5 offset from the centre point
	// Norm: 143 |	Coeff: 6	-10	-5	5	10	6	-5	-15	-10	30	131

	window[0] = (6 * window[10] - 10 * window[9] - 5 * window[8]
		+ 5 * window[7] + 10 * window[6] + 6 * window[5]
		- 5 * window[4] - 15 * window[3] - 10 * window[2]
		+ 30 * window[1] + 131 * newest_value) / 143;


	// *Finite Difference, SG Coefficients, First Derivative, linear fit with 11 pts and +5 offset
	*first_derivative = (5 * window[0] + 4 * window[1] + 3 * window[2] + 2 * window[3] +
		1 * window[4] + 0 * window[5] - 1 * window[6] - 2 * window[7] -
		3 * window[8] - 4 * window[9] - 5 * window[10]) / (110) * RATE;

}
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
using namespace Eigen;

// accBasedControl Paramount Variables
EPOS_NETWORK* accBasedControl::m_epos;
AXIS* accBasedControl::m_eixo_in;
AXIS* accBasedControl::m_eixo_out;
float accBasedControl::m_seconds;
int   accBasedControl::pos0_out;
int   accBasedControl::pos0_in;
atomic<bool> accBasedControl::Run(true);
atomic<bool> accBasedControl::logging(false);
system_clock::time_point accBasedControl::control_t_begin;
system_clock::time_point accBasedControl::timestamp_begin;
float accBasedControl::control_t_Dt = 0.001;				// [s]
float accBasedControl::timestamp = 0.0;

// Speed Control [s]
float accBasedControl::Kp_V = 0;
float accBasedControl::Ki_V = 0;
float accBasedControl::Kd_V = 0;
float accBasedControl::Kff_V = 0;

// acc-based Controller
float accBasedControl::Kp_acc = 0.3507;
float accBasedControl::Ki_acc = 10.760;
float accBasedControl::Kff_acc = 0.00000f;

// Basic Impedance Control
float accBasedControl::Kp_bic(0);
float accBasedControl::Kd_bic(0);


//	Kalman Filter	//
StateSzVec accBasedControl::xk;	// State Vector
SensorSzVec accBasedControl::zk;	// Sensor reading Vector
ControlSzVec accBasedControl::uk; 	// Control Vector
StateSzMtx accBasedControl::Pk;	// State Covariance Matrix
StateSzMtx accBasedControl::Fk;	// Prediction Matrix
StateSzMtx accBasedControl::At;	// Continuous time state transition Matrix
ControlSzMtx accBasedControl::Gk;	// Control Matrix
ControlSzMtx accBasedControl::Bt;	// Continuous time state Control Matrix
StateSzMtx accBasedControl::Qk;	// Process noise Covariance
SensorSzMtx accBasedControl::Rk;	// Sensor noise Covariance
Matrix<float,KF_SENSOR_DIM,KF_STATE_DIM> accBasedControl::Ck;	// Sensor Expectations Matrix
Matrix<float,KF_SENSOR_DIM,KF_CTRL_DIM> accBasedControl::Dk;	// Feedthrough Matrix
Matrix<float,KF_STATE_DIM,KF_SENSOR_DIM> accBasedControl::KG;	// Kalman Gain Matrix


kfhStateVec  accBasedControl::KFH_xk;	// State Vector				[dTr Tr dTi Ti dpr pr dph ph Terr]
kfhSensorVec accBasedControl::KFH_zk;	// Sensor reading Vector	[Tr Ti pr dph ph]
kfhStateMtx  accBasedControl::KFH_Pk;	// State Covariance Matrix
kfhStateMtx  accBasedControl::KFH_Fk;	// Transition Matrix (discrete)
kfhStateVec  accBasedControl::KFH_Gk;	// Control Matrix (discrete)
kfhStateMtx  accBasedControl::KFH_Qk;	// Process noise Covariance
kfhSensorMtx accBasedControl::KFH_Rk;	// Sensor noise Covariance
Matrix<float,KFH_SENSOR_DIM,KFH_STATE_DIM> accBasedControl::KFH_Ck;	// Sensor Expectations Matrix
Matrix<float,KFH_STATE_DIM,KFH_SENSOR_DIM> accBasedControl::KFH_KG;	// Gain Matrix

float accBasedControl::kf_pos_hum(0);
float accBasedControl::kf_pos_exo(0);
float accBasedControl::kf_pos_act(0);
float accBasedControl::kf_vel_exo(0);
float accBasedControl::kf_vel_act(0);
float accBasedControl::kf_vel_hum(0);
float accBasedControl::kf_acc_hum(0);
float accBasedControl::kf_acc_exo(0);
float accBasedControl::kf_torque_int(0);
float accBasedControl::kf_vel_hum_last(0);
float accBasedControl::kf_vel_hum_hold(0);
float accBasedControl::kf_vel_exo_hold(0);
uint8_t accBasedControl::downsamplekf = 1;

// Adimittance Control [a,u] Variables
float accBasedControl::Ki_adm = 1.190;
float accBasedControl::Kp_adm = 11.90;
float accBasedControl::torque_m;
float accBasedControl::IntInnerC = 0;
float accBasedControl::vel_inner = 0;
float accBasedControl::IntTorqueM = 0;
int   accBasedControl::resetInt = 0;
float accBasedControl::torque_ref = 0;
float accBasedControl::Adm_In;
float accBasedControl::IntAdm_In = 0;
float accBasedControl::stiffness_d = STIFFNESS / 2;
float accBasedControl::damping_d = 0.001;
float accBasedControl::k_bar = 1 - stiffness_d / STIFFNESS;
float accBasedControl::vel_adm = 0;
float accBasedControl::vel_adm_last = 0;
float accBasedControl::kd_min = STIFFNESS/10;
float accBasedControl::kd_max = STIFFNESS;
float accBasedControl::IntTsea = 0;

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
float accBasedControl::des_tsea_last = 0;   // [N.m]
float accBasedControl::torque_sea = 0;		// [N.m]
float accBasedControl::torque_sea_last = 0;		// [N.m]
float accBasedControl::d_torque_sea = 0;	// [N.m/s]
float accBasedControl::grav_comp = 0;		// [N.m]
float accBasedControl::vel_leg = 0;			// [rpm ?]
float accBasedControl::acc_motor = 0;		// [rad/s^2]
float accBasedControl::vel_motor = 0;		// [rad/s]
float accBasedControl::vel_motor_filt = 0;	// [rad/s]
float accBasedControl::voltage = 0;			// [V]
float accBasedControl::theta_m = 0;			// [rad]
float accBasedControl::theta_m_last = 0;	//
int	  accBasedControl::actualVelocity = 0;	// [rpm]
int   accBasedControl::exoVelocity = 0;		// [rpm]
float accBasedControl::IntegratorHum = 0;
float accBasedControl::IntegratorExo = 0;
float accBasedControl::IntAccMotor = 0;
float accBasedControl::IntAccHum = 0;
float accBasedControl::IntAccExo = 0;
uint8_t	accBasedControl::downsample = 1;
uint8_t	accBasedControl::downsamplelog = 1;
float accBasedControl::int_stiffness(0);
float accBasedControl::vel_motor_last(0);
float accBasedControl::torque_error_last(0);
float accBasedControl::IntVelBIC(0);

LowPassFilter2pFloat  accBasedControl::kfVelHumFilt(C_RATE, 7);
LowPassFilter2pFloat  accBasedControl::kfAccHumFilt(C_RATE, 7);
LowPassFilter2pFloat  accBasedControl::kfAccExoFilt(C_RATE, 7);
LowPassFilter2pFloat  accBasedControl::AccHumFilt(RATE, 15);
LowPassFilter2pFloat  accBasedControl::AccExoFilt(RATE, 15);
LowPassFilter2pFloat  accBasedControl::AccMtrFilt(RATE, 15);
LowPassFilter2pFloat  accBasedControl::TSeaFilt(C_RATE, 30);


// Control Functions //

void accBasedControl::Controller(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)		// Generic Controller
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = system_clock::now();

    //this_thread::sleep_for(microseconds(4000)); // ... 270 hz
    //this_thread::sleep_for(nanoseconds(700)); // ... 840 Hz

		m_epos->sync();	// CAN Synchronization

		// Positions Measurement:
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		
		torque_sea = TSeaFilt.apply(STIFFNESS*(theta_c - theta_l));		// tau_s = Ks*(theta_a - theta_e)
		grav_comp  = -(LOWERLEGMASS*GRAVITY*L_CG)*cos(theta_l);			// inverse dynamics, \tau_W = -M g l sin(\theta_e)

		// Motor Velocity
		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();
		// Motor Current
		m_eixo_in->ReadPDO01();
		actualCurrent = m_eixo_in->PDOgetActualCurrent();

		vel_motor_filt += 0.1016*(vel_motor - vel_motor_filt);	// SMF for 840Hz and fc 15Hz
		acc_motor = (vel_motor_filt - vel_motor_last)*C_RATE;
		vel_motor_last = vel_motor_filt;
		acc_motor = AccMtrFilt.apply(acc_motor);

#if KF_ENABLE
		// Assigning the measured states to the Sensor reading Vector
		zk << kf_torque_int, ang_vel[0], theta_l, theta_c*GEAR_RATIO, ang_vel[1], vel_motor*GEAR_RATIO;
		uk << ang_vel[0], grav_comp, 0.001f*actualCurrent; //ok
		updateKalmanFilter();
#else
		downsample++;
		if (downsample >= IMU_DELAY){
			vel_hum = ang_vel[0];
			vel_exo = ang_vel[1];
			acc_hum = (vel_hum - vel_hum_last)*RATE;
			acc_exo = (vel_exo - vel_exo_last)*RATE;
			acc_hum = AccHumFilt.apply(acc_hum);
			acc_exo = AccExoFilt.apply(acc_exo);
			vel_hum_last = vel_hum;				// VelHum_k-1 <- VelHum_k
			vel_exo_last = vel_exo;				// VelExo_k-1 <- VelExo_k
			downsample = 1;
		}
#endif

		switch (m_control_mode)
		{
		case MTC:
			//accFeedforward(); // descomentar aqui para rodar o Controle por Acc
			break;
		case ATC:
			Admittance();
			break;
		case ITC:
			Impedance();
			break;
		case STC:
			SeaFeedback();
			break;
		default:
			break;
		}

		timestamp = 1e-6*(float)duration_cast<microseconds>(system_clock::now() - timestamp_begin).count();
		downsamplelog++;
		if(downsamplelog >= LOG_DELAY){ // Logging 1/4 freq Hz
			Run_Logger();
			downsamplelog = 1;
		}
    	// forcando a rodar @ 1kHz..
		std::this_thread::sleep_until(control_t_begin + microseconds(800));
		auto control_t_end = system_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;
	}
}

void accBasedControl::accFeedforward()
{
	accbased_comp = INERTIA_EXO*acc_hum;	// human disturbance input
	torque_m =  JACT*acc_motor + Kff_acc*accbased_comp + Kp_acc*(acc_hum - acc_exo) + Ki_acc*(vel_hum - vel_exo);
	torque_m += abs(grav_comp);
	
	SetMotorTorque(torque_m);
}

void accBasedControl::Admittance()
{
	// Putting Dt from (Tsea_k - Tsea_k-1)/Dt
	// into the old C2 = (1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d / C_DT)
	float C1 = damping_d / (damping_d + stiffness_d*C_DT);
	float C2 = (1 - stiffness_d / STIFFNESS) / (C_DT*stiffness_d + damping_d);
	// Se damping = 0:
	float C1 = 0;
	float C2 = (1 - stiffness_d / STIFFNESS) / (C_DT*stiffness_d);
	
	accbased_comp =  Kff_acc*INERTIA_EXO*acc_hum + Kp_acc*(acc_hum - acc_exo) + Ki_acc*(vel_hum - vel_exo);
	float des_tsea = accbased_comp + abs(grav_comp);
	vel_adm = C1*vel_adm + C2*(des_tsea - des_tsea_last - (torque_sea - torque_sea_last));   // C2*(Tsea_d_k - Tsea_d_k-1 - (Tsea_k - Tsea_k-1))
    //vel_adm = C1*vel_adm + C2*(0 - (torque_sea - torque_sea_last));
	torque_sea_last = torque_sea; 	// Tsea_k-1 <- Tsea_k
	des_tsea_last = des_tsea;

#if KF_ENABLE
	vel_motor = vel_adm + kf_vel_hum;
#else
	vel_motor = vel_adm + vel_hum;
#endif

	SetEposVelocityLimited(vel_motor);
}

void accBasedControl::Impedance()
{
	float v_error = vel_hum - vel_exo;
	IntVelBIC = update_i(v_error, Kp_bic, (IntVelBIC > 1000), &IntVelBIC);
	torque_m =  Kd_bic*v_error + IntVelBIC;

	SetMotorTorque(torque_m);
}

void accBasedControl::SeaFeedback()
{
	float v_error = vel_hum - vel_motor_filt;
	IntVelBIC = update_i(v_error, STIFFNESS,  (IntVelBIC > 2000), &IntVelBIC);
	float des_tsea = B_EQ*v_error + IntVelBIC + abs(grav_comp);
	float torque_error =  des_tsea - torque_sea;
	torque_m =  Kp_bic*torque_error + Kd_bic*(torque_error - torque_error_last)/C_DT;
	torque_error_last = torque_error;
	
	SetMotorTorque(torque_m);
}

void accBasedControl::SetEposVelocityLimited(float speed_stp)
{
	int speed_limited = (int) constrain_float(RADS2RPM*GEAR_RATIO*speed_stp, -SPEED_CONST*VOLTAGE_MAX, SPEED_CONST*VOLTAGE_MAX);
	m_eixo_in->PDOsetVelocitySetpoint(speed_limited); // speed in RPM
	m_eixo_in->WritePDO02();
}

void accBasedControl::SetEposCurrentLimited(float current_stp)
{
	actualCurrent = (int) 1000*constrain_float(current_stp, CURRENT_MAX);
	m_eixo_in->PDOsetCurrentSetpoint(actualCurrent);	// esse argumento é em mA !!!
	m_eixo_in->WritePDO01();
}

void accBasedControl::SetMotorTorque(float torque_stp)
{
	float current_stp = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_stp; 		// in Ampere!
	actualCurrent = (int) 1000*constrain_float(current_stp, CURRENT_MAX);
	m_eixo_in->PDOsetCurrentSetpoint(actualCurrent);						// esse argumento é em mA !!!
	m_eixo_in->WritePDO01();
}

void accBasedControl::GainScan()
{
	switch (m_control_mode)
	{
	case MTC:
		gains_values = fopen("gainsAbc.txt", "rt");

		if (gains_values != NULL)
		{
			fscanf(gains_values, "Kff %f\nKp %f\nKi %f\n", &Kff_acc, &Kp_acc, &Ki_acc);
			fclose(gains_values);
		}
		break;
	case ATC:
		gains_values = fopen("gainsCAC.txt", "rt");

		if (gains_values != NULL)
		{
      		fscanf(gains_values, "Kff %f\nKp %f\nKi %f\nSTF %f\nDAM %f\n", &Kff_acc, &Kp_acc, &Ki_acc, &stiffness_d, &damping_d);
			fclose(gains_values);
		}
		break;
	case ITC:
		gains_values = fopen("gainsBIC.txt", "rt");

		if (gains_values != NULL)
		{
			fscanf(gains_values, "KP %f\nKD %f\n", &Kp_bic, &Kd_bic);
			fclose(gains_values);
		}
		break;
	case STC:
		gains_values = fopen("gainsSEA.txt", "rt");

		if (gains_values != NULL)
		{
			fscanf(gains_values, "KP %f\nKD %f\n", &Kp_bic, &Kd_bic);
			fclose(gains_values);
		}
		break;
	default:
		break;
	}
}

void accBasedControl::Recorder()
{
	logger = fopen(getLogfilename(), "a");
	if (logger != NULL)
	{
		switch (m_control_mode)
		{
		case MTC:
			fprintf(logger, "%5.6f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n",\
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, vel_motor_filt, acc_motor);
			break;
		case ATC:
    case STC:
			fprintf(logger, "%5.6f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n",\
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, torque_sea, vel_motor);
			break;
		case ITC:
			fprintf(logger, "%5.6f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n",\
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, vel_motor_filt, torque_m);
			break;
		default:
			break;
		}// everything logged in standard units (SI)
		fclose(logger);
	}

#if KF_ENABLE
	kalmanLogger();	// logging measurements, control and states
#endif
}

void accBasedControl::kalmanLogger()
{
	kfLogFile = fopen(kfLogFileName, "a");
	if(kfLogFile != NULL){
		fprintf(kfLogFile, "%5.6f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f\n",\
		timestamp, kf_pos_hum, kf_pos_exo, kf_pos_act, kf_vel_exo, kf_vel_act, kf_vel_hum, kf_acc_hum,\
		kf_acc_exo, kf_torque_int);
		fclose(kfLogFile);
	}
}

void accBasedControl::Run_Logger()
{
	if (logging.load())
	{
		if (timestamp < m_seconds)
			Recorder();
		else
			logging = false;
	}
}

void accBasedControl::UpdateControlStatus()
{
	char numbers_str[20];

	switch (m_control_mode)
	{
	case MTC:
		ctrl_word = " MTC CONTROLLER\n";
		sprintf(numbers_str, "%+5d", actualCurrent);
		ctrl_word += " Current: " + (std::string) numbers_str + " mA\n";
		sprintf(numbers_str, "%5.3f", Kff_acc);
		ctrl_word += " Kff: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kp_acc);
		ctrl_word += " Kp: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Ki_acc);
		ctrl_word += " Ki: " + (std::string) numbers_str + "\n";
		break;
	case ATC:
		ctrl_word = " ATC CONTROLLER\n";
		sprintf(numbers_str, "%+5.3f", vel_motor);
		ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm\n";
		sprintf(numbers_str, "%5.3f", stiffness_d);
		ctrl_word += " STF: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", damping_d);
		ctrl_word += " DAM: " + (std::string) numbers_str + "\n";
    	ctrl_word += "\n -> Passivity Constraints <-\n ";
		k_bar = 1 - stiffness_d / STIFFNESS;
		kd_min = damping_d*(Ki_adm / Kp_adm - 1 / JACT*(damping_d / k_bar - Kp_adm));
		ctrl_word += std::to_string(kd_min) + " < STF_d < " + std::to_string(kd_max) + "\n\n";
		break;
	case ITC:
		ctrl_word = " ITC CONTROLLER\n";
		sprintf(numbers_str, "%+5d", actualCurrent);
		ctrl_word += " Current: " + (std::string) numbers_str + " mA\n";
		sprintf(numbers_str, "%5.3f", Kff_acc);
		ctrl_word += " Kff: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kp_bic);
		ctrl_word += " Kp: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kd_bic);
		ctrl_word += " Kd: " + (std::string) numbers_str + "\n";
		break;
	case STC:
		ctrl_word = " STC CONTROLLER\n";
		sprintf(numbers_str, "%+5d", actualCurrent);
		ctrl_word += " Current: " + (std::string) numbers_str + " mA\n";
		sprintf(numbers_str, "%5.3f", Kff_acc);
		ctrl_word += " Kff: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kp_bic);
		ctrl_word += " Kp: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kd_bic);
		ctrl_word += " Kd: " + (std::string) numbers_str + "\n";
		break;
	default:
		break;
	}
	ctrl_word += " T_Sea: " + std::to_string(torque_sea) + " N.m" +\
 	" Motor Encoder " + std::to_string(180 / MY_PI*theta_c) +\
	" Leg Encoder " + std::to_string(180 / MY_PI*theta_l) + "\n";
	ctrl_word += " InvDyn: " + std::to_string(grav_comp) + " N.m ";
	ctrl_word += " AccBased: " + std::to_string(accbased_comp) + " N.m\n";
	sprintf(numbers_str, "%.2f", (1/control_t_Dt) );
  	ctrl_word += " EPOS Rate: " + (std::string) numbers_str + " Hz\n";
}

float accBasedControl::constrain_float(float val, float min, float max)
{
	if (_isnan(val)) return (min + max)/2;

	if (val < min) return min;

	if(val > max) return max;
	
	return val;
}

float accBasedControl::constrain_float(float val, float constrain)
{
	if (_isnan(val)) return 0.0f;

	float lmt = abs(constrain);

	if (val < -lmt) return -lmt;

	if(val > lmt) return lmt;
	
	return val;
}

float accBasedControl::update_i(float error, float Ki, bool limit, float *integrator)
{
	if (Ki > 0.00001f){
		if(!limit || ((*integrator > 0.0f && error < 0.0f) || (*integrator < 0.0f && error > 0.0f)) ) {
			*integrator += error * Ki * C_DT;
			*integrator = constrain_float(*integrator, 1e4f);
		}
    return *integrator;
	}
	else {
		return 0.0f;
	}
}

StateSzMtx accBasedControl::discretize_A(StateSzMtx* A, float dt)
{
	// Fourth order discretezation
  const StateSzMtx intA = dt*(*A);
	return StateSzMtx::Identity() + intA + (intA)*(intA)/2 + (intA)*(intA)*(intA)/6 + (intA)*(intA)*(intA)*(intA)/24;
	// or
	//return StateSzMtx::Identity() + A*dt + (A*dt).pow(2)/2 + (A*dt).pow(3)/6 + (A*dt).pow(4)/24;
}

ControlSzMtx accBasedControl::discretize_B(StateSzMtx* A, ControlSzMtx* B, float dt)
{
	// Fourth order discretezation
  const StateSzMtx intA = dt*(*A);
	return dt*(StateSzMtx::Identity() + intA/2 + (intA)*(intA)/6 + (intA)*(intA)*(intA)/24 + (intA)*(intA)*(intA)*(intA)/120)*(*B);
	// or
	//return dt*(StateSzMtx::Identity() + A*dt/2 + (A*dt).pow(2)/6 + (A*dt).pow(3)/24 + (A*dt).pow(4)/120)*B;
}

void accBasedControl::updateKalmanFilter()
{
	
	//-->	Kalman Filter Loop	----------------------------------------//
	// State vector is  [tau_i x_h x_e x_a \dot{x_e} \dot{x_a}]			//
	// Sensor vector is [tau_i \dot{x_h} x_e x_m \dot{x_e} \dot{x_m}]	//
	// Control vector is [\dot{x_h} tau_w I_m]							//
	//																	//
	//------------------------------------------------------------------//
	
	// Prediction
	xk = Fk*xk + Gk*uk;
	Pk = Fk*Pk*Fk.transpose() + Qk;

	// Kalman Gain
	FullPivLU<SensorSzMtx> TotalCovariance(Ck * Pk * Ck.transpose() + Rk);
	if (TotalCovariance.isInvertible()){
		KG = Pk * Ck.transpose() * TotalCovariance.inverse();
	}

	// Update
	xk = xk + KG * (zk - ( Ck*xk + Dk*uk ));
	Pk = (StateSzMtx::Identity() - KG*Ck)*Pk;

	kf_torque_int = xk(0,0);
	kf_pos_exo = xk(2,0);
	kf_pos_act = xk(3,0);
	kf_vel_act = xk(5,0);

	kf_vel_hum = (xk(1,0) - kf_pos_hum)/C_DT;
	kf_pos_hum = xk(1,0);

	float kf_vel_hum_filt = kfVelHumFilt.apply(kf_vel_hum);
	kf_acc_hum = (kf_vel_hum_filt - kf_vel_hum_last)/C_DT;
	kf_vel_hum_last = kf_vel_hum_filt;

	kf_acc_hum = kfAccHumFilt.apply(kf_acc_hum);

	kf_acc_exo = (xk(4,0) - kf_vel_exo)/C_DT;
	kf_acc_exo = kfAccExoFilt.apply(kf_acc_exo);
	kf_vel_exo = xk(4,0);
	
}

void accBasedControl::updateHumKalmanFilter()
{
	// Prediction
	KFH_xk = KFH_Fk*KFH_xk + KFH_Gk*vel_motor;
	KFH_Pk = KFH_Fk*KFH_Pk*KFH_Fk.transpose() + KFH_Qk;

	// Kalman Gain
	FullPivLU<kfhSensorMtx> TotalCovariance(KFH_Ck * KFH_Pk * KFH_Ck.transpose() + KFH_Rk);
	if (TotalCovariance.isInvertible()){
		KFH_KG = KFH_Pk * KFH_Ck.transpose() * TotalCovariance.inverse();
	}

	// remover essas var locais para globais com os sensores...
	float torque_int = 0.1234;
	float pos_hum = 0.5678;

	KFH_zk << torque_sea, torque_int, theta_l, vel_hum, pos_hum;
	// Update
	KFH_xk = KFH_xk + KFH_KG * (KFH_zk - KFH_Ck*KFH_xk);
	KFH_Pk = (kfhStateMtx::Identity() - KFH_KG*KFH_Ck)*KFH_Pk;
	
}
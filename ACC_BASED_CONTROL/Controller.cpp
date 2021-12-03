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

// qASGD-KF for IMUs AHRS:
Vector4f accBasedControl::qASGD1_qk;
Matrix4f accBasedControl::qASGD1_Pk;
Vector4f accBasedControl::qASGD2_qk;
Matrix4f accBasedControl::qASGD2_Pk;

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

void accBasedControl::accBasedController(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)		// MTC
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

    //this_thread::sleep_for(microseconds(4000)); // ... 270 hz
    this_thread::sleep_for(nanoseconds(700)); // ... 840 Hz

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
		accbased_comp = INERTIA_EXO*kf_acc_hum;	// human disturbance input
		torque_m =  JACT*acc_motor + Kff_acc*accbased_comp + Kp_acc*(kf_acc_hum - kf_acc_exo) + Ki_acc*(kf_vel_hum - kf_vel_exo);
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
    	accbased_comp = INERTIA_EXO*acc_hum;	// human disturbance input
		torque_m =  JACT*acc_motor + Kff_acc*accbased_comp + Kp_acc*(acc_hum - acc_exo) + Ki_acc*(vel_hum - vel_exo);
		torque_m += abs(grav_comp);

#endif
		setpoint_filt = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_m; // now in Ampere!
		SetEposCurrentLimited(setpoint_filt);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		timestamp = 1e-6*(float)duration_cast<microseconds>(steady_clock::now() - timestamp_begin).count();
		// Logging/Sending Data @250 Hz
		downsamplelog++;
		if(downsamplelog >= LOG_DELAY){
			Run_Logger();
			downsamplelog = 1;
		}
	}
}

void accBasedControl::CAdmittanceControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)	// ATC
{
	while (Run.load())
	{
		unique_lock<std::mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		
		//no sleep: 1600 Hz
		this_thread::sleep_for(nanoseconds(700)); // ... 840 Hz
		//this_thread::sleep_for(milliseconds(7)); // ... 160 Hz
		//this_thread::sleep_for(microseconds(4000)); // ... 270 hz

		m_epos->sync();	// CAN Synchronization

		// Positions
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		// Motor Velocity
		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();
		// Motor Current
		m_eixo_in->ReadPDO01();
		actualCurrent = m_eixo_in->PDOgetActualCurrent();

		grav_comp  = -(0.92*LOWERLEGMASS*GRAVITY*L_CG)*cos(theta_l);			// inverse dynamics, \tau_W = -M g l sin(\theta_e)

		// Putting Dt from (Tsea_k - Tsea_k-1)/Dt
		// into the old C2 = (1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d / C_DT)
		float C2 = (1 - stiffness_d / STIFFNESS) / (C_DT*stiffness_d + damping_d);
		float C1 = damping_d / (damping_d + stiffness_d*C_DT);

    	torque_sea = TSeaFilt.apply(STIFFNESS*(theta_c - theta_l));

		
#if KF_ENABLE
		// Assigning the measured states to the Sensor reading Vector
    	zk << kf_torque_int, ang_vel[0], theta_l, theta_c*GEAR_RATIO, ang_vel[1], vel_motor*GEAR_RATIO;
		uk << ang_vel[0], grav_comp, 0.001f*actualCurrent; //ok
		updateKalmanFilter();
		accbased_comp =  Kff_acc*INERTIA_EXO*kf_acc_hum + Kp_acc*(kf_acc_hum - kf_acc_exo) + Ki_acc*(kf_vel_hum - kf_vel_exo);
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
		accbased_comp =  Kff_acc*INERTIA_EXO*acc_hum + Kp_acc*(acc_hum - acc_exo) + Ki_acc*(vel_hum - vel_exo);
#endif

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

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		timestamp = 1e-6*(float)duration_cast<microseconds>(steady_clock::now() - timestamp_begin).count();
		// Logging ~250 Hz
		downsamplelog++;
		if(downsamplelog >= LOG_DELAY){
			Run_Logger();
			downsamplelog = 1;
		}
	}
}

void accBasedControl::ImpedanceControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)		// ITC
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

    //this_thread::sleep_for(microseconds(4000)); // ... 270 hz
    this_thread::sleep_for(nanoseconds(700)); // ... 840 Hz

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
		float v_error = kf_vel_hum - kf_vel_exo;
		flaot p_error = kf_pos_hum - kf_pos_exo;
		torque_m = Kd_bic*v_error + Kp_bic*p_error;
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

		float v_error = vel_hum - vel_exo;
		IntVelBIC = update_i(v_error, Kp_bic, (IntVelBIC > 1000), &IntVelBIC);
		torque_m =  Kd_bic*v_error + IntVelBIC;

#endif
		setpoint_filt = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_m; // now in Ampere!
		SetEposCurrentLimited(setpoint_filt);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		timestamp = 1e-6*(float)duration_cast<microseconds>(steady_clock::now() - timestamp_begin).count();
		// Logging ~250 Hz
		downsamplelog++;
		if(downsamplelog >= LOG_DELAY){
			Run_Logger();
			downsamplelog = 1;
		}
	}
}

void accBasedControl::SeaFeedbackControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)		// STC
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

    //this_thread::sleep_for(microseconds(4000)); // ... 270 hz
    this_thread::sleep_for(nanoseconds(700)); // ... 840 Hz

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
		float v_error = kf_vel_hum - kf_vel_act;
		float p_error = kf_pos_hum - kf_pos_act;
		float des_tsea = B_EQ*v_error + STIFFNESS*p_error + abs(grav_comp);
		float torque_error = des_tsea - torque_sea;
		torque_m =  Kp_bic*torque_error + Kd_bic*(torque_error - torque_error_last)/C_DT;
		torque_error_last = torque_error;
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

		float v_error = vel_hum - vel_motor_filt;
		IntVelBIC = update_i(v_error, STIFFNESS,  (IntVelBIC > 2000), &IntVelBIC);
		float des_tsea = B_EQ*v_error + IntVelBIC + abs(grav_comp);
		float torque_error =  des_tsea - torque_sea;
		torque_m =  Kp_bic*torque_error + Kd_bic*(torque_error - torque_error_last)/C_DT;
		torque_error_last = torque_error;

#endif
		setpoint_filt = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_m; // now in Ampere!
		SetEposCurrentLimited(setpoint_filt);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		timestamp = 1e-6*(float)duration_cast<microseconds>(steady_clock::now() - timestamp_begin).count();
		// Logging ~250 Hz
		downsamplelog++;
		if(downsamplelog >= LOG_DELAY){
			Run_Logger();
			downsamplelog = 1;
		}
	}
}

void accBasedControl::Controller(std::vector<float> &ang_vel, std::vector<float> &imus, std::condition_variable &cv, std::mutex &m)		// Generic Controller
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

	      float Dt = (float) IMU_DELAY*control_t_Dt;
		  Vector3f acc;
		  Vector3f gyro;
		  acc << imus[0], imus[1], imus[2];
		  gyro << imus[3], imus[4], imus[5];
		  updateqASGD1Kalman(gyro, acc, Dt);
		  acc << imus[6], imus[7], imus[8];
		  gyro << imus[9], imus[10], imus[11];
		  updateqASGD2Kalman(gyro, acc, Dt);
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
		if(downsamplelog >= LOG_DELAY){ // Logging ~250 Hz
			Run_Logger();
			downsamplelog = 1;
		}

		std::this_thread::sleep_until(control_t_begin + microseconds(200)); // forcando a rodar @ 5kHz
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
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",\
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, vel_motor_filt, acc_motor);
			break;
		case ATC:
    case STC:
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",\
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, torque_sea, vel_motor);
			break;
		case ITC:
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",\
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
	sprintf(numbers_str, "%4.2f", (1/control_t_Dt) );
  	ctrl_word += " EPOS Rate: " + (std::string) numbers_str + " Hz\n";
	Vector3f euler;
	//euler = quatDelta2euler(&qASGD1_qk, &qASGD2_qk)*(180 / MY_PI);
  	euler = quat2euler(&qASGD2_qk)*(180 / MY_PI);
	ctrl_word += "Euler: " + std::to_string(euler(0)) + " " + std::to_string(euler(1)) + " " + std::to_string(euler(2)) + "\n";
}

char* accBasedControl::TCPMessage()
{
	switch (m_control_mode)
		{
		case MTC:
			sprintf(tcp_message,"%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n", \
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, vel_motor_filt, acc_motor);
			break;
		case ATC:
			sprintf(tcp_message,"%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n", \
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, torque_sea, vel_motor);
			break;
		case ITC:
			sprintf(tcp_message,"%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n", \
			timestamp, vel_hum, vel_exo, acc_hum, acc_exo, theta_c, theta_l, vel_motor_filt, torque_m);
			break;
		default:
			break;
		}// everything logged in standard units (SI)
	return tcp_message;
}

void accBasedControl::SavitskyGolay(float window[], float newest_value, float* first_derivative)
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

void accBasedControl::updateqASGD1Kalman(Vector3f gyro, Vector3f acc, float Dt)
{
	//Vector4f a_b(0, acc.normalized()(0), acc.normalized()(1), acc.normalized()(2));

	float q0 = qASGD1_qk(0);
	float q1 = qASGD1_qk(1);
	float q2 = qASGD1_qk(2);
	float q3 = qASGD1_qk(3);

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

	Vector3f F_obj = Vector3f::Zero();
	//F_obj = Rot.transpose()*Vector3f(0,0,1) - acc.normalized();

	// De maneira simplificada podemos fazer apenas:
	// g = [0 0 1]^T, portanto so nos interessa a ultima coluna de C^T:
	Vector3f Z_c = Vector3f::Zero();
	Z_c << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 - q3*q3);
	F_obj = Z_c - acc.normalized();	// Eq.23

	Matrix<float,3,4> Jq;
	Jq << -2*q2, 2*q3, -2*q0, 2*q1, 
		   2*q1, 2*q0,  2*q3, 2*q2, 
		   2*q0,-2*q1, -2*q2, 2*q3;
	
	Vector4f GradF;
	GradF = Jq.transpose()*F_obj;	// Eq.25

	float mi0 = 0.001;
	float Beta = 1.00;
	float omg_norm = gyro.norm();
	float Ts = DELTA_T;

	float mi = mi0 + Beta*Ts*omg_norm; // Eq.29

	Vector4f z_k;
	z_k = qASGD1_qk - mi*GradF.normalized(); // Eq.24

	Matrix4f Q = Matrix4f::Identity()*5.476e-6;	// Usar Eq. 19...
	Matrix4f R = Matrix4f::Identity()*5.476e-6;
	Matrix4f H = Matrix4f::Identity();

	Matrix4f OmG = Matrix4f::Zero();
	OmG << 0, -gyro(0), -gyro(1), -gyro(2),
		 gyro(0), 0, gyro(2), -gyro(1),
		 gyro(1), -gyro(2), 0, gyro(0),
		 gyro(2), gyro(1), -gyro(0), 0;

	OmG = 0.5*OmG;

	Matrix4f Psi;
	Psi = (1 - ((omg_norm*Ts)*(omg_norm*Ts))/8)*Matrix4f::Identity() + 0.5*Ts*OmG;

	// Process noise covariance update (Eq. 19):
	Matrix<float,4,3> Xi;
	Xi << q0, q3, -q2,
	     -q3, q0,  q1,
		  q2, -q1, q0,
		 -q1, -q2, -q3; 

	Q = 0.5*Ts*Xi*(Matrix3f::Identity()*5.476e-6)*Xi.transpose();

	// Projection:
	qASGD1_qk = Psi*qASGD1_qk;
	qASGD1_Pk = Psi*qASGD1_Pk*Psi.transpose() + Q;

	// Kalman Gain
	Matrix4f Kg;
	FullPivLU<Matrix4f> TotalCovariance(H * qASGD1_Pk * H.transpose() + R);
	if (TotalCovariance.isInvertible()){
		Kg = qASGD1_Pk * H.transpose() * TotalCovariance.inverse();
	}
	// Update
	qASGD1_qk = qASGD1_qk + Kg * (z_k - H*qASGD1_qk);
	qASGD1_Pk = (Matrix4f::Identity() - Kg*H)*qASGD1_Pk;

}

void accBasedControl::updateqASGD2Kalman(Vector3f gyro, Vector3f acc, float Dt)
{
	float q0 = qASGD2_qk(0);
	float q1 = qASGD2_qk(1);
	float q2 = qASGD2_qk(2);
	float q3 = qASGD2_qk(3);

	Vector3f F_obj = Vector3f::Zero();
	Vector3f Z_c = Vector3f::Zero();
	// De maneira simplificada podemos fazer apenas:
	// g = [0 0 1]^T, portanto so nos interessa a ultima coluna de C^T:
	Z_c << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 - q3*q3);
	F_obj = Z_c - acc.normalized();	// Eq.23

	Matrix<float,3,4> Jq;
	Jq << -2*q2, 2*q3, -2*q0, 2*q1, 
		   2*q1, 2*q0,  2*q3, 2*q2, 
		   2*q0,-2*q1, -2*q2, 2*q3;
	
	Vector4f GradF;
	GradF = Jq.transpose()*F_obj;	// Eq.25

	float mi0 = 0.001;
	float Beta = 1.00;
	float omg_norm = gyro.norm();
	//float Ts = DELTA_T;
	float Ts = constrain_float(Dt, 1.00e-6, 0.010);

	float mi = mi0 + Beta*Ts*omg_norm; // Eq.29

	Vector4f z_k;
	z_k = qASGD2_qk - mi*GradF.normalized(); // Eq.24

	Matrix4f Q = Matrix4f::Identity()*5.476e-6;	// Usar Eq. 19...
	Matrix4f R = Matrix4f::Identity()*5.476e-6;
	Matrix4f H = Matrix4f::Identity();

	Matrix4f OmG = Matrix4f::Zero();
	OmG << 0, -gyro(0), -gyro(1), -gyro(2),
		 gyro(0), 0, gyro(2), -gyro(1),
		 gyro(1), -gyro(2), 0, gyro(0),
		 gyro(2), gyro(1), -gyro(0), 0;

	OmG = 0.5*OmG;

	Matrix4f Psi;
	Psi = (1 - ((omg_norm*Ts)*(omg_norm*Ts))/8)*Matrix4f::Identity() + 0.5*Ts*OmG;

	// Process noise covariance update (Eq. 19):
	Matrix<float,4,3> Xi;
	Xi << q0, q3, -q2,
	     -q3, q0,  q1,
		  q2, -q1, q0,
		 -q1, -q2, -q3; 

	Q = 0.5*Ts*Xi*(Matrix3f::Identity()*5.476e-6)*Xi.transpose();

	// Projection:
	qASGD2_qk = Psi*qASGD2_qk;
	qASGD2_Pk = Psi*qASGD2_Pk*Psi.transpose() + Q;

	// Kalman Gain
	Matrix4f Kg;
	FullPivLU<Matrix4f> TotalCovariance(H * qASGD2_Pk * H.transpose() + R);
	if (TotalCovariance.isInvertible()){
		Kg = qASGD2_Pk * H.transpose() * TotalCovariance.inverse();
	}
	// Update
	qASGD2_qk = qASGD2_qk + Kg * (z_k - H*qASGD2_qk);
	qASGD2_Pk = (Matrix4f::Identity() - Kg*H)*qASGD2_Pk;
}

Vector3f accBasedControl::quat2euler(Vector4f* quat)
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


Vector3f accBasedControl::quatDelta2euler(Vector4f* quat_r, Vector4f* quat_m)
{
	float qr0 = (*quat_r)(0);
	float qr1 = (*quat_r)(1);
	float qr2 = (*quat_r)(2);
	float qr3 = (*quat_r)(3);
	// q_m conjugate (*q_m):
	float qm0 =  (*quat_m)(0);
	float qm1 = -(*quat_m)(1);
	float qm2 = -(*quat_m)(2);
	float qm3 = -(*quat_m)(3);

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
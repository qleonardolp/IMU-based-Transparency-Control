///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2022      //
// leonardo.felipe.santos@usp.br					 //
// github/bitbucket qleonardolp 					 //
// --------------------------------------------------//
// Félix Maurício Escalante, 2022				     //
// ...                                               //
// ...  											 //
///////////////////////////////////////////////////////
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "LowPassFilter2p.h"
// #include <unsupported/Eigen/MatrixFunctions>

#define		RADS2RPM		(30/3.141592653f)		// rad/s to rpm 
#define		RPM2RADS		(3.141592653f/30)		// rpm to rad/s

//-- motor Maxon RE40  Specs: -- //
//	Torque Constant: 0.0603 N.m/A
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A

#define		CURRENT_MAX		3.1400f		// Max corrente nominal no motor Maxon RE40 [A]
#define		VOLTAGE_MAX		21.600f		// Max tensao de saida Vcc = 0.9*24V fornecida pela EPOS 24/5
#define		TORQUE_CONST	0.0603f		// Constante de torque do motor RE40	[N.m/A]
#define		SPEED_CONST		158.00f		// Constante de velocidade do motor RE40 [rpm/V]
#define		CM_CONST		0.00287f	// [N.m s/rad]
#define		JACT			0.14e-4f	// [Kg.m^2]	Maxon motor RE40 datasheet

// KF human-based
#define KFH_STATE_DIM  9
#define KFH_SENSOR_DIM 5
#define KF_CTRL_DIM    1

using namespace Eigen;
// KF human-based
typedef Matrix<float, KFH_STATE_DIM,  1> StateSzVec;
typedef Matrix<float, KFH_SENSOR_DIM, 1> SensorSzVec;
typedef Matrix<float, KF_CTRL_DIM,   1> ControlSzVec;
typedef Matrix<float, KFH_STATE_DIM, KFH_STATE_DIM>  StateSzMtx;
typedef Matrix<float, KFH_SENSOR_DIM, KFH_SENSOR_DIM> SensorSzMtx;

// Kalman Filter: human-based model designed with Felix M. Escalante //
StateSzVec   xk; // State Vector				[dTr Tr dTi Ti dpr pr dph ph Terr]
SensorSzVec  zk; // Sensor reading Vector	[Tr Ti pr dph ph]
ControlSzVec uk; // Control Vector 	[omega_m]
StateSzMtx  Pk;	// State Covariance Matrix
StateSzMtx  Fk;	// Transition Matrix (discrete)
StateSzVec  Gk;	// Control Matrix (discrete)
StateSzMtx  Qk;	// Process noise Covariance
SensorSzMtx Rk;	// Sensor noise Covariance
Matrix<float,KFH_SENSOR_DIM,KFH_STATE_DIM> Ck;	// Sensor Expectations Matrix
Matrix<float,KFH_STATE_DIM,KFH_SENSOR_DIM> KG;	// Kalman Gain Matrix

{
	float Nr = 150;
	float Cm = 0.00287; // [N.m s/rad]
	float Ks = 104.00;
	float Jr = 0.8768;
	float Jw = 0.4700;		// [Kg.m^2]
	float Br = 60.000;		// [N.m s/rad]
	float BR = (Br/Nr) + Nr*Cm;
	float Ka = Ks/20;
	float Ba = 0.300;
	float Jh = 0.1500;
	float Bh = 4.000;
	float Kh = 400.0;
	float Ts = 0.005;

	// Human-based Kalman Filter Setup:
	xk.setZero();
	zk.setZero();
	Fk.setZero();
	Gk.setZero();
	Ck.setZero();
	KG.setZero();
	// TODO: Ajustar
	Pk = 0.100f*StateSzMtx::Identity(); // P(k=0)
	Qk = 0.007f*StateSzMtx::Identity();
	Rk = 0.010f*SensorSzMtx::Identity();
	//...
	// "Modelo Joelho Expandido":
	StateSzMtx A = StateSzMtx::Zero();
	StateSzVec B = StateSzVec::Zero();
	// linha1
	A(0,1) = -((Ks*(Jr + Jw))/(Jw*Jr)); 
	A(0,3) = (Ks/Jr);
	A(0,4) = ((Ks*Br)/(Jr)); 
	// linha2
	A(1,0) = 1.0000f;
	// linha3
	A(2,1) = (((Ka*Jr)-(Br*Ba))/(Jr*Jr)); 
	A(2,2) = -((Ba*(Jh+Jr))/(Jr*Jh));
	A(2,3) = (((Ba*Bh -Ka*Jh)/(Jh*Jh))-((Ka*Jr -Br*Ba)/(Jr*Jr)));
	A(2,4) = -(((Br*(Ka*Jr -Br*Ba))/(Jr*Jr)) + ((Ba*Ks)/Jr));
	A(2,6) = (((Ba*Kh)/(Jh))-((Bh*(Ba*Bh - Ka*Jh))/(Jh*Jh)));
	A(2,7) = -(((Ba*Bh - Ka*Jh)*Kh)/(Jh*Jh));
	// linha4
	A(3,2) = 1.0000f;
	// linha5
	A(4,1) =  (1/Jr); 
	A(4,3) = -(1/Jr);
	A(4,4) = -(Br/Jr);
	// linha6
	A(5,0) = -1/Ks; 
	// linha7
	A(6,3) =  (1/Jh);
	A(6,6) = -(Bh/Jh);
	A(6,7) = -(Kh/Jh);
	// linha8
	A(7,6) = 1.0000f;
	// linha9
	A(8,1) = -1.0000f;
	B(0,0) = ((Ks*Cm)/(Jw));
	B(2,0) = ((Ba*Ks)/(Jr*Nr));
	B(5,0) = 1/Nr;
	// Discretizacao 4 Ord
	Fk = StateSzMtx::Identity() + A*Ts + (A*Ts).pow(2)/2 + (A*Ts).pow(3)/6 + (A*Ts).pow(4)/24;
	Gk = (StateSzMtx::Identity() + A*Ts/2 + (A*Ts).pow(2)/6 + (A*Ts).pow(3)/24 + (A*Ts).pow(4)/120)*B*Ts;
	// Sensor reading Vector zk: [Tr Ti pr dph ph]
	Ck(0,1) = 1.0000f;
	Ck(1,3) = 1.0000f;
	Ck(2,5) = 1.0000f;
	Ck(3,6) = 1.0000f;
	Ck(4,7) = 1.0000f;
}

FILE *kfLogFile;
char kfLogFileName[50];
LowPassFilter2pFloat  kfVelHumFilt(200, 8);
LowPassFilter2pFloat  kfAccHumFilt(200, 8);
LowPassFilter2pFloat  kfAccExoFilt(200, 8);
LowPassFilter2pFloat  AccHumFilt(200, 15);
LowPassFilter2pFloat  AccExoFilt(200, 15);
LowPassFilter2pFloat  AccMtrFilt(200, 15);
LowPassFilter2pFloat  TSeaFilt(200, 20);


void kalmanLogger()
{
	kfLogFile = fopen(kfLogFileName, "a");
	if(kfLogFile != NULL){
		//fprintf(kfLogFile, "%5.6f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f\n",\
		//timestamp, kf_pos_hum, kf_pos_exo, kf_pos_act, kf_vel_exo, kf_vel_act, kf_vel_hum, kf_acc_hum,\
		//kf_acc_exo, kf_torque_int);
		fclose(kfLogFile);
	}
}

float constrain_float(float val, float min, float max)
{
	if (_isnan(val)) return (min + max)/2;
	if (val < min) return min;
	if(val > max) return max;
	return val;
}

float constrain_float(float val, float constrain)
{
	if (_isnan(val)) return 0.0f;
	float lmt = abs(constrain);
	if (val < -lmt) return -lmt;
	if(val > lmt) return lmt;
	return val;
}

void updateKalmanFilter()
{
	using namespace Eigen;
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
	xk = xk + KG * (zk -Ck*xk);
	Pk = (StateSzMtx::Identity() - KG*Ck)*Pk;
}

#endif // KALMANFILTER_H

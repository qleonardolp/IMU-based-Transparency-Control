///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2022      //
// leonardo.felipe.santos@usp.br					 //
// github/bitbucket qleonardolp 					 //
// --------------------------------------------------//
// Félix Maurício Escalante, 2022				     //
// ...                                               //
// ...  											 //
///////////////////////////////////////////////////////
#include "KalmanFilter.h"


/*
void updateKalmanFilter(SensorSzVec sensor_in, ControlSzVec control_in)
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
*/
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

//		Extended Kalman Filter				    //
StateSzMtx accBasedControl::ekf_Gk;										// State transition Jacobian
StateSzMtx accBasedControl::ekf_Pk;										// State Covariance Matrix
StateSzMtx accBasedControl::ekf_Qk;	 									// Process noise Covariance
SensorSzMtx accBasedControl::ekf_Rk; 									// Sensor noise Covariance
Matrix<float, EKF_STATE_DIM, 1> accBasedControl::ekf_xk;				// State Vector				[x_h x_e \dot{x_e} x_a \dot{x_a}]
Matrix<float, EKF_SENSOR_DIM, 1> accBasedControl::ekf_zk;				// Sensor reading Vector	[\dot{x_h} x_e \dot{x_e} x_a \dot{x_a}]
Matrix<float, EKF_CTRL_DIM, 1> accBasedControl::ekf_uk; 			 	// Control Vector
Matrix<float, EKF_STATE_DIM, EKF_CTRL_DIM> 	accBasedControl::ekf_Bk;	// Control Matrix
Matrix<float, EKF_SENSOR_DIM, EKF_STATE_DIM> accBasedControl::ekf_Hk;	// Sensor expectations Jacobian
Matrix<float, EKF_STATE_DIM, EKF_SENSOR_DIM> accBasedControl::ekf_KG;	// Kalman Gain Matrix
Matrix<float, EKF_SENSOR_DIM, EKF_CTRL_DIM>	accBasedControl::ekf_Dk;	// Feedforward Matrix

float accBasedControl::int_stiffness = STIFFNESS/50;
uint8_t	accBasedControl::ekf_skip = 1;
float accBasedControl::ekfPosHum;
float accBasedControl::ekfPosExo;
float accBasedControl::ekfVelExo;
float accBasedControl::ekfPosAct;
float accBasedControl::ekfVelAct;

void accBasedControl::ekfUpdate(float velHum, float posExo, float velExo, float posAct, float velAct, float mCurrent)
{
    // Assigning measurements and control
	ekf_skip++;
	if(ekf_skip < IMU_DELAY){
		ekf_zk(0,1) = posExo;
		ekf_zk(0,3) = posAct;
		ekf_zk(0,4) = velAct;
		ekf_uk(0,1) = mCurrent;
	}
	else{
		ekf_zk << velHum, posExo, velExo, posAct, velAct; // !!! NOTE: WITHOUT GEAR_RATIO !!!
		ekf_uk << velHum, mCurrent;
		ekf_skip = 1;
	}

	ekfLogger();	// logging measurements, control and states

	// Prediction, g(x_k-1,u_k)      //
	ekf_xk(0,0) = (ekf_Bk * ekf_uk)(0,0);
	ekf_xk(0,1) = ekf_xk(0,2);
	ekf_xk(0,2) = (-(int_stiffness + STIFFNESS)*ekf_xk(0,1) + STIFFNESS*ekf_xk(0,3) + LOWERLEGMASS*GRAVITY*L_CG*sin(ekf_xk(0,1)))/INERTIA_EXO;
	ekf_xk(0,3) = ekf_xk(0,4);
	ekf_xk(0,4) = (STIFFNESS*(ekf_xk(0,1) - ekf_xk(0,3)) - B_EQ*ekf_xk(0,4))/J_EQ + (ekf_Bk * ekf_uk)(0,4);
	// Predict process covariance
	ekf_Pk = ekf_Gk * ekf_Pk * ekf_Gk.transpose() + ekf_Qk;

	// Kalman Gain	//
	static FullPivLU<SensorSzMtx> TotalCovariance(ekf_Hk * ekf_Pk * ekf_Hk.transpose() + ekf_Rk);
	if (TotalCovariance.isInvertible()){
		ekf_KG = ekf_Pk * ekf_Hk.transpose() * TotalCovariance.inverse();
	}
	// Updating		//
	ekf_xk = ekf_xk + ekf_KG * (ekf_zk - ekf_Hk*ekf_xk);
	ekf_Pk = (StateSzMtx::Identity() - ekf_KG * ekf_Hk)*ekf_Pk;

	// Put the state on class variables
	ekfPosHum = ekf_xk(0,0);
	ekfPosExo = ekf_xk(0,1);
	ekfVelExo = ekf_xk(0,2);
	ekfPosAct = ekf_xk(0,3);
	ekfVelAct = ekf_xk(0,4);

	// Update the transition Jacobian
    ekf_Gk(2, 1) = -(int_stiffness + STIFFNESS - LOWERLEGMASS*GRAVITY*L_CG*cos(ekfPosExo))/INERTIA_EXO;
}

void accBasedControl::ekfLogger()
{
	static float ekf_time = 1e-6*((float)duration_cast<microseconds>(steady_clock::now() - timestamp_begin).count());

	if (ekf_time < 60.0000f){
		ekfLogFile = fopen(strcat("./data/ekf-",header_timestamp), "a");

		if(ekfLogFile != NULL){
			fprintf(ekfLogFile, "%5.6f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f\n",\
			ekf_time, ekf_zk(0,0), ekf_zk(0,1), ekf_zk(0,2), ekf_zk(0,3), ekf_zk(0,4), ekf_uk(0,0), ekf_uk(0,1),\
			ekf_xk(0,0), ekf_xk(0,1), ekf_xk(0,2), ekf_xk(0,3), ekf_xk(0,4));
			fclose(ekfLogFile);
		}
	}
}
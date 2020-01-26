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


#include "Controller.h"

using namespace Eigen;

// Control Functions //

void accBasedControl::FiniteDiff(float velHum, float velExo)
{
	m_epos->sync();	// CAN Synchronization 


	// -- Acc-Based Torque -- //

	//vel_hum = vel_hum - 0.334*(vel_hum - velHum);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	//savitskygolay(velhumVec, velHum, &acc_hum);		// Updating window, smoothing and First Derivative
	//vel_hum = velhumVec[0];
	
	// velHum derivative using a Gain and a Discrete Integrator in loop
	vel_hum = velHum;
	acc_hum = diffCutoff*( vel_hum - IntegratorHum );
	IntegratorHum += acc_hum*(1/RATE);
	
	
	//vel_exo = vel_exo - 0.334*(vel_exo - velExo);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	//savitskygolay(velexoVec, velExo, &acc_exo);		// Updating window, smoothing and First Derivative
	//vel_exo = velexoVec[0];
	
	// velExo derivative using a Gain and a Discrete Integrator in loop
	vel_exo = velExo;
	acc_exo = diffCutoff*( vel_exo - IntegratorExo );
	IntegratorExo += acc_exo*(1/RATE);

	accbased_comp = K_ff * (INERTIA_EXO + J_EQ)*acc_hum + Kp_A * (acc_hum - acc_exo) + Ki_A * (vel_hum - vel_exo);
	savitskygolay(torqueAccVec, accbased_comp, &d_accbased_comp);
	accbased_comp = torqueAccVec[0];

	// -- SEA Torque -- //

	m_eixo_out->ReadPDO01();
	theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
	m_eixo_in->ReadPDO01();
	theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
	
	savitskygolay(theta_c_vec, theta_c, &vel_motor);
	vel_motor = GEAR_RATIO*vel_motor;
	// here, vel_motor is used as rad/s

	//torque_sea = torque_sea - 0.334*(torque_sea - STIFFNESS * (theta_c - theta_l)); // precisa?
	torque_sea = STIFFNESS * (theta_c - theta_l);
	
	savitskygolay(torqueSeaVec, torque_sea, &d_torque_sea);	

	// rever:
	setpoint = (1 / (TORQUE_CONST * GEAR_RATIO)) * (accbased_comp + J_EQ * acc_exo + B_EQ * vel_exo - Kp_F * torque_sea - Kd_F * d_torque_sea) * Amplifier;
	setpoint_filt += LPF_SMF * (setpoint - setpoint_filt);

	if (abs(setpoint_filt) < CURRENT_MAX * 1000)
	{
		if ((theta_l >= - 1.5708) && (theta_l <= 0.5236)) //(caminhando)
		{
			m_eixo_in->PDOsetCurrentSetpoint((int)setpoint_filt);	// esse argumento é em mA !!!
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint(0);
		}
	}
	else
	{
		if (setpoint_filt < 0)
		{
			m_eixo_in->PDOsetCurrentSetpoint(-(int)(CURRENT_MAX * 1000));
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint((int)(CURRENT_MAX * 1000));
		}
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();

}

void accBasedControl::OmegaControl(float velHum, float velExo)
{
	m_epos->sync();	// CAN Synchronization

  // IMPLEMENT HIGH PASS FITLER FOR GYRO BIAS

	//vel_hum = vel_hum - 0.334*(vel_hum - velHum);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	savitskygolay(velhumVec, velHum, &acc_hum);		// Updating window, smoothing and First Derivative
	vel_hum = velhumVec[0];
	
	//vel_exo = vel_exo - 0.334*(vel_exo - velExo);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	savitskygolay(velexoVec, velExo, &acc_exo);		// Updating window, smoothing and First Derivative
	vel_exo = velexoVec[0];
	
	// velHum derivative using a Gain and a Discrete Integrator in loop
	//vel_hum = velHum;
	//acc_hum = diffCutoff*( vel_hum - IntegratorHum );
	//IntegratorHum += acc_hum*(1/RATE);
	
	// velExo derivative using a Gain and a Discrete Integrator in loop
	//vel_exo = velExo;
	//acc_exo = diffCutoff*( vel_exo - IntegratorExo );
	//IntegratorExo += acc_exo*(1/RATE);
	
	// acc_hum derivative using a Gain and a Discrete Integrator in loop
	jerk_hum = 3.0f*( acc_hum - IntAccHum );	// diffCutoff == 3.0
	IntAccHum += jerk_hum*(1/RATE);
	
	// acc_exo derivative using a Gain and a Discrete Integrator in loop
	jerk_exo = 3.0f*( acc_exo - IntAccExo );
	IntAccExo += jerk_exo*(1/RATE);

  // KFF_V 200.00 KP_V 0.100  KI_V 0.000  KD_V 0.010  AMP 1
  //vel_motor = Amp_V * GEAR_RATIO * ( Kff_V*vel_hum + Kp_V*(vel_hum - vel_exo) + Kd_V*(acc_hum - acc_exo) );   // [rpm]
  
  m_eixo_in->ReadPDO02();
  actualVelocity = MY_PI/30*m_eixo_in->PDOgetActualVelocity();
  
  // vel_motor using a inverted dependency of T_sea and feedforward command. Check my own ICAR2019 poster for further details...
  torque_sea += LPF_SMF*( STIFFNESS/RATE * (actualVelocity - vel_exo) - torque_sea); 
  
  acc_motor = diffCutoff*( actualVelocity - IntAccMotor );
  IntAccMotor += acc_motor*(1/RATE);
  
  //accbased_comp = J_EQ*acc_motor + INERTIA_EXO*acc_hum + Weight + B_EQ*actualVelocity*MY_PI/30; // actual correct term
  accbased_comp = Kff_V*INERTIA_EXO*acc_hum + B_EQ*actualVelocity + J_EQ*acc_motor;
  vel_motor = Amp_V * GEAR_RATIO *( vel_hum + Kp_V*accbased_comp + Kd_V*(acc_hum - acc_exo) - Kp_V*torque_sea);   // [rad/s]
  
  // Or, Jerk Feedforward:
  //m_eixo_out->ReadPDO02();
  //exoVelocity = m_eixo_out->PDOgetActualVelocity();
  //vel_motor = vel_exo + ( INERTIA_EXO*jerk_hum + Kp_V*(jerk_hum - jerk_exo) + Ki_V*(acc_hum - acc_exo) )/STIFFNESS;
  
  vel_motor = 30/MY_PI*vel_motor;
  vel_motor_filt += LPF_SMF*(vel_motor - vel_motor_filt);

  voltage = abs(vel_motor_filt / SPEED_CONST);

  if (voltage < 0.130)			// lower saturation
  {
	  m_eixo_in->PDOsetVelocitySetpoint(0);
  }
  else if ((voltage > 0.100) && (voltage <= VOLTAGE_MAX))
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
  m_eixo_in->WritePDO02();
  
  m_eixo_in->ReadPDO02();
  actualVelocity = MY_PI/30*m_eixo_in->PDOgetActualVelocity();
}

void accBasedControl::CurrentControlKF(float velHum, float velExo)
{
	m_epos->sync();

	//	LPF_FC = 10Hz >> 0.334
	//vel_hum = vel_hum - 0.334*(vel_hum - velHum);		// [rad/s]
	//vel_exo = vel_exo - 0.334*(vel_exo - velExo);		// [rad/s]

	m_eixo_out->ReadPDO01();
	theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
	m_eixo_in->ReadPDO01();
	theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

	torque_sea = STIFFNESS *(theta_c - theta_l);	// [N.m]


	for (int i = 10; i > 0; --i)
		theta_c_vec[i] = theta_c_vec[i - 1];  // shifting values in 1 position
	theta_c_vec[0] = theta_c;
	// *Finite Difference, SG Coefficients, First Derivative, linear fit with 11 pts and +5 offset
	vel_motor = (5 * theta_c_vec[0] + 4 * theta_c_vec[1] + 3 * theta_c_vec[2] + 2 * theta_c_vec[3] +
		1 * theta_c_vec[4] + 0 * theta_c_vec[5] - 1 * theta_c_vec[6] - 2 * theta_c_vec[7] -
		3 * theta_c_vec[8] - 4 * theta_c_vec[9] - 5 * theta_c_vec[10]) / (110) * RATE;
	vel_motor = GEAR_RATIO*vel_motor;
	// here, vel_motor is used as rad/s


	// Assigning the measured states to the Sensor reading Vector
	z_k << velHum, velExo, torque_sea;

	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();

	// Predicting
	// or x_k = Fk * x_k + Bk * 0; test
	x_k = Fk * x_k + Bk * (actualCurrent * TORQUE_CONST / Amp_kf);
	Pk = Fk * Pk * Fk.transpose() + Qk;

	// Kalman Gain:
	FullPivLU<Matrix3f> TotalCovariance(Hk * Pk * Hk.transpose() + Rk);
	if (TotalCovariance.isInvertible())
	{
		KG = Pk * Hk.transpose() * TotalCovariance.inverse();
		kf_error = " ";
	}
	else
	{
		kf_error = "Total Covariance matrix (Hk Pk Hk' + Rk) has no inverse!";
	}

	// Updating
	x_k = x_k + KG * (z_k - Hk*x_k);
	Pk = Pk - KG * Hk*Pk;
	
	// Controlling using the state estimations

	acc_hum = x_k(1);
	acc_exo = x_k(3);
	vel_hum = x_k(0);
	vel_exo = x_k(2);
	torque_sea = x_k(4);
	
	// As considered for the Kalman Filter designed in Fk
	accbased_comp = (1 / GEAR_RATIO) * ((J_EQ + INERTIA_EXO)*acc_hum + (B_EQ / GEAR_RATIO)*vel_motor);

	setpoint = (1/TORQUE_CONST) * (accbased_comp) * Amp_kf;
	setpoint_filt = setpoint_filt - LPF_SMF * (setpoint_filt - setpoint);		// test without it

	if (abs(setpoint_filt) <= CURRENT_MAX * 1000)
	{
		if ((theta_c >= -0.5200) && (theta_c <= 1.4800)) // (sentado)
			//if ((theta_l >= - 1.30899) && (theta_l <= 0.26179)) //(caminhando)
			//if ((theta_l >= - 1.39626) && (theta_l <= 0.17000)) //(em pe parado)
		{
			m_eixo_in->PDOsetCurrentSetpoint((int)setpoint_filt);	// esse argumento é em mA !!!
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint(0);
		}
	}
	else
	{
		if (setpoint_filt < 0)
		{
			m_eixo_in->PDOsetCurrentSetpoint(-(int)(CURRENT_MAX * 1000));
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint((int)(CURRENT_MAX * 1000));
		}
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();
}

void accBasedControl::FFPosition(float velHum, float velExo)
{
	m_epos->sync();

	//vel_hum = vel_hum - 0.334*(vel_hum - velHum);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	savitskygolay(velhumVec, velHum, &acc_hum);		// Updating window, smoothing and First Derivative
	vel_hum = velhumVec[0];

	//vel_exo = vel_exo - 0.334*(vel_exo - velExo);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	savitskygolay(velexoVec, velExo, &acc_exo);		// Updating window, smoothing and First Derivative
	vel_exo = velexoVec[0];

	m_eixo_out->ReadPDO01();
	theta_l_vec[0] = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
	m_eixo_in->ReadPDO01();
	theta_c_vec[0] = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

	torque_sea = STIFFNESS * (theta_c_vec[0] - theta_l_vec[0]);

	theta_c = Kff_P*(INERTIA_EXO / STIFFNESS)*acc_hum + theta_l_vec[0];	
	//[kg m² . rad/ Nm . rad / s²] = [Kg m / N . rad²/s² ] = [Kg m s²/ (Kg m) rad²/s²] = [s² rad²/s²] = rad² ???

	if ((theta_c >= -0.5200) && (theta_c <= 1.4800))
	{
		// theta_c from RAD to encoder counts, considering the initial offset pos0_in
		theta_m = theta_m - LPF_SMF*( theta_m - ((ENCODER_IN * GEAR_RATIO / (2 * MY_PI)) * theta_c + pos0_in) );		// LP Filtering
		m_eixo_in->PDOsetPositionSetpoint((int)theta_m);
	}
	m_eixo_in->WritePDO01();
}

void accBasedControl::FFPositionGrav(float accHumT, float accHumR, float accExoT, float accExoR, float velHum, float velExo)
{
	m_epos->sync();

	// LP Filtering, LPF_FC = 10 Hz		(0.334)
	vel_hum = vel_hum - 0.334*(vel_hum - velHum);
	vel_exo = vel_exo - 0.334*(vel_exo - velExo);

	accHum_R = accHum_R - 0.334*(accHum_R - accHumR);
	accHum_T = accHum_T - 0.334*(accHum_T - accHumT);

	accExo_R = accExo_R - 0.334*(accExo_R - accExoR);
	accExo_T = accExo_T - 0.334*(accExo_T - accExoT);

	/*
	theta_l_vec[2] = acos((MTW_DIST_LIMB*vel_hum*vel_hum - accHum_R) / GRAVITY);
	if ((theta_l_vec[2] >= -0.5200) && (theta_l_vec[2] <= 1.4800))
	{
		acc_hum = (1 / MTW_DIST_LIMB)*(accHum_T - GRAVITY*sin(theta_l_vec[2]));
	} // deu ruim
	*/

	m_eixo_out->ReadPDO01();
	theta_l_vec[0] = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
	m_eixo_in->ReadPDO01();
	theta_c_vec[0] = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

	torque_sea = STIFFNESS * (theta_c_vec[0] - theta_l_vec[0]);

	acc_hum = (1 / MTW_DIST_LIMB)*(accHum_T - GRAVITY * sin(theta_l_vec[0]));	// testar! equivalente ao FFPosition

	theta_c = Kff_P*(INERTIA_EXO / STIFFNESS)*acc_hum + theta_l_vec[0];
	//[kg m² . rad/ Nm . rad / s²] = [Kg m / N . rad²/s² ] = [Kg m s²/ (Kg m) rad²/s²] = [s² rad²/s²] = rad² ???

	if ((theta_c >= -0.5200) && (theta_c <= 1.4800))
	{
		// theta_c from RAD to encoder counts, considering the initial offset pos0_in
		theta_m = theta_m - LPF_SMF * (theta_m - ((ENCODER_IN * GEAR_RATIO / (2 * MY_PI)) * theta_c + pos0_in));		// LP Filtering
		m_eixo_in->PDOsetPositionSetpoint((int)theta_m);
	}
	m_eixo_in->WritePDO01();
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
		fscanf(gains_values, "AMP %d\n", &Amp_kf);
		fclose(gains_values);
	}
}

void accBasedControl::GainScan_Velocity()
{
	gains_values = fopen("gainsSpeed.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "KFF_V %f\nKP_V %f\nKI_V %f\nKD_V %f\nAMP %f\n", &Kff_V, &Kp_V, &Ki_V, &Kd_V, &Amp_V);
		fclose(gains_values);
	}
}

void accBasedControl::GainScan_Position()
{
	gains_values = fopen("gainsPosition.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "KFF_P %f\nKP_P %f\nKI_P %f\nKD_P %f\nAMP %f\n", &Kff_P, &Kp_P, &Ki_P, &Kd_P, &Amp_P);
		fclose(gains_values);
	}
}

void accBasedControl::UpdateCtrlWord_Current()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+5.3f", setpoint_filt);
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
	sprintf(numbers_str, "%+5.3f", (2*MY_PI/60) * vel_motor);
	ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm\n";
}

void accBasedControl::UpdateCtrlWord_CurrentKF()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+.3f", setpoint_filt);
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
	sprintf(numbers_str, "%.3f", Amp_kf);
	ctrl_word += " Amplifier: " + (std::string) numbers_str + "\n";
	sprintf(numbers_str, "%+5.3f", (2 * MY_PI / 60) * vel_motor);
	ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm\n";
	ctrl_word += " " + kf_error + "\n";
}


void accBasedControl::UpdateCtrlWord_Velocity()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+5.3f", vel_motor);
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
	sprintf(numbers_str, "%5.3f", Amp_V);
	ctrl_word += " Amplifier: " + (std::string) numbers_str + "\n";
}

void accBasedControl::UpdateCtrlWord_Position()
{
	char numbers_str[20];

	sprintf(numbers_str, "%+5.3f", theta_l_vec[0] * (180 / MY_PI));
	ctrl_word = " theta_l: " + (std::string) numbers_str + " deg";
	sprintf(numbers_str, "%+5.3f", theta_c_vec[0] * (180 / MY_PI));
	ctrl_word += " theta_c: " + (std::string) numbers_str + " deg";
	sprintf(numbers_str, "%+5.3f", torque_sea);
	ctrl_word += " T_sea: " + (std::string) numbers_str + " N.m\n";
	sprintf(numbers_str, "%+5.3f", theta_c * (180 / MY_PI));
	ctrl_word += " theta_m: " + (std::string) numbers_str + " deg\n";
}

void accBasedControl::Recorder_Current()
{
	if (logging)
	{
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5d   %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",
				setpoint_filt, actualCurrent, theta_l * (180 / MY_PI), theta_c * (180 / MY_PI), accbased_comp, acc_hum, acc_exo, vel_hum, vel_exo, vel_motor);
			fclose(logger);
		}
	}

}

void accBasedControl::Recorder_Velocity()
{
	if (logging)
	{
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5d\n",
				acc_hum, acc_exo, vel_hum, vel_exo, vel_motor_filt, acc_motor, actualVelocity);
				// everything logged in standard units (SI)
			fclose(logger);
		}
	}
	
}

void accBasedControl::Recorder_Position()
{
	if (logging)
	{
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",
				acc_hum, acc_exo, vel_hum, vel_exo, theta_c, theta_c_vec[0], theta_l_vec[0]);
			fclose(logger);
		}
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
///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2019     ///
// leonardo.felipe.santos@usp.br	_____ ___  ___   //
// github/bitbucket qleonardolp		| |  | \ \/   \  //
////////////////////////////////	| |   \ \   |_|  //
////////////////////////////////	\_'_/\_`_/__|    //
///////////////////////////////////////////////////////

#include "currentController.h"

// Control Functions Descriptions //

void accBasedControl::FiniteDiff(float velHum, float velExo)
{
	m_epos->sync();	// CAN Synchronization 


	// -- Acc-Based Torque -- //

	vel_hum = vel_hum - 0.334*(vel_hum - velHum);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	for (int i = 10; i > 0; --i)
	{
		velhumVec[i] = velhumVec[i - 1];  // shifting values in 1 position
	}

	// *Savitsky-Golay Smoothing Coefficients, 4th order fit with 11 pts and +5 offset from the centre point
	// 143 |	6	-10	-5	5	10	6	-5	-15	-10	30	131
	velhumVec[0] = (6 * velhumVec[10] - 10 * velhumVec[9] - 5 * velhumVec[8]
		+ 5 * velhumVec[7] + 10 * velhumVec[6] + 6 * velhumVec[5]
		- 5 * velhumVec[4] - 15 * velhumVec[3] - 10 * velhumVec[2]
		+ 30 * velhumVec[1] + 131 * vel_hum) / 143;

	vel_hum = velhumVec[0];

	// *Finite Difference Coefficients Calculator:
	//acc_hum = (-2 * velhumVec[3] + 9 * velhumVec[2] - 18 * velhumVec[1] + 11 * velhumVec[0]) / (6)*RATE;  // Using the last 4 pts
	acc_hum = (velhumVec[0] - velhumVec[1])*RATE;                                                       // Using the last 2 pts

	vel_exo = vel_exo - 0.334*(vel_exo - velExo);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	for (int i = 10; i > 0; --i)
	{
		velexoVec[i] = velexoVec[i - 1];  // shifting values in 1 position
	}

	// *Savitsky-Golay Smoothing Coefficients, 4th order fit with 11 pts and +5 offset from the centre point
	velexoVec[0] = (6 * velexoVec[10] - 10 * velexoVec[9] - 5 * velexoVec[8]
		+ 5 * velexoVec[7] + 10 * velexoVec[6] + 6 * velexoVec[5]
		- 5 * velexoVec[4] - 15 * velexoVec[3] - 10 * velexoVec[2]
		+ 30 * velexoVec[1] + 131 * vel_exo) / 143;

	vel_exo = velexoVec[0];

	// *Finite Difference Coefficients Calculator:
	//acc_exo = (-2 * velexoVec[3] + 9 * velexoVec[2] - 18 * velexoVec[1] + 11 * velexoVec[0]) / (6)*RATE;  // Using the last 4 pts
	acc_exo = (velexoVec[0] - velexoVec[1])*RATE;                                                       // Using the last 2 pts

	for (int i = 3; i > 0; --i)
	{
		torqueAccVec[i] = torqueAccVec[i - 1];  // shifting values in 1 position
	}
	accbased_comp = K_ff * (4 * INERTIA_EXO)*acc_hum + Kp_A * (acc_hum - acc_exo) + Ki_A * (vel_hum - vel_exo);
	torqueAccVec[0] = torqueAccVec[0] - 0.334*(torqueAccVec[0] - accbased_comp);	// LP Filtering, LPF_FC  = 10 Hz  0.334

	// *Finite Difference Coefficients Calculator:
	d_accbased_comp = (-2 * torqueAccVec[3] + 9 * torqueAccVec[2] - 18 * torqueAccVec[1] + 11 * torqueAccVec[0]) / (6)*RATE;
	//d_accbased_comp = (torqueAccVec[0] - torqueAccVec[0])*RATE;



	  // -- SEA Torque -- //

	m_eixo_out->ReadPDO01();
	theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]

	//m_eixo_out->ReadPDO02();
	//vel_leg = -m_eixo_out->PDOgetActualVelocity();

	m_eixo_in->ReadPDO01();
	theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

	torque_sea = torque_sea - 0.334*(torque_sea - STIFFNESS * (theta_c - theta_l));
	//torque_sea = STIFFNESS*(theta_c - theta_l);

	for (int i = 3; i > 0; --i)
	{
		torqueSeaVec[i] = torqueSeaVec[i - 1];  // shifting values in 1 position
	}
	torqueSeaVec[0] = torque_sea;
	d_torque_sea = (-2 * torqueSeaVec[3] + 9 * torqueSeaVec[2] - 18 * torqueSeaVec[1] + 11 * torqueSeaVec[0]) / (6)*RATE;
	//d_torque_sea = (torqueSeaVec[0] - torqueSeaVec[1])*RATE;



	//setpoint = (1 / TORQUE_CONST) * (1 / GEAR_RATIO) * (Kp_F*(accbased_comp - torque_sea) + Kd_F*(d_accbased_comp - d_torque_sea)) * Amplifier;

	setpoint = (1 / (TORQUE_CONST * GEAR_RATIO)) * (accbased_comp + J_EQ * acc_exo + B_EQ * vel_exo - Kp_F * torque_sea - Kd_F * d_torque_sea) * Amplifier;
	setpoint_filt = setpoint_filt - LPF_SMF * (setpoint_filt - setpoint);

	if ((setpoint_filt >= -CURRENT_MAX * 1000) && (setpoint_filt <= CURRENT_MAX * 1000))
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
			m_eixo_in->PDOsetCurrentSetpoint(-(int)CURRENT_MAX * 1000);
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint((int)CURRENT_MAX * 1000);
		}
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();

}


void accBasedControl::Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z)
{
	m_epos->sync();

	float sin_thetag = (velExo_Z*velExo_Z*MTW_DIST_EXO - accExo_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_exo = ((-1)*accExo_Y + GRAVITY * sqrt(1 - sin_thetag)) / MTW_DIST_EXO;
	}

	sin_thetag = (velHum_Z*velHum_Z*MTW_DIST_LIMB - accHum_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_hum = (accHum_Y + GRAVITY * sqrt(1 - sin_thetag)) / MTW_DIST_LIMB;
	}

	accbased_comp = (INERTIA_EXO)*acc_hum + KP_A * (acc_hum - acc_exo) + KI_A * (velHum_Z - velExo_Z);


	m_eixo_out->ReadPDO01();
	theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
	d_torque_sea = (STIFFNESS*(theta_c - theta_l) - torque_sea)*RATE;
	torque_sea = STIFFNESS * (theta_c - theta_l);

	// NOT WORKING
	setpoint = (1 / TORQUE_CONST) * (1 / GEAR_RATIO) * (10000 * accbased_comp);// - KP_F*torque_sea - KD_F*d_torque_sea );
	setpoint_filt = setpoint_filt - LPF_SMF * (setpoint_filt - setpoint);

	printf("setpt: %7.3f", setpoint_filt);

	if ((setpoint_filt >= -CURRENT_MAX * 1000) && (setpoint_filt <= CURRENT_MAX * 1000))
	{
		if ((theta_c >= -0.70000) && (theta_c <= 0.70000))
		{
			m_eixo_in->PDOsetCurrentSetpoint((int)setpoint_filt);	// esse argumento é em mA
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint(0);
		}
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();
}

void accBasedControl::OmegaControl(float velHum, float velExo)
{
	m_epos->sync();	//Sincroniza a CAN

	vel_hum = vel_hum - 0.334*(vel_hum - velHum);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	for (int i = 10; i > 0; --i)
	{
		velhumVec[i] = velhumVec[i - 1];  // shifting values in 1 position
	}

	// *Savitsky-Golay Smoothing Coefficients, 4th order fit with 11 pts and +5 offset from the centre point
	// 143 |	6	-10	-5	5	10	6	-5	-15	-10	30	131
	velhumVec[0] = (6 * velhumVec[10] - 10 * velhumVec[9] - 5 * velhumVec[8]
		+ 5 * velhumVec[7] + 10 * velhumVec[6] + 6 * velhumVec[5]
		- 5 * velhumVec[4] - 15 * velhumVec[3] - 10 * velhumVec[2]
		+ 30 * velhumVec[1] + 131 * vel_hum) / 143;

	vel_hum = velhumVec[0];

	// *Finite Difference Coefficients Calculator:
	//acc_hum = (-2 * velhumVec[3] + 9 * velhumVec[2] - 18 * velhumVec[1] + 11 * velhumVec[0]) / (6)*RATE;  // Using the last 4 pts
	acc_hum = (velhumVec[0] - velhumVec[1])*RATE;                                                       // Using the last 2 pts

	vel_exo = vel_exo - 0.334*(vel_exo - velExo);		// LP Filtering, LPF_FC  = 10 Hz  0.334
	for (int i = 10; i > 0; --i)
	{
		velexoVec[i] = velexoVec[i - 1];  // shifting values in 1 position
	}

	// *Savitsky-Golay Smoothing Coefficients, 4th order fit with 11 pts and +5 offset from the centre point
	velexoVec[0] = (6 * velexoVec[10] - 10 * velexoVec[9] - 5 * velexoVec[8]
		+ 5 * velexoVec[7] + 10 * velexoVec[6] + 6 * velexoVec[5]
		- 5 * velexoVec[4] - 15 * velexoVec[3] - 10 * velexoVec[2]
		+ 30 * velexoVec[1] + 131 * vel_exo) / 143;

	vel_exo = velexoVec[0];

	// *Finite Difference Coefficients Calculator:
	//acc_exo = (-2 * velexoVec[3] + 9 * velexoVec[2] - 18 * velexoVec[1] + 11 * velexoVec[0]) / (6)*RATE;  // Using the last 4 pts
	acc_exo = (velexoVec[0] - velexoVec[1])*RATE;                                                       // Using the last 2 pts

	float offset = 26;    // [rpm]
	vel_motor = Amp_V * GEAR_RATIO * (2 * MY_PI / 60) * vel_hum + offset;   // [rpm]
	voltage = vel_motor / SPEED_CONST;
	if (abs(voltage) <= VOLTAGE_MAX)
	{
		m_eixo_in->PDOsetVelocitySetpoint((int)vel_motor);
	}
	else
	{
		if (vel_motor < 0)
		{
			m_eixo_in->PDOsetVelocitySetpoint(-(int)SPEED_CONST*VOLTAGE_MAX);
		}
		else
		{
			m_eixo_in->PDOsetVelocitySetpoint((int)SPEED_CONST*VOLTAGE_MAX);
		}
	}
	m_eixo_in->WritePDO02();

	m_eixo_in->ReadPDO02();
	actualVelocity = m_eixo_in->PDOgetActualVelocity();
}

void accBasedControl::GainsScan_Current()
{
	gains_values = fopen("gains_values_Current.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "K_FF %f\nKP_A %f\nKI_A %f\nKP_F %f\nKD_F %f\nAMP %d\n", &K_ff, &Kp_A, &Ki_A, &Kp_F, &Kd_F, &Amplifier);
		fclose(gains_values);
	}
}

void accBasedControl::GainsScan_Velocity()
{
	gains_values = fopen("gains_values_Speed.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "KFF_V %f\nKP_V %f\nKI_V %f\nKD_V %f\nAMP %d\n", &Kff_V, &Kp_V, &Ki_V, &Kd_V, &Amp_V);
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
}


void accBasedControl::UpdateCtrlWord_Velocity()
{
	char numbers_str[20];
	sprintf(numbers_str, "%+5.3f", vel_motor);
	ctrl_word = " vel_motor: " + (std::string) numbers_str + " rpm";
	sprintf(numbers_str, "%+5d", actualVelocity);
	ctrl_word += " [" + (std::string) numbers_str + " rpm	";
	sprintf(numbers_str, "%+5.3f", abs(actualVelocity / SPEED_CONST));
	ctrl_word += (std::string) numbers_str + " V]\n";
	sprintf(numbers_str, "%5.3f", Kff_V);
	ctrl_word += " Kff_V: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Kp_V);
	ctrl_word += " Kp_V: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Ki_V);
	ctrl_word += " Ki_V: " + (std::string) numbers_str;
	sprintf(numbers_str, "%5.3f", Kd_V);
	ctrl_word += " Kd_V: " + (std::string) numbers_str + "\n";
	sprintf(numbers_str, "%5d", Amp_V);
	ctrl_word += " Amplifier: " + (std::string) numbers_str + "\n";
}

void accBasedControl::Recorder_Current()
{
	if (logging)
	{
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5d   %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",
				setpoint_filt, actualCurrent, theta_l * (180 / MY_PI), theta_c * (180 / MY_PI), accbased_comp, acc_hum, acc_exo, vel_hum, vel_exo);
			fclose(logger);
		}
	}
	//return NULL;
}

void accBasedControl::Recorder_Velocity()
{
	if (logging)
	{
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5d  %5.3f\n",
				acc_hum, acc_exo, vel_hum, vel_exo, vel_motor, actualVelocity, abs(actualVelocity / SPEED_CONST));
			fclose(logger);
		}
	}
	//return NULL;
}
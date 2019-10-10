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
	m_epos->sync();	//Sincroniza a CAN
	m_eixo_in->VCS_SetOperationMode(CURRENT_MODE);


  // Acc-Based Torque//
	vel_hum = vel_hum - LPF_SMF*(vel_hum - velHum);
	for (int i = 3; i > 0; --i)
	{
		velhumVec[i] = velhumVec[i - 1];
	}
	velhumVec[0] = vel_hum;
	acc_hum = (-2*velhumVec[3] + 9*velhumVec[2] - 18*velhumVec[1] + 11*velhumVec[0]) / (6)*RATE;


	vel_exo = vel_exo - LPF_SMF*(vel_exo - velExo);
	for (int i = 3; i > 0; --i)
	{
		velexoVec[i] = velexoVec[i - 1];
	}
	velexoVec[0] = vel_exo;
	acc_exo = (-2*velexoVec[3] + 9*velexoVec[2] - 18*velexoVec[1] + 11*velexoVec[0]) / (6)*RATE;


  for (int i = 3; i > 0; --i)
	{
		torqueAccVec[i] = torqueAccVec[i - 1];
	}
	torqueAccVec[0] = K_ff*(4 * INERTIA_EXO)*acc_hum + Kp_A*(acc_hum - acc_exo) + Ki_A*(vel_hum - vel_exo);
  accbased_comp = torqueAccVec[0];

	d_accbased_comp = (-2*torqueAccVec[3] + 9*torqueAccVec[2] - 18*torqueAccVec[1] + 11*torqueAccVec[0]) / (6)*RATE;
  // Acc-Based Torque//



  // SEA Torque //
	m_eixo_out->ReadPDO01();
	theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

	torque_sea = torque_sea - LPF_SMF*(torque_sea - STIFFNESS*(theta_c - theta_l)); // smmooooooooooth operaaatoorr
  
	for (int i = 3; i > 0; --i)
	{
		torqueSeaVec[i] = torqueSeaVec[i - 1];
	}
  torqueSeaVec[0] = torque_sea;
	d_torque_sea = (-2*torqueSeaVec[3] + 9*torqueSeaVec[2] - 18*torqueSeaVec[1] + 11*torqueSeaVec[0]) / (6)*RATE;
  // SEA Torque //

  //setpoint = (1 / TORQUE_CONST) * (1 / GEAR_RATIO) * (-Kp_F*(accbased_comp - torque_sea) - Kd_F*(d_accbased_comp - d_torque_sea)) * Amplifier;

  setpoint = (1/(TORQUE_CONST * GEAR_RATIO)) * (accbased_comp + J_EQ*acc_exo + B_EQ*vel_exo + Kp_F*torque_sea + Kd_F*d_torque_sea) * Amplifier;
	setpoint_filt = setpoint_filt - LPF_SMF*(setpoint_filt - setpoint);

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
			if (theta_c < -0.5200)
			{
				m_eixo_in->PDOsetPositionSetpoint((int) -0.5200 / (2 * MY_PI) * ENCODER_IN * GEAR_RATIO + pos0_in);
				//m_eixo_in->WritePDO01();
			}
			if (theta_c > 1.4800)
			{
				m_eixo_in->PDOsetPositionSetpoint((int) 1.4800 / (2 * MY_PI) * ENCODER_IN * GEAR_RATIO + pos0_in);
				//m_eixo_in->WritePDO01();
			}
		}
	}
	else
	{
		if (setpoint_filt < 0)
		{
			m_eixo_in->PDOsetCurrentSetpoint( -(int)CURRENT_MAX*1000 );
		}
		else
		{
			m_eixo_in->PDOsetCurrentSetpoint( (int)CURRENT_MAX * 1000 );
		}
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();

	/*
	if (logging && (log_count != 0))
	{
		--log_count;
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5d   %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5d\n", 
			setpoint_filt, actualCurrent, theta_l * (180 / MY_PI), theta_c * (180 / MY_PI), torque_sea, accbased_comp, K_ff, Kp_A, Ki_A, Kp_F, Kd_F, Amplifier);
			fclose(logger);
		}
	}
	*/
}


void accBasedControl::Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z)
{
	m_epos->sync();

	float sin_thetag = (velExo_Z*velExo_Z*MTW_DIST_EXO - accExo_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_exo = ((-1)*accExo_Y + GRAVITY*sqrt(1 - sin_thetag)) / MTW_DIST_EXO;
	}

	sin_thetag = (velHum_Z*velHum_Z*MTW_DIST_LIMB - accHum_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_hum = (accHum_Y + GRAVITY*sqrt(1 - sin_thetag)) / MTW_DIST_LIMB;
	}

	accbased_comp = (INERTIA_EXO)*acc_hum + KP_A*(acc_hum - acc_exo) + KI_A*(velHum_Z - velExo_Z);


	m_eixo_out->ReadPDO01();
	theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
	d_torque_sea = (STIFFNESS*(theta_c - theta_l) - torque_sea)*RATE;
	torque_sea = STIFFNESS * (theta_c - theta_l);

	// NOT WORKING
	setpoint = (1 / TORQUE_CONST) * (1 / GEAR_RATIO) * (10000 * accbased_comp);// - KP_F*torque_sea - KD_F*d_torque_sea );
	setpoint_filt = setpoint_filt - LPF_SMF*(setpoint_filt - setpoint);

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
	printf(" %5d mA theta_l: %5.3f deg theta_c: %5.3f deg T_sea: %5.3f N.m T_acc: %-5.3f N.m \n", m_eixo_in->PDOgetActualCurrent(), theta_l * (180 / MY_PI), theta_c * (180 / MY_PI), torque_sea, accbased_comp);
}

void accBasedControl::OmegaControl(float velHum, float velExo)
{
  m_epos->sync();	//Sincroniza a CAN
  //m_eixo_in->VCS_SetOperationMode(VELOCITY_MODE);

  m_eixo_in->VCS_SetOperationMode(POSITION_MODE);

  m_eixo_out->ReadPDO01();
  theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;			// [rad]

  m_eixo_in->ReadPDO01();
  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

  torque_sea = torque_sea - LPF_SMF*(torque_sea - STIFFNESS*(theta_c - theta_l)); // smmooooooooooth operaaatoorr

  m_epos->sync();
  m_eixo_in->VCS_SetOperationMode(VELOCITY_MODE);

  vel_hum = vel_hum - LPF_SMF*(vel_hum - velHum);

  float offset = 26;    // [rpm]
  vel_motor = 170 * GEAR_RATIO * (2*MY_PI/60) * vel_hum + offset;   // [rpm]
  voltage = vel_motor / SPEED_CONST;
  if (abs(voltage) <= VOLTAGE_MAX)
  {
    m_eixo_in->PDOsetVelocitySetpoint( (int)vel_motor );
  }
  else
  {
    if (vel_motor < 0)
    {
      m_eixo_in->PDOsetVelocitySetpoint( -(int)SPEED_CONST*VOLTAGE_MAX );
    }
    else
    {
      m_eixo_in->PDOsetVelocitySetpoint( (int)SPEED_CONST*VOLTAGE_MAX );
    }
  }
  m_eixo_in->WritePDO02();

  m_eixo_in->ReadPDO02();
  actualVelocity = m_eixo_in->PDOgetActualVelocity();
}

void accBasedControl::GainsScan()
{
	gains_values = fopen("gains_values.txt", "rt");

	if (gains_values != NULL)
	{
		fscanf(gains_values, "K_FF %f\nKP_A %f\nKI_A %f\nKP_F %f\nKD_F %f\nAMP %d\n", &K_ff, &Kp_A, &Ki_A, &Kp_F, &Kd_F, &Amplifier);
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
  ctrl_word += " [" + (std::string) numbers_str + " rpm]\n";
  sprintf(numbers_str, "%+5.3f", torque_sea);
  ctrl_word += " T_sea: " + (std::string) numbers_str + " N.m\n";
  sprintf(numbers_str, "%+5.3f", abs(actualVelocity / SPEED_CONST) );
  ctrl_word += " Voltage: " + (std::string) numbers_str + " V\n";
}

void* accBasedControl::Recorder()
{
	if (logging)
	{
		logger = fopen(logger_filename, "a");
		if (logger != NULL)
		{
			fprintf(logger, "%5.3f  %5d   %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5d\n",
				setpoint_filt, actualCurrent, theta_l * (180 / MY_PI), theta_c * (180 / MY_PI), torque_sea, accbased_comp, K_ff, Kp_A, Ki_A, Kp_F, Kd_F, Amplifier);
			fclose(logger);
		}
	}
}
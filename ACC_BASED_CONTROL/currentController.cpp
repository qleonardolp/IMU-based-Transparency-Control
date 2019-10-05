#include "currentController.h"
#include <stdio.h>
#include <math.h>

// Control Functions Descriptions //

void accBasedControl::FiniteDiff(float velHum, float velExo)
{
	m_epos->sync();	//Sincroniza a CAN

	vel_hum = vel_hum - LPF_SMF*( vel_hum - velHum);
	acc_hum = (vel_hum - vel_hum_ant)*RATE;
	vel_hum_ant = vel_hum;

	vel_exo = vel_exo- LPF_SMF*( vel_exo - velExo);
	acc_exo = (vel_exo - vel_exo_ant)*RATE;
	vel_exo_ant = vel_exo;

	m_eixo_out->ReadPDO01();
	theta_l = ( (float) (-m_eixo_out->PDOgetActualPosition()- pos0_out)/ENCODER_OUT )*2*MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ( (float) (m_eixo_in->PDOgetActualPosition()-pos0_in )/(ENCODER_IN * GEAR_RATIO) )*2*MY_PI;	// [rad

  torque_sea = torque_sea - LPF_SMF*( torque_sea - STIFFNESS*(theta_c - theta_l) ); // smmooooooooooth operaaatoorr
  d_torque_sea = (torque_sea - torque_sea_ant)*RATE;
  torque_sea_ant = torque_sea;

	//grav_comp = (INERTIA_EXO + 0.038)*GRAVITY*(0.50)*sin(theta_l);
    accbased_comp = K_ff*(4 * INERTIA_EXO)*acc_hum + Kp_A*(acc_hum - acc_exo) + Ki_A*(vel_hum - vel_exo);

    setpoint = (1/TORQUE_CONST) * (1/GEAR_RATIO) * ( accbased_comp + Kp_F*torque_sea + Kd_F*d_torque_sea );
	setpoint = Amplifier * setpoint;
	//setpoint = 300000 * setpoint;

	setpoint_filt = setpoint_filt - LPF_SMF*( setpoint_filt - setpoint); // does it really needs to be filtered again?  TEST IT!

	if ( (setpoint_filt >= - CURRENT_MAX*1000) && (setpoint_filt <= CURRENT_MAX*1000) )
	{
    if ((theta_l >= - 0.43633) && (theta_l <= 1.13446)) // -80 deg <= theta_l (perna) <= 10 deg // - 1.39626 theta_l 0.17453
    {
       m_eixo_in->PDOsetCurrentSetpoint( (int)setpoint_filt );	// esse argumento � em mA !!!
    }
    else
    {
      m_eixo_in->PDOsetCurrentSetpoint( 0 );
    }
	}
	m_eixo_in->WritePDO01();

    m_eixo_in->ReadPDO01();
    int actualCurrent = m_eixo_in->PDOgetActualCurrent();

    char numbers_str[20];

    sprintf(numbers_str, "%+5.3f", setpoint_filt);
    ctrl_word = " setpt: " + (std::string) numbers_str;
    sprintf(numbers_str, "%+5d", actualCurrent);
    ctrl_word += " [" + (std::string) numbers_str + " mA]\n";
    sprintf(numbers_str, "%+5.3f", theta_l * (180/MY_PI));
    ctrl_word += " theta_l: " + (std::string) numbers_str + " deg";
    sprintf(numbers_str, "%+5.3f", theta_c * (180/MY_PI));
    ctrl_word += " theta_c: " + (std::string) numbers_str + " deg\n";
    sprintf(numbers_str, "%+5.3f", torque_sea);
    ctrl_word += " T_sea: " + (std::string) numbers_str + " N.m";
    sprintf(numbers_str, "%+5.3f", accbased_comp);
    ctrl_word += " T_acc: " + (std::string) numbers_str + " N.m\n";
    sprintf(numbers_str, "%6.4f", K_ff);
    ctrl_word += " K_ff: " + (std::string) numbers_str;
    sprintf(numbers_str, "%6.4f", Kp_A);
    ctrl_word += " Kp_A: " + (std::string) numbers_str;
    sprintf(numbers_str, "%6.4f", Ki_A);
    ctrl_word += " Ki_A: " + (std::string) numbers_str + "\n";
    sprintf(numbers_str, "%6.4f", Kp_F);
    ctrl_word += " Kp_F: " + (std::string) numbers_str;
    sprintf(numbers_str, "%6.4f", Kd_F);
    ctrl_word += " Kd_F: " + (std::string) numbers_str + "\n";
    sprintf(numbers_str, "%5d", Amplifier);
    ctrl_word += " Amp: " + (std::string) numbers_str + "\n";
    
    /*
    ctrl_word = " setpt: " + std::to_string(setpoint_filt) +
                " [" + std::to_string(actualCurrent) + " mA]\n" + 
                " theta_l: " + std::to_string(theta_l * (180/MY_PI)) + " deg" + 
                " theta_c: " + std::to_string(theta_c * (180/MY_PI)) + " deg\n" + 
                " T_sea: " + std::to_string(torque_sea) + " N.m" + 
                " T_acc: " + std::to_string(accbased_comp) + " N.m\n" +
                " K_ff: " + std::to_string(K_ff) +
				        " Kp_A: " + std::to_string(Kp_A) +
                " Ki_A: " + std::to_string(Ki_A) + "\n" + 
                " Kp_F: " + std::to_string(Kp_F) + 
                " Kd_F: " + std::to_string(Kd_F) + "\n" +
				        " Amp: " + std::to_string(Amplifier) + "\n";
    */   
}


void accBasedControl::Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z)
{
	m_epos->sync();
	
	float sin_thetag = (velExo_Z*velExo_Z*MTW_DIST_EXO - accExo_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_exo = ( (-1)*accExo_Y + GRAVITY*sqrt(1 - sin_thetag) ) / MTW_DIST_EXO;
	}

	sin_thetag = (velHum_Z*velHum_Z*MTW_DIST_LIMB - accHum_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_hum = ( accHum_Y + GRAVITY*sqrt(1 - sin_thetag) ) / MTW_DIST_LIMB;
	}

	accbased_comp = (INERTIA_EXO)*acc_hum + KP_A*(acc_hum - acc_exo) + KI_A*(velHum_Z - velExo_Z);


	m_eixo_out->ReadPDO01();
	theta_l = ( (float) (-m_eixo_out->PDOgetActualPosition()- pos0_out)/ENCODER_OUT )*2*MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ( (float) (m_eixo_in->PDOgetActualPosition()-pos0_in )/(ENCODER_IN * GEAR_RATIO) )*2*MY_PI;	// [rad]
	d_torque_sea = ( STIFFNESS*(theta_c - theta_l) - torque_sea )*RATE;
	torque_sea = STIFFNESS * (theta_c - theta_l);

  // NOT WORKING
	setpoint = (1/TORQUE_CONST) * (1/GEAR_RATIO) * ( 10000 * accbased_comp );// - KP_F*torque_sea - KD_F*d_torque_sea );
	setpoint_filt = setpoint_filt - LPF_SMF*( setpoint_filt - setpoint);

	printf("setpt: %7.3f", setpoint_filt);

	if ( (setpoint_filt >= - CURRENT_MAX*1000) && (setpoint_filt <= CURRENT_MAX*1000) )
	{
    if ((theta_c >= - 0.70000) && (theta_c <= 0.70000))
    {
       m_eixo_in->PDOsetCurrentSetpoint( (int)setpoint_filt );	// esse argumento � em mA
    }
    else
    {
      m_eixo_in->PDOsetCurrentSetpoint( 0 );
    }
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	printf(" %5d mA theta_l: %5.3f deg theta_c: %5.3f deg T_sea: %5.3f N.m T_acc: %-5.3f N.m \n", m_eixo_in->PDOgetActualCurrent(), theta_l * (180/MY_PI), theta_c * (180/MY_PI), torque_sea, accbased_comp);
}

void accBasedControl::Gains_Scan()
{
  gains_values = fopen("gains_values.txt", "rt");

  if (gains_values != NULL)
  {
	  fscanf(gains_values, "K_FF %f\nKP_A %f\nKI_A %f\nKP_F %f\nKD_F %f\nAMP %d\n", &K_ff, &Kp_A, &Ki_A, &Kp_F, &Kd_F, &Amplifier);
	  fclose(gains_values);
  }
}

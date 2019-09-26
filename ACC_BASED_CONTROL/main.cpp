/*	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.	Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2.	Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3.	Neither the names of the copyright holders nor the names of their contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/////////////////////////////////////////////////////////////////////////
// Adapted by Leonardo Felipe L. S. dos Santos, 2019 (@qleonardolp)    //
/////////////////////////////////////////////////////////////////////////

#include <WinSock2.h>

#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <utility>
#include <string>
#include <list>
#include <set>

//#include <thread>

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "generalheader.h"

#include "findClosestUpdateRate.h"
#include "mastercallback.h"
#include "mtwcallback.h"

#include <xsensdeviceapi.h> // The Xsens device API header 
#include <xsens/xsmutex.h>
#include "xstypes.h"
#include <conio.h>
#include <math.h>
#include <chrono>



void Habilita_Eixo(int ID);

void Desabilita_Eixo(int ID);

class accBasedControl
{
public:
	accBasedControl(int zero_out, int zero_in)
	{ 
		vel_hum = 0;
		vel_exo = 0;
		vel_hum_ant = 0;
		vel_exo_ant = 0;
		torque_sea = 0;

		pos0_out = zero_out;
		pos0_in = zero_in;
	}
	~accBasedControl()
	{
		eixo_in.PDOsetCurrentSetpoint(0);
		eixo_in.WritePDO01();
	}

	void currentControl(float accHum, float accExo, float velHum, float velExo)
	{
		epos.sync();	//Sincroniza a CAN

		vel_hum = vel_hum - LPF_SMF*( vel_hum - velHum);
		acc_hum = (vel_hum - vel_hum_ant)*RATE;
		vel_hum_ant = vel_hum;

		vel_exo = vel_exo- LPF_SMF*( vel_exo - velExo);
		acc_exo = (vel_exo - vel_exo_ant)*RATE;
		vel_exo_ant = vel_exo;

		eixo_out.ReadPDO01();
		theta_l = ( (float) (-eixo_out.PDOgetActualPosition()- pos0_out)/ENCODER_OUT )*2*MY_PI;				// [rad]

		eixo_in.ReadPDO01();
		theta_c = ( (float) (eixo_in.PDOgetActualPosition()-pos0_in )/(ENCODER_IN * GEAR_RATIO) )*2*MY_PI;	// [rad]
		d_torque_sea = ( STIFFNESS*(theta_c - theta_l) - torque_sea )*RATE;
		torque_sea = STIFFNESS * (theta_c - theta_l);

		grav_comp = (INERTIA_EXO + 0.038)*GRAVITY*(0.50)*sin(theta_l);
    accbased_comp = INERTIA_EXO*acc_hum + KP_A*(acc_hum - acc_exo) + KI_A*(vel_hum - vel_exo);

    setpoint = (1/TORQUE_CONST) * (1/GEAR_RATIO) * ( accbased_comp + KP_F*torque_sea + KD_F*d_torque_sea );
    setpoint = 700000 * setpoint;

		printf("setpt: %7.3f", setpoint);

		if ( (setpoint >= - CURRENT_MAX*1000) && (setpoint <= CURRENT_MAX*1000) )
		{
			eixo_in.PDOsetCurrentSetpoint( (int)setpoint );	// esse argumento � em mA
		}
		eixo_in.WritePDO01();

    eixo_in.ReadPDO01();
    printf(" %5d mA theta_l: %5.3f deg theta_c: %5.3f deg T_sea: %5.3f N.m T_grav: %5.3f N.m T_acc: %-5.3f N.m ", eixo_in.PDOgetActualCurrent(), theta_l * (180/MY_PI), theta_c * (180/MY_PI), torque_sea, grav_comp, accbased_comp);
	}
private:
	float acc_hum;			// [rad/s^2]
	float acc_exo;			// [rad/s^2]

	float vel_hum;			// [rad/s]
	float vel_exo;			// [rad/s]
	float vel_hum_ant;		// [rad/s]
	float vel_exo_ant;		// [rad/s]

	float setpoint;			// [mA]

	float theta_l;			  // [rad]
	float theta_c;			  // [rad]
	float torque_sea;		  // [N.m]
	float d_torque_sea;	  // [N.m/s]
  float accbased_comp;  // [N.m]
	float grav_comp;		  // [N.m]
	int pos0_out;
	int pos0_in;
	
};

/*
void Control_Corrente(float accHum, float accExo, float velHum, float velExo)
{
	//Sincroniza a CAN
	epos.sync();

		//	Torque Constant: 0.0603 N.m/A = 60.3 N.m/mA
		//	Speed Constant: 158 rpm/V
		//	Max current (@ 48 V)  ~3.1 A
		//	Stall current (@ 48 V)  42.4 A

	float setpoint = (1/TORQUE_CONST) * (1/GEAR_RATIO) * ( INERTIA_EXO*accHum + KP*(1/0.250)*(accHum - accExo) + KI*(velHum - velExo) );
	setpoint = 1000000 * setpoint;

	printf("setpoint: %8.4f", setpoint);

	if ( (setpoint >= - CURRENT_MAX*1000) && (setpoint <= CURRENT_MAX*1000) )
	{
		eixo_in.PDOsetCurrentSetpoint( (int)setpoint );	// esse argumento � em mA
	}
	eixo_in.WritePDO01();

	eixo_in.ReadPDO01();
	printf(" current: %5d [mA]  Encoder: %5d\n", eixo_in.PDOgetActualCurrent(), eixo_in.PDOgetActualPosition());

}
*/

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const & p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString()
		;
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const & d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}



int main(int argc, char** argv)
{
	QueryPerformanceFrequency(&TICKS_PER_SECOND);
	ticksSampleTime= TICKS_PER_SECOND.QuadPart * SAMPLE_TIME;

	//START DE TRANSMISS�O DA REDE CAN
	epos.StartPDOS(1);
	epos.StartPDOS(2);
	epos.StartPDOS(3);
	epos.StartPDOS(4);
	epos.StartPDOS(5);
	epos.StartPDOS(1);
	epos.StartPDOS(2);
	epos.StartPDOS(3);
	epos.StartPDOS(4);
	epos.StartPDOS(5);

	std::cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << std::endl;

	//Inicializando a comunica��o com os eixos
	for (int i = 0; i < 10; i++)
	{
		//Aguarda tempo
		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait)
		{
		}

		//Sincroniza as epos
		epos.sync();

		eixo_out.ReadPDO01();
		eixo_in.ReadPDO01();

		printf(".");
	}

	std::cout << "Resetando Falhas " << std::endl;

	//ZERA CONTADOR DE CICLOS
	total_time = 0;

	//EPOS 01
	eixo_out.PDOsetControlWord_FaultReset(true);
	eixo_out.WritePDO01();

	//printf("\nResetando as falhas.");

	endwait = clock () + 2 * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}

	std::cout << "..";

	//EPOS 01
	eixo_out.PDOsetControlWord_FaultReset(false);
	eixo_in.WritePDO01();

	std::cout << "..";

	endwait = clock () + 2 * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}

	printf("..");

	//EPOS 02
	eixo_in.PDOsetControlWord_FaultReset(true);
	eixo_in.WritePDO01();

	std::cout << "..";

	endwait = clock () + 2 * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}

	std::cout << "..";

	//EPOS 02
	eixo_in.PDOsetControlWord_FaultReset(false);
	eixo_in.WritePDO01();

	std::cout << "..";

	endwait = clock () + 2 * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}

	std::cout << "OK" << std::endl;

	// ---------------------------------------------  Xsens Awinda Station management  ---------------------------------------------- //


	/*

	| MTw  | desiredUpdateRate (max) |
	|------|-------------------------|
	|  1   |           150 Hz        |
	|  2   |           120 Hz        |
	|  4   |           100 Hz        |
	|  6   |            75 Hz        |
	|  12  |            50 Hz        |
	|  18  |            40 Hz        |

	*/

	const int desiredUpdateRate = 120;						// Use 120 Hz update rate for MTw, 150 Hz usually crashes!
	const int desiredRadioChannel = 25;						// Use radio channel 25 for wireless master.

	WirelessMasterCallback wirelessMasterCallback;			// Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks;					// Callbacks for mtw devices

	//std::cout << "Constructing XsControl..." << std::endl;
	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
		//return -1;
	}

	try
	{
		//std::cout << "Scanning ports..." << std::endl;
		XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		//std::cout << "Finding wireless master..." << std::endl;
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw std::runtime_error("No wireless masters found");
		}
		std::cout << "Wireless master found @ " << *wirelessMasterPort << std::endl;

		std::cout << "Opening port..." << std::endl;
		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			std::ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		//std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

		//std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		//std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates: ";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled..." << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw std::runtime_error(error.str());
			}
		}

		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

		bool quitOnMTw = false;
		bool waitForConnections = true;

		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTWs: " << nextCount << ". Press 'y' to start measurement or 'q' to quit \n";
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
			if (_kbhit())
			{
				char keypressed = _getch();
				if ('y' == keypressed)
					waitForConnections = false;
				if ('q' == keypressed)
				{
					quitOnMTw = true;
					waitForConnections = false;
				}
			}
		}
		while (waitForConnections);

		if (quitOnMTw)
		{
			wirelessMasterDevice->gotoConfig();
			wirelessMasterDevice->disableRadio();
			throw std::runtime_error("quit by user request");
		}

		std::cout << "Starting measurement..." << std::endl;
		if (!wirelessMasterDevice->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		//std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		//std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
		}

		std::vector<XsVector> accData(mtwCallbacks.size());
		std::vector<XsVector> gyroData(mtwCallbacks.size());

		//Sincroniza as epos
		epos.sync();

		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}


		//Habilita o controle de corrente nos servomotores
		eixo_in.VCS_SetOperationMode(CURRENT_MODE);

		eixo_out.ReadPDO01();
		eixo_in.ReadPDO01();

		Habilita_Eixo(2);

		// Fun��o de Controle (loop MTw + Controle) ...
		epos.sync();
		eixo_out.ReadPDO01();
		eixo_in.ReadPDO01();
		accBasedControl xsens2Eposcan(-eixo_out.PDOgetActualPosition(), eixo_in.PDOgetActualPosition());

		std::cout << "Loop de Controle, pressione qualquer tecla para interromper!" << std::endl;

		while (!_kbhit())
		{
			XsTime::msleep(0);

			bool newDataAvailable = false;
			std::chrono::high_resolution_clock::time_point mtw_data_stamp;

			for (size_t i = 0; i < mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i]->dataAvailable())
				{
					mtw_data_stamp = std::chrono::high_resolution_clock::now();

					newDataAvailable = true;
					XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();

					accData[i] = packet->calibratedAcceleration();
					gyroData[i] = packet->calibratedGyroscopeData();

					mtwCallbacks[i]->deleteOldestPacket();
				}
			}

			if (newDataAvailable)
			{

				xsens2Eposcan.currentControl((float) accData[0].value(1), 
											 (float) accData[1].value(1), 
											 (float) gyroData[0].value(2), 
											 (float) gyroData[1].value(2));

				auto control_stamp = std::chrono::high_resolution_clock::now();
				float delay = std::chrono::duration_cast<std::chrono::microseconds>(control_stamp - mtw_data_stamp).count();
				printf("%.2f us\n", delay);   // printing the delay

			}
		}
		(void)_getch();

		xsens2Eposcan.~accBasedControl();

		//Zera o comando do motor
		eixo_in.PDOsetCurrentSetpoint(0);
		eixo_in.WritePDO01();

		//Desabilita o Eixo
		Desabilita_Eixo(0);


		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio())
		{
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}
	}
	catch (std::exception const & ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	//FINALIZA A COMUNICA��O COM AS EPOS
	epos.StopPDOS(1);

	endwait = clock () + 2 * CLOCKS_PER_SEC;
	while (clock() < endwait) {}

	std::cout << "Successful exit." << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl; 
	std::cin.get();

	return 0;
}



/* EPOS FUNCTIONS */

void Habilita_Eixo(int ID)
{

	if ((ID==2) | (ID==0))
	{

		eixo_in.PDOsetControlWord_SwitchOn(false);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();

		printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");

		endwait = clock () + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		eixo_in.PDOsetControlWord_SwitchOn(true);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();

		endwait = clock () + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		eixo_in.PDOsetControlWord_SwitchOn(true);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(true);
		eixo_in.WritePDO01();

	}

}


void Desabilita_Eixo(int ID)
{

	if ((ID==2) | (ID==0))
	{
		printf("\nDESABILITANDO O MOTOR E CONTROLE");

		eixo_in.PDOsetControlWord_SwitchOn(true);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();

		endwait = clock () + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		eixo_in.PDOsetControlWord_SwitchOn(false);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();

	}

}


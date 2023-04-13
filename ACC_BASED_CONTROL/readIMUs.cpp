//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // inclui <stdio.h> / <thread> / <mutex> / <condition_variable> / mastercallback.h / mtwcallback.h
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>
#include <conio.h>
#include <string>
#include <random>


// Copyright (c) 2003-2016 Xsens Technologies B.V.
// or subsidiaries worldwide. All rights reserved.

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator<<(std::ostream& out, XsPortInfo const& p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", "
		<< "ID: " << p.deviceId().toString().toStdString();
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator<<(std::ostream& out, XsDevice const& d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

void readIMUs(ThrdStruct& data_struct)
{
	using namespace std;
	
	vector<string> imus_names;
	imus_names.push_back("00B412DF");
	imus_names.push_back("00B410D2");
	imus_names.push_back("00B41244");
	imus_names.push_back("00B4108C");
	imus_names.push_back("00B412CC");
	imus_names.push_back("00B410EE");

	vector<Eigen::Matrix3f> Rotate;
	Eigen::Matrix3f R;

	R << 0, -1, 0, 
		 0, 0, -1, 
		 1, 0,  0;
	Rotate.push_back(R); // pe dir

	R << 0, 0, 1,
		 0,-1, 0,
		 1, 0, 0;
	Rotate.push_back(R); // canela dir

	// R idem
	Rotate.push_back(R); // coxa dir

	R << 0, -1, 0,
		 0, 0, -1,
		 1, 0,  0;
	Rotate.push_back(R); // pe esq

	R << 0, 0, 1,
		 0,-1, 0,
		 1, 0, 0;
	Rotate.push_back(R); // canela esq

	// R idem
	Rotate.push_back(R); // coxa esq

#if IMU_DBG_LOG
	char filename[] = "./data/im_debug_log.txt";
	FILE* logFileHandle = fopen(filename, "w");
	if (logFileHandle != NULL) fclose(logFileHandle);
#endif

	{
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false; // not ready
		*data_struct.param1A_ = false;  // not aborting
		*data_struct.param3F_ = false;  // not finished
	}

	/* Xsens Awinda Station management
	__________________________________
	| MTw  | desiredUpdateRate (max) |
	|------|-------------------------|
	|  1   |           150 Hz        |
	|  2   |           120 Hz        |
	|  4   |           100 Hz        |
	|  6   |            75 Hz        |
	|  12  |            50 Hz        |
	|__18__|____________40 Hz________| */

	int desiredUpdateRate;
	switch (NUMBER_OF_IMUS)
	{
	case 1:
		desiredUpdateRate = 150;
		break;
	case 2:
		desiredUpdateRate = 120;
		break;
	case 4:
		desiredUpdateRate = 75; // 100; // using 75 due to low performance!
		break;
	case 6:
		desiredUpdateRate =  50; // 75; // using 50 due to low performance!
		break;
	case 12:
		desiredUpdateRate =  50;
		break;
	case 18:
		desiredUpdateRate =  40;
		break;
	default:
		desiredUpdateRate =  50;
		break;
	}
	const int desiredRadioChannel = 25;
	const float sampleTime = 1 / float(desiredUpdateRate);

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	vector<MtwCallback*> mtwCallbacks;            // Callbacks for mtw devices

	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
	}

#ifdef QASGD_THREAD_DEBUG

	float imus_data[DTVC_SZ] = { 0 };
	std::default_random_engine generator;
	std::normal_distribution<float> dist(9.81, 0.5);

	{   // readIMUs nao espera as outras threads:
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = true; // readIMUs avisa que esta pronto!
		std::cout << "-> 'Reading' IMUs!\n";
	}

	std::cout << "Update Rate: " << desiredUpdateRate << std::endl;

	looptimer xsensTimer(sampleTime, data_struct.exectime_);
	int sampleT_us = data_struct.sampletime_ * MILLION;

	// inicializar looptimer:
	xsensTimer.start();
	do
	{
		auto begin_timestamp = chrono::steady_clock::now();
		/*
		imus_data[1] = 0.1;
		imus_data[7] = 0.1;
		imus_data[9] = 0.5;
		imus_data[13] = 0.1;
		imus_data[19] = 0.1;
		imus_data[23] = dist(generator);
		{
			std::unique_lock<mutex> _(*data_struct.mtx_);
			std::memcpy(*data_struct.datavec_, imus_data, (DTVC_SZ * sizeof(float)));
		}
		*/

		for (size_t i = 0; i < NUMBER_OF_IMUS; i++)
		{
			std::unique_lock<mutex> _(*data_struct.mtx_vector_[i]);
			for (int k = 0; k < IMU_DATA_SZ; k++) {
				size_t idx = i*IMU_DATA_SZ + k;
				imus_data[idx] = 10*(i+1)+(k+1);
				*data_struct.datavec_[idx] = imus_data[idx];
			}
			data_struct.cv_vector_[i]->notify_one();
		}

		this_thread::sleep_until(begin_timestamp + chrono::microseconds(sampleT_us));
	} while (!xsensTimer.end());
#else

	try
	{
		XsPortInfoArray SerialPorts = XsScanner::enumerateSerialPorts();
		XsPortInfoArray::iterator SPt = SerialPorts.begin();
		while (SPt != SerialPorts.end())
		{
			cout << *SPt << endl;
			++SPt;
		}

		XsPortInfoArray detectedDevices = XsScanner::scanPorts();
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw runtime_error("No wireless masters found");
		}
		cout << "Wireless master found @ " << *wirelessMasterPort << endl;

		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw runtime_error(error.str());
		}
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
			ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw runtime_error(error.str());
		}
		if (!wirelessMasterDevice->gotoConfig())
		{
			ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}

		//detectedDevices.clear();

		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();
		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw runtime_error(error.str());
			}
		}
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}

		cout << "Waiting for MTW to wirelessly connect..." << endl;

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
					cout << "Number of connected MTWs: " << nextCount << ". Press 'y' to start measurement or 'q' to quit \n";
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
		} while (waitForConnections);

		if (quitOnMTw)
		{
			wirelessMasterDevice->gotoConfig();
			wirelessMasterDevice->disableRadio();
			throw runtime_error("quit by user request");
		}

		if (!wirelessMasterDevice->gotoMeasurement())
		{
			ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}

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
				throw runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		cout << "Attaching callback handlers to MTWs..." << endl;

		XsDevicePtrArray mtwDevicesOrdered(mtwDevices.size());
		mtwCallbacks.resize(mtwDevices.size());

		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			int idx_unordered;
			// loop through mtwDevices to find the index according to the desired imus_names[i] (ordered)
			for (int j = 0; j < (int)mtwDevices.size(); ++j) 
			{
				string check_name = mtwDevices[j]->deviceId().toString().toStdString();
				if (check_name.compare(imus_names[i]) == 0)
				{
					idx_unordered = j;
					break;
				}
			}

			mtwDevicesOrdered[i] = mtwDevices[idx_unordered];
			mtwCallbacks[i] = new MtwCallback(i, mtwDevicesOrdered[i]);
			mtwDevicesOrdered[i]->addCallbackHandler(mtwCallbacks[i]);

			{
				unique_lock<mutex> lock(*data_struct.mtx_);
				data_struct.xs_callbacks[i] = mtwCallbacks[i]; // aqui funciona a passagem de endereco
			}

			string display_name = mtwDevicesOrdered[i]->deviceId().toString().toStdString();
			string imu_placement;

			switch (i)
			{
			case 0:
				imu_placement = "IMU1 Pe Direito: ";
				break;
			case 1:
				imu_placement = "IMU2 Canela Direita: ";
				break;
			case 2:
				imu_placement = "IMU3 Coxa Direita: ";
				break;
			case 3:
				imu_placement = "IMU4 Pe Esquerdo: ";
				break;
			case 4:
				imu_placement = "IMU5 Canela Esquerda: ";
				break;
			case 5:
				imu_placement = "IMU6 Coxa Esquerda: ";
				break;
			default:
				break;
			}
			cout << imu_placement << display_name << "\n";
		}

		{
			unique_lock<mutex> lock(*data_struct.mtx_);
			*data_struct.param3A_ = true;
		}

		vector<XsVector> accData(mtwCallbacks.size());
		vector<XsVector> gyroData(mtwCallbacks.size());
		vector<XsEuler> eulerData(mtwCallbacks.size());
		uint32_t print_cntr = 0;

		float imus_data[DTVC_SZ] = { 0 };

		// Low Pass Filters:
		LowPassFilter2pFloat imu_filters[DTVC_SZ];
		for (int i = 0; i < DTVC_SZ; i++)
		{
			imu_filters[i].set_cutoff_frequency(float(desiredUpdateRate), LPF_CUTOFF);
			imu_filters[i].reset();
		}

		{   // readIMUs nao espera as outras threads:
			unique_lock<mutex> _(*data_struct.mtx_);
			*data_struct.param0A_ = true; // readIMUs avisa que esta pronto!
			cout << "-> Reading IMUs!\n";
		}

		looptimer xsensTimer(sampleTime, data_struct.exectime_);
		int sampleT_us = data_struct.sampletime_ * MILLION;

		// inicializar looptimer:
		xsensTimer.start();
		do
		{
			auto begin_timestamp = chrono::steady_clock::now();
			// IMU connection check for safety
			// Avoid wirelessMasterCallback here, I dont know if their mutex is the same of
			// mtwCallbacks!!!
			//int imus_connected = wirelessMasterCallback.getWirelessMTWs().size();
			if ((int)mtwCallbacks.size() < 2)
			{
				unique_lock<mutex> _(*data_struct.mtx_);
				*data_struct.param1A_ = true; // aborting
				break; // get out of the reading loop!
			}

			for (size_t i = 0; i < (int)mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i] == NULL) 
					continue;

				bool newDataAvailable = false;
				if (mtwCallbacks[i]->dataAvailable())
				{
					newDataAvailable = true;

					//XsDataPacket const* packet = mtwCallbacks[i]->getOldestPacket();
					XsDataPacket packet = mtwCallbacks[i]->fetchOldestPacket();
					if (packet.containsCalibratedGyroscopeData())
						gyroData[i] = packet.calibratedGyroscopeData();

					if (packet.containsCalibratedAcceleration())
						accData[i] = packet.calibratedAcceleration();
#ifdef IMU_ATT_LOG
					if (packet.containsOrientation())
						eulerData[i] = packet.orientationEuler();
#endif

					//mtwCallbacks[i]->deleteOldestPacket();
				}

				if (newDataAvailable) {
					// Orientacao Perna DIR: [-3 2 1]
					// Orientacao Perna ESQ: [3 -2 1]
					// Orientacao Pes:		 [2 -1 3]
					// Avoid gyroData[i][k] or gyroData[i].at(k) or gyroData[i].value(k)
					// due to the 'assert' inside these operators on xsvector.h !!!
					// Old IMUs package names: 00342322 | 00342323 | 00342324

					vector<XsReal> gyroVector = gyroData[i].toVector();
					vector<XsReal> accVector = accData[i].toVector();

					Eigen::Vector3f gyroRotated = Rotate[i] * Eigen::Vector3f(gyroVector[0], gyroVector[1], gyroVector[2]);
					Eigen::Vector3f accRotated = Rotate[i] * Eigen::Vector3f(accVector[0], accVector[1], accVector[2]);
					
					std::unique_lock<mutex> _(*data_struct.mtx_vector_[i]);
					for (size_t k = 0; k < (IMU_DATA_SZ - 3); k++)
					{
						size_t idx = i * IMU_DATA_SZ + k;
						*data_struct.datavec_[idx] = imu_filters[idx].apply(gyroRotated(k));
					}

					for (size_t k = 3; k < IMU_DATA_SZ; k++)
					{
						size_t idx = i * IMU_DATA_SZ + k;
						*data_struct.datavec_[idx] = imu_filters[idx].apply(accRotated(k-3));
					}
					data_struct.cv_vector_[i]->notify_one();
#if IMU_DBG_LOG
					logFileHandle = fopen(filename, "a");
					if (logFileHandle != NULL) {
						float timestamp = float(xsensTimer.micro_now()) / MILLION;
						fprintf(logFileHandle, "%.5f", timestamp);
						for (size_t i = 0; i < 18; i++)
							fprintf(logFileHandle, ", %.4f", imus_data[i]);
						fprintf(logFileHandle, "\n");
						fclose(logFileHandle);
					}
#endif // IMU_DBG_LOG

					/*
					for (int k = 0; k < IMU_DATA_SZ; k++) {
						size_t idx = i * IMU_DATA_SZ + k;
						*data_struct.datavec_[idx] = imus_data[idx];
					}
					data_struct.cv_vector_[i]->notify_one();
					*/
					if (data_struct.param39_ == OPMODE::IMU_BYPASS_CONTROL) {
						unique_lock<mutex> _(*data_struct.mtx_);
						*(*data_struct.datavecB_ + 1) = imus_data[12]; // hum_rgtknee_vel
					}
				}
			}

			this_thread::sleep_until(begin_timestamp + chrono::microseconds(sampleT_us));
		} while (!xsensTimer.end());

		if (!wirelessMasterDevice->gotoConfig())
		{
			ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}
		if (!wirelessMasterDevice->disableRadio())
		{
			ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}
	}
	catch (exception const& ex)
	{
		cout << ex.what() << endl;
		cout << "****ABORT****" << endl;
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false; // not ready anymore
		*data_struct.param1A_ = true; // aborting
		*data_struct.param3F_ = true; // finished
		return;
	}
	catch (...)
	{
		cout << "An unknown fatal error has occured. Aborting." << endl;
		cout << "****ABORT****" << endl;
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false; // not ready anymore
		*data_struct.param1A_ = true; // aborting
		*data_struct.param3F_ = true; // finished
		return;
	}
#endif
	std::cout << "Closing XsControl..." << endl;
	control->close();

	for (vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i) {
		delete (*i);
	}

	{
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false;
		*data_struct.param3F_ = true;
	}
}

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
		return 0;

	if (supportedUpdateRates.size() == 1)
		return supportedUpdateRates[0];

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);
		if ((uRateDist == -1) || (currDist < uRateDist)) {
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}
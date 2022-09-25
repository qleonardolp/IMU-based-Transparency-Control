//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <chrono>

void Logging(ThrdStruct& data_struct) {
	using namespace std;
#if PRIORITY
	SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

	// setup stuff...
	char filename[] = "./data/log_thread.txt";
	FILE* logFileHandle = fopen(filename, "w");
	if (logFileHandle != NULL) fclose(logFileHandle);

	float rad2deg = 180 / (M_PI);
	float log_states[DTVCA_SZ];
	float log_gains[DTVC_SZ];
	float log_ftsensor[DTVCF_SZ];

	bool isready_imu(false);
	bool aborting_imu(false);
	bool aborting_ctr(false);
	bool logging_abort(false);

	do {
		{   // Loggging confere IMU
			unique_lock<mutex> _(*data_struct.mtx_);
			isready_imu = *data_struct.param0A_;
			aborting_imu = *data_struct.param1A_;
			aborting_ctr = *data_struct.param1C_;
			if (aborting_imu || aborting_ctr)
			{
				logging_abort = *data_struct.param1D_ = true;
				break;
			}
		}
	} while (!isready_imu);

	if (logging_abort) {
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param3F_ = true;
		return; // quit!
	}

	{   // Loggging avisa que esta pronto!
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0D_ = true;
		*data_struct.param3F_ = false;
	}

	looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
	auto begin_timestamp = Timer.micro_now();
	// inicializa looptimer
	Timer.start();
	do
	{
		Timer.tik();

		{   // sessao critica:
			unique_lock<mutex> _(*data_struct.mtx_);
			memcpy(log_gains, *data_struct.datavec_, sizeof(log_gains));
			memcpy(log_states, *data_struct.datavecA_, sizeof(log_states));
			memcpy(log_ftsensor, *data_struct.datavecF_, sizeof(log_ftsensor));
		}   // fim da sessao critica

		logFileHandle = fopen(filename, "a");
		if (logFileHandle != NULL) {
			float timestamp = float(Timer.micro_now() - begin_timestamp) / MILLION;
			fprintf(logFileHandle, "%.5f", timestamp);
			for (size_t i = 0; i < DTVCA_SZ; i++) {
				fprintf(logFileHandle, ", %.4f", log_states[i]);
			}
#ifdef IMU_ATT_LOG
			for (size_t i = 24; i < (DTVC_SZ-3); i++) {
				fprintf(logFileHandle, ", %.4f", log_gains[i]);
			}
#endif
			fprintf(logFileHandle, "\n");
			fclose(logFileHandle);
		}
		Timer.tak();
	} while (!Timer.end());

	{
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0D_ = false;
		*data_struct.param3F_ = true;
	}
}
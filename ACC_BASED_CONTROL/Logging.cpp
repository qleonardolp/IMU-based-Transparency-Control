//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>

void Logging(ThrdStruct& data_struct) {
	using namespace std;
#if PRIORITY
	SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

	// setup stuff...
	char filename[] = "./data/log_thread.csv";
	FILE* logFileHandle = fopen(filename, "w");
	if (logFileHandle != NULL) {
		fclose(logFileHandle);
	}

	float log_states[DTVCA_SZ] = { 0 };
	float log_gains[DTVC_SZ] = { 0 };
	float log_ftsensor[DTVCF_SZ] = { 0 };

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

	size_t n_lines = (data_struct.exectime_ / data_struct.sampletime_); // samples
	vector<string> string_vector(n_lines, " ");
	char temp_string[256];
	float timestamp(0);
	size_t  counter(0);


	looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
	int sampleT_us = data_struct.sampletime_ * MILLION;
	auto begin_t = Timer.micro_now();
	// inicializa looptimer
	Timer.start();
	do
	{
		auto begin_timestamp = chrono::steady_clock::now();

		{
			unique_lock<mutex> _(*data_struct.mtx_);
			switch (data_struct.param39_)
			{
			case OPMODE::IMUS_READ:
				memcpy(log_states, *data_struct.datavecA_, sizeof(log_states));
				break;
			case OPMODE::PARAMS_READ:
				memcpy(log_gains, *data_struct.datavec_, sizeof(log_gains));
				break;
			case OPMODE::FT_READ:
				memcpy(log_ftsensor, *data_struct.datavecF_, sizeof(log_ftsensor));
				break;
			default:
				memcpy(log_gains, *data_struct.datavec_, sizeof(log_gains));
				memcpy(log_states, *data_struct.datavecA_, sizeof(log_states));
				memcpy(log_ftsensor, *data_struct.datavecF_, sizeof(log_ftsensor));
				break;
			}
		}

		timestamp = float(Timer.micro_now() - begin_t) / MILLION;

		int char_p = sprintf(temp_string, "%.4f", timestamp);
		for (size_t i = 0; i < DTVCA_SZ; i++) {
			int pos = sprintf(&temp_string[char_p-1], ", %.4f", log_states[i]);
			char_p += pos;
		}
		if (counter < n_lines) {
			string_vector[counter] = temp_string;
			counter++;
		}

		this_thread::sleep_until(begin_timestamp + chrono::microseconds(sampleT_us));
	} while (!Timer.end());


	// Saves in the file:
	logFileHandle = fopen(filename, "a");
	if (logFileHandle != NULL) {
		for (size_t i = 0; i < n_lines; i++) {
			if (string_vector[i].size() > 1) {
				const char *str = string_vector[i].c_str();
				fprintf(logFileHandle, "%s\n", str);
			}
		}
		fclose(logFileHandle);
	}

	{
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0D_ = false;
		*data_struct.param3F_ = true;
	}
}
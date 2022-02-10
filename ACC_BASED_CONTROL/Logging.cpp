//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br  	_____ ___  ___  //|
// github/bitbucket qleonardolp /	  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <chrono>

void Logging(ThrdStruct &data_struct){
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

    // setup stuff...
    char filename[] = "./data/log_thread.txt";
    FILE* logFileHandle = fopen(filename,"w");
    if (logFileHandle != NULL) fclose(logFileHandle);

    float log_data[18];
    bool isready_imu(false);
    bool isready_asg(false);
    bool isready_ctr(false);

    do{ 
        {   // Loggging confere IMU, ASGD e CONTROLE
            unique_lock<mutex> _(*data_struct.mtx_);
            isready_imu = *data_struct.param0A_;
            isready_asg = *data_struct.param0B_;
            isready_ctr = *data_struct.param0C_;
        } 
    } while (!isready_imu || !isready_asg || !isready_ctr);

    {   // Loggging avisa que esta pronto!
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0D_ = true;
    }

    looptimer Timer(data_struct.sampletime_);
    llint exec_time_micros = data_struct.exectime_*MILLION;
    llint t_begin = Timer.micro_now();
    do
    {
        Timer.tik();
        {   // sessao critica: minimo codigo necessario para pegar datavec_
            unique_lock<mutex> _(*data_struct.mtx_);
            memcpy(log_data, *data_struct.datavec_, 18*sizeof(float));
        }   // fim da sessao critica

        logFileHandle = fopen(filename,"a");
        if (logFileHandle != NULL){
            fprintf(logFileHandle, "%lld", Timer.micro_now());
            for (size_t i = 0; i < 18; i++)
                fprintf(logFileHandle, ", %.4f", log_data[i]);
            fprintf(logFileHandle, "\n");
            fclose(logFileHandle);
        }
        Timer.tak();
    } while (Timer.micro_now() - t_begin <= exec_time_micros);

    // ending stuff...
}
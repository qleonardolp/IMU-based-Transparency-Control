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

    float rad2deg = 180/(M_PI);
    const size_t vecsize = 10;
    float log_data[vecsize];
    float log_ftsensor[6];
    for (int i = 0; i < vecsize; i++) log_data[i] = 0;

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
            memcpy(log_data, *data_struct.datavecA_, sizeof(log_data));
            memcpy(log_ftsensor, *data_struct.datavecF_, sizeof(log_ftsensor));
        }   // fim da sessao critica
        

        logFileHandle = fopen(filename,"a");
        if (logFileHandle != NULL){
            fprintf(logFileHandle, "%lld", Timer.micro_now());
            for (size_t i = 0; i < (vecsize-1); i++)
                fprintf(logFileHandle, ", %.4f", rad2deg*log_data[i]);
            fprintf(logFileHandle, ", %f", log_ftsensor[1]);
            fprintf(logFileHandle, "\n");
            fclose(logFileHandle);
        }
        Timer.tak();
    } while (Timer.micro_now() - t_begin <= exec_time_micros);

    {   
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0D_ = false;
    }
}
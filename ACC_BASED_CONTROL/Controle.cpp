/////////////////////////////\_____///\///\////\/\|//\
// Leonardo Felipe Lima Santos dos Santos /\____//|\//\
// leonardo.felipe.santos@usp.br /	_____ ___  ___  \//|
// github/bitbucket qleonardolp /   | |  | . \/   \  \/|
// *Copyright 2021-2026* \//// //\ 	| |   \ \   |_|   \|
//\////////\/\/\/\/\/\/\//\// ////\	\_'_/\_`_/__|     /
///\_]//////\/\/\/\/\/\////\_////\////|////////\//[__/

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <iostream>

extern AXIS eixo_out;
extern AXIS eixo_in;
extern EPOS_NETWORK epos;
extern void HabilitaEixo(int ID);
extern void DesabilitaEixo(int ID);

void Controle(ThrdStruct &data_struct){
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif
    
    const size_t statesize = 10;
    const size_t loggsize  = 10;
    float states_data[statesize];
    float logging_data[loggsize];

    for (int i = 0; i < statesize; i++) states_data[i] = 0;
    for (int i = 0; i < loggsize; i++) logging_data[i] = 0;

    bool isready_imu(false);
    bool isready_asg(false);
    bool isready_log(false);
    do{ 
        {   // Controle confere se IMU e ASGD estao prontos:
            unique_lock<mutex> _(*data_struct.mtx_);
            isready_imu = *data_struct.param0A_;
            isready_asg = *data_struct.param0B_;
        } 
    } while (!isready_imu || !isready_asg);

    // Sincroniza as epos
    epos.sync();
    HabilitaEixo(2);

    // Habilita o controle de velocidade
    eixo_in.VCS_SetOperationMode(VELOCITY_MODE);
    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();
    
    {   // Controle avisa que esta pronto!
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0C_ = true;
    }

    looptimer Timer(data_struct.sampletime_);
    llint exec_time_micros = data_struct.exectime_*MILLION;
    llint t_begin = Timer.micro_now();
    do
    {
        Timer.tik();
        epos.sync();

        { // sessao critica: minimo codigo necessario para pegar datavec_
            unique_lock<mutex> _(*data_struct.mtx_);
            memcpy(states_data, *data_struct.datavecB_, sizeof(states_data));
            isready_log = *data_struct.param0D_;
        } // fim da sessao critica

        float vel_hum = states_data[3];
        int desiredVelRPM = 4*(30/M_PI)*vel_hum; // 4 eh ganho de teste

        eixo_in.PDOsetVelocitySetpoint(desiredVelRPM);
        eixo_in.WritePDO02();

        for (int i = 0; i < loggsize; i++){
          logging_data[i] = states_data[i];
        }

        // Share states with Logging:
        if(isready_log) 
        {
            // sessao critica
            unique_lock<mutex> _(*data_struct.mtx_);
            memcpy(*data_struct.datavecA_, logging_data, sizeof(logging_data));
            // fim da sessao critica
        }
        
        Timer.tak();
    } while (Timer.micro_now() - t_begin <= exec_time_micros);
    
    {
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0C_ = false;
    }

    epos.sync();
    DesabilitaEixo(0);
}
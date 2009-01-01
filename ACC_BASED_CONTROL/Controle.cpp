//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <iostream>
#include <chrono>

extern AXIS kneeRightMotor;
extern AXIS kneeRightEncoder;
extern AXIS kneeLeftMotor;
extern AXIS hipRightMotor;
extern AXIS hipLeftMotor;
extern EPOS_NETWORK epos;
extern void HabilitaEixo(int ID);
extern void DesabilitaEixo(int ID);

float controle_junta(const float states[10]);
float controle_acc(const float states[10]);
float controle_sea(const float states[10]);

void Controle(ThrdStruct &data_struct){
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif
    // pulando o 0 --\\/
    enum  controller{ _, ABC, ZTC, LTC, ITC, MKZ, MKT, MK0R};
    const size_t statesize = 10;
    const size_t loggsize  = 10;
    const size_t gainsize  = 18;
    float states_data[statesize];
    float logging_data[loggsize];
    float gains_data[gainsize];
    // Inicialização por segurança:
    for (int i = 0; i < statesize; i++) states_data[i] = 0;
    for (int i = 0; i < loggsize; i++) logging_data[i] = 0;
    for (int i = 0; i < gainsize; i++)   gains_data[i] = 0;

    float zero_kr = 0;      // kneeRightMotor zero
    float zero_kl = 0;      // kneeLeftMotor  zero
    float zero_hr = 0;      // hipRightMotor  zero
    float zero_hl = 0;      // hipLeftMotor   zero
    float zero_kr_enc = 0;  // kneeRightEncoder '0'
    float zero_kl_enc = 0;  // kneeLeftEncoder  '0'

    bool isready_imu(false);
    bool isready_asg(false);
    bool isready_ati(false);
    bool isready_log(false);
    do{ 
        {   // Espera IMU / ASGD / FT Sensor (ATI) estarem prontos:
            unique_lock<mutex> _(*data_struct.mtx_);
            isready_imu = *data_struct.param0A_;
            isready_asg = *data_struct.param0B_;
            isready_ati = *data_struct.param0E_;
        } 
    } while (!isready_imu || !isready_asg || !isready_ati);

    // Sincroniza as epos
    {
    epos.sync();
    this_thread::sleep_for(chrono::milliseconds(1000));

    kneeRightEncoder.ReadPDO01();
    zero_kr_enc = kneeRightEncoder.PDOgetActualPosition();
    kneeRightMotor.ReadPDO01();
    zero_kr = kneeRightMotor.PDOgetActualPosition();
    kneeLeftMotor.ReadPDO01();
    zero_kl = kneeLeftMotor.PDOgetActualPosition();
    hipRightMotor.ReadPDO01();
    zero_hr = hipRightMotor.PDOgetActualPosition();
    hipLeftMotor.ReadPDO01();
    zero_hl = hipLeftMotor.PDOgetActualPosition();

    // Definindo modo de operacao das EPOS:
    if (data_struct.param01_ == LTC){
        kneeRightMotor.VCS_SetOperationMode(CURRENT_MODE);
    } else{
        kneeRightMotor.VCS_SetOperationMode(VELOCITY_MODE);
    }
    kneeLeftMotor.VCS_SetOperationMode(VELOCITY_MODE);
    hipRightMotor.VCS_SetOperationMode(VELOCITY_MODE);
    hipLeftMotor.VCS_SetOperationMode(VELOCITY_MODE);

    kneeRightEncoder.ReadPDO01();
    kneeRightMotor.ReadPDO01();
    kneeLeftMotor.ReadPDO01();
    hipRightMotor.ReadPDO01();
    hipLeftMotor.ReadPDO01();

    this_thread::sleep_for(chrono::milliseconds(2000));
    HabilitaEixo(2);
    }

    {   // Controle avisa que esta pronto!
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0C_ = true;
    }

    looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
    // inicializa looptimer
    Timer.start();
    do
    {
        Timer.tik();
        epos.sync();

        { // sessao critica:
            unique_lock<mutex> _(*data_struct.mtx_);
            memcpy(states_data, *data_struct.datavecB_, sizeof(states_data));
            memcpy(gains_data, *data_struct.datavec_, sizeof(gains_data));
            isready_log = *data_struct.param0D_;
        } // fim da sessao critica


        switch (data_struct.param01_)
        {
        case ABC:
            /* code */
            break;
        case ZTC:
            /* code */
            break;
        case ITC:
            /* code */
            break;
        case LTC:
            /* code */
            kneeRightMotor.PDOsetCurrentSetpoint(0);
            kneeRightMotor.WritePDO01();
            break;
        default:
            break;
        }

        float vel_hum = states_data[3];
        int desiredVelRPM = 4*(30/M_PI)*vel_hum; // 4 eh ganho de teste

        kneeRightMotor.PDOsetVelocitySetpoint(desiredVelRPM);
        kneeRightMotor.WritePDO02();
        kneeLeftMotor.PDOsetVelocitySetpoint(desiredVelRPM);
        kneeLeftMotor.WritePDO02();

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
    } while (!Timer.end());
    
    {   // Fim da execução
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0C_ = false;
    }

    epos.sync();
    DesabilitaEixo(0);
}
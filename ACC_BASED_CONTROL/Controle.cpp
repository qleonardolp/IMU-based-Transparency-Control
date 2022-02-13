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
#include <math.h>

//	Torque Constant: 0.0603 N.m/A
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A
//  Rotor Inertia: 137 gcm^2

extern AXIS kneeRightMotor;
extern AXIS kneeRightEncoder;
extern AXIS kneeLeftMotor;
extern AXIS hipRightMotor;
extern AXIS hipLeftMotor;
extern EPOS_NETWORK epos;
extern void HabilitaEixo(int ID);
extern void DesabilitaEixo(int ID);

float controle_junta(const states input);  // generico
float controle_acc(const states input, const float gains[18]);  // acc-based
float controle_adm(const states input);    // admittance
float controle_sea(const states input);    // SEA feedback
float controle_lpshap(const states input, const float gains[18], float buffer[10]); // loop-haping

float constrain_float(float val, float min, float max);

void Controle(ThrdStruct &data_struct){
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif
    // pulando o 0 --\\/
    enum  controller{ _, ABC, ZTC, LTC, ITC, MKZ, MKT, MK0R};
    //Rotor Inertia: 137 gcm^2 = 0.137 Kg(0.01m)^2 =  0.137e-4 Kgm^2
    const float motor_inertia = 0.137e-4;
    const int sea_kr_stiffness = 104; // [N.m/rad]
    const int sea_kr_ratio = 150; // 
    const int encoder_kr_steps = 2048;
    const int motor_kr_steps = 4096;
    const size_t statesize = 10;
    const size_t loggsize  = 10;
    const size_t gainsize  = 18;
    float states_data[statesize];
    float logging_data[loggsize];
    float gains_data[gainsize];
    float lp_buffer[10];
    for (size_t i = 0; i < 10; i++) lp_buffer[i] = 0;
    // Inicialização por segurança:
    for (int i = 0; i < statesize; i++) states_data[i] = 0;
    for (int i = 0; i < loggsize; i++) logging_data[i] = 0;
    for (int i = 0; i < gainsize; i++)   gains_data[i] = 0;

    states sendto_control;
    // Zeros EPOS:
    float zero_kr(0);      // kneeRightMotor zero  (epos)
    float zero_kl(0);      // kneeLeftMotor  zero  (epos)
    float zero_hr(0);      // hipRightMotor  zero  (epos)
    float zero_hl(0);      // hipLeftMotor   zero  (epos)
    float zero_kr_enc(0);  // kneeRightEncoder '0' (epos)
    float zero_kl_enc(0);  // kneeLeftEncoder  '0' (epos)

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
    auto ctrl_id = data_struct.param01_;
    if (ctrl_id == LTC || ctrl_id == ABC){
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


    // Filtros PB para alguns sensores EPOS:
    LowPassFilter2pFloat sensor_filters[4];
    for (int i = 0; i < sizeof(sensor_filters)/sizeof(LowPassFilter2pFloat); i++)
    {
      sensor_filters[i].set_cutoff_frequency(data_struct.sampletime_, 16);
      sensor_filters[i].reset();
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
            memcpy(gains_data,  *data_struct.datavec_,  sizeof(gains_data));
            isready_log = *data_struct.param0D_;
        } // fim da sessao critica

        kneeRightEncoder.ReadPDO01();
        float exo_pos = 2*M_PI*(-kneeRightEncoder.PDOgetActualPosition() - zero_kr_enc)/encoder_kr_steps;
        kneeRightMotor.ReadPDO01();
        float mtr_pos = 2*M_PI*( kneeRightMotor.PDOgetActualPosition() - zero_kr)/(motor_kr_steps*sea_kr_ratio);

        exo_pos = sensor_filters[0].apply(exo_pos);
        mtr_pos = sensor_filters[1].apply(mtr_pos);

        states_data[3] = exo_pos;
        states_data[6] = sea_kr_stiffness*(mtr_pos - exo_pos);

        kneeRightMotor.ReadPDO02();
        states_data[4] = sensor_filters[2].apply( (M_PI/30)*(kneeRightMotor.PDOgetActualVelocity()) );
        states_data[9] = states_data[4];

        kneeRightMotor.ReadPDO01();
        // Torque Constant: 0.0603 N.m/A = 60.3 N.m/mA
        states_data[8] = sensor_filters[3].apply( 60.30f*(kneeRightMotor.PDOgetActualCurrent()) ); // [mA]
        states_data[5] = states_data[8]/motor_inertia;

        sendto_control.assign(states_data);
        switch (data_struct.param01_)
        {
        case ABC:
            float setpoint = controle_acc(sendto_control, gains_data);
            int setpoint_mA = constrain_float(1000*setpoint, -3100, 3100);
            kneeRightMotor.PDOsetCurrentSetpoint(setpoint_mA);
            kneeRightMotor.WritePDO01();
            break;
        case ZTC:
            /* code */
            break;
        case ITC:
            /* code */
            break;
        case LTC:
            float setpoint = controle_lpshap(sendto_control, gains_data, lp_buffer);
            int setpoint_mA = constrain_float(1000*setpoint, -3100, 3100);
            kneeRightMotor.PDOsetCurrentSetpoint(setpoint_mA);
            kneeRightMotor.WritePDO01();
            break;
        default:
            break;
        }

        //int desiredVelRPM = 4*(30/M_PI)*sendto_control.hum_rgtknee_vel; // 4 eh "ganho"
        //kneeRightMotor.PDOsetVelocitySetpoint(desiredVelRPM);
        //kneeRightMotor.WritePDO02();
        //kneeLeftMotor.PDOsetVelocitySetpoint(desiredVelRPM);
        //kneeLeftMotor.WritePDO02();

        // Share states with Logging:
        if(isready_log) 
        {
            for (int i = 0; i < loggsize; i++) logging_data[i] = states_data[i];
            // sessao critica:
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

// Controladores (em essência):
float controle_junta(const states input) {return 0;} 

float controle_acc(const states input, const float gains[18])
{
    const float Jr = 0.885;
    float Kp = gains[0];
    float Kd = gains[1];
    float actuation(0);
    // Feedback
    actuation += Kp*(input.hum_rgtknee_vel - input.rbt_rgtknee_vel) + \
                 Kd*(input.hum_rgtknee_acc - input.rbt_rgtknee_acc);
    // Feedforward
    actuation += input.mtr_rgtknee_tau + Jr*input.hum_rgtknee_acc;
    return actuation;
} 

float controle_adm(const states input) {return 0;} 

float controle_sea(const states input) {return 0;} 

float controle_lpshap(const states input, const float gains[18], float buffer[10])
{
    const float Jr = 0.885;
    float Kp = gains[4];
    float Kd = gains[5];
    //buffer[4] = 1; util para Transfer Funct...
    float actuation(0);
    // Feedback
    actuation += Kp*(input.hum_rgtknee_vel - input.rbt_rgtknee_vel) + \
                 Kd*(input.hum_rgtknee_acc - input.rbt_rgtknee_acc);
    // Feedforward
    actuation += input.mtr_rgtknee_tau + Jr*input.hum_rgtknee_acc;
    return actuation;
}

float constrain_float(float val, float min, float max)
{
	if (_isnan(val)) return (min + max)/2;

	if (val < min) return min;

	if(val > max) return max;
	
	return val;
}
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

float controle_junta(const states input, const float gains[18], float buffer[10]);  // generico
float controle_acc(const states input, const float gains[18], float buffer[10]);  // acc-based
float controle_adm(const states input, const float gains[18], float buffer[10]);    // admittance
float controle_sea(const states input, const float gains[18], float buffer[10]);    // SEA feedback
float controle_lpshap(const states input, const float gains[18], float buffer[10], const float smpl_time); // loop-shaping

float constrain_float(float val, float min, float max);

void Controle(ThrdStruct &data_struct){
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif
    // pulando o 0 --\\/
    enum  controller{ _, ABC, ZTC, LTC, ITC, MKZ, MKT, MK0R, SEA};
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
    float setpoint(0);
    int   setpoint_rpm(0);
    int   setpoint_mA(0);
    float lp_buffer[10];
    float sea_bffr[10];
    float acc_bffr[10];
    for (size_t i = 0; i < 10; i++)
    {
        lp_buffer[i] = 0;
        sea_bffr[i]  = 0;
        acc_bffr[i]  = 0;
    }
    // Inicialização por segurança:
    for (int i = 0; i < statesize; i++) states_data[i] = 0;
    for (int i = 0; i < loggsize; i++) logging_data[i] = 0;
    for (int i = 0; i < gainsize; i++)   gains_data[i] = 0;

    States sendto_control;
    // Zeros EPOS:
    int zero_kr(0);      // kneeRightMotor zero  (epos)
    int zero_kl(0);      // kneeLeftMotor  zero  (epos)
    int zero_hr(0);      // hipRightMotor  zero  (epos)
    int zero_hl(0);      // hipLeftMotor   zero  (epos)
    int zero_kr_enc(0);  // kneeRightEncoder '0' (epos)
    int zero_kl_enc(0);  // kneeLeftEncoder  '0' (epos)

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
      float thread_frequency = 1/(data_struct.sampletime_); // Hz !!!
      sensor_filters[i].set_cutoff_frequency(thread_frequency, 16);
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
        float exo_pos = 2*M_PI*float(-kneeRightEncoder.PDOgetActualPosition() - zero_kr_enc)/encoder_kr_steps;
        kneeRightMotor.ReadPDO01();
        float mtr_pos = 2*M_PI*float( kneeRightMotor.PDOgetActualPosition() - zero_kr)/(motor_kr_steps*sea_kr_ratio);

        exo_pos = sensor_filters[0].apply(exo_pos);
        mtr_pos = sensor_filters[1].apply(mtr_pos);

        states_data[3] = exo_pos;
        states_data[6] = sea_kr_stiffness*(mtr_pos - exo_pos);

        kneeRightMotor.ReadPDO02();
        states_data[4] = sensor_filters[2].apply( RPM2RADS*(kneeRightMotor.PDOgetActualVelocity()) );
        states_data[9] = states_data[4];

        kneeRightMotor.ReadPDO01();
        // Torque Constant: 0.0603 N.m/A
        states_data[8] = sensor_filters[3].apply( 0.0603f*float(kneeRightMotor.PDOgetActualCurrent())/1000 ); // [mA]
        states_data[5] = states_data[8]/motor_inertia;

        sendto_control.assign(states_data);
        switch (data_struct.param01_)
        {
        case ABC:
            setpoint = controle_acc(sendto_control, gains_data, acc_bffr);
            setpoint_mA = constrain_float(1000*setpoint, -3100, 3100);
            kneeRightMotor.PDOsetCurrentSetpoint(setpoint_mA);
            kneeRightMotor.WritePDO01();
            break;
        case ZTC:
            setpoint = RADS2RPM*sea_kr_ratio*sendto_control.hum_rgtknee_vel;
            setpoint_rpm = constrain_float(setpoint, -7590, 7590); // No load speed
            // simplesmente... esqueci a reducao do sistema!!!
            kneeRightMotor.PDOsetVelocitySetpoint(setpoint_rpm);
            kneeRightMotor.WritePDO02();
            break;
        case ITC:
            /* code */
            break;
        case LTC:
            setpoint = controle_lpshap(sendto_control, gains_data, lp_buffer, data_struct.sampletime_);
            setpoint_mA = constrain_float(1000*setpoint, -3100, 3100);
            kneeRightMotor.PDOsetCurrentSetpoint(setpoint_mA);
            kneeRightMotor.WritePDO01();
            break;
        case SEA:
            setpoint = controle_sea(sendto_control, gains_data, sea_bffr);
            setpoint_rpm = constrain_float(RADS2RPM*sea_kr_ratio*setpoint, -7590, 7590);
            kneeRightMotor.PDOsetVelocitySetpoint(setpoint_rpm);
            kneeRightMotor.WritePDO02();
            break;
        default:
            break;
        }

        // Share states with Logging:
        if(isready_log) 
        {
            for (int i = 0; i < loggsize; i++) logging_data[i] = states_data[i];
            logging_data[1] = sendto_control.hum_rgtknee_vel;
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
float controle_junta(const states input, const float gains[18], float buffer[10]) {return 0;} 

float controle_acc(const states input, const float gains[18], float buffer[10])
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

float controle_adm(const states input, const float gains[18], float buffer[10]) {return 0;} 

float controle_sea(const states input, const float gains[18], float buffer[10]) {return 0;} 

float controle_lpshap(const states input, const float gains[18], float buffer[10], const float smpl_time)
{
    using namespace Eigen;

    const float Jr = 0.885;
    float Kp = gains[4];
    float Kd = gains[5];
    bool  update_tf = false;
    float a0 = gains[14];
    float b0 = gains[10]/a0;
    float b1 = gains[11]/a0;
    float b2 = gains[12]/a0;
    float b3 = gains[13]/a0;
    float a1 = gains[15]/a0;
    float a2 = gains[16]/a0;
    float a3 = gains[17]/a0; //... I'm tired

    // Def.:
    //        b0 s^3 + b1 s^2 + b2 s + b3
    // FT = --------------------------------, with a0 = 1!
    //        a0 s^3 + a1 s^2 + a2 s + a3
    // Usar Forma Canonica Controlavel... (Ogata pg. 596)
    Matrix3f A; A << 0, 1, 0, 0, 0, 1, -a3, -a2, -a1;
    Vector3f B; B << 0, 0, 1;
    RowVector3f C; C << b3 - a3*b0, b2 - a2*b0, b1 - a1*b0;
    float D = b0;
    // Discretizacao 3 Ord:
    float Ts = smpl_time;
    Matrix3f Ak = Matrix3f::Identity() + A*Ts + (A*Ts).pow(2)/2 + (A*Ts).pow(3)/6;
    Vector3f Bk = (Matrix3f::Identity() + A*Ts/2 + (A*Ts).pow(2)/6 + (A*Ts).pow(3)/24)*B*Ts;

    float vel_error = input.hum_rgtknee_vel - input.rbt_rgtknee_vel;
    Vector3f xk; xk << buffer[0], buffer[1], buffer[2];
    xk = Ak*xk + Bk*vel_error;
    float yk = C*xk + D*vel_error;
    buffer[0] = xk(0);
    buffer[1] = xk(1);
    buffer[2] = xk(2);

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
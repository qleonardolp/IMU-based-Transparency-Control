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
#include <iostream>
#include <chrono>
#include <math.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

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

// Controllers:
float  controle_junta(const states input, const float gains[18], float buffer[10], const float sample_tm); // generico
float    controle_acc(const states input, const float gains[18], float buffer[10], const float sample_tm); // acc-based
float    controle_adm(const states input, const float gains[18], float buffer[10], const float sample_tm); // admittance
float    controle_sea(const states input, const float gains[18], float buffer[10], const float sample_tm); // SEA feedback
float controle_lpshap(const states input, const float gains[18], float buffer[10], const float sample_tm); // loop-shaping
// Low Level abstraction:
void SetEPOSTorque(float desired_torque);
void SetEPOSVelocity(float desired_rads);
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

    bool control_abort(false);
    bool isready_imu(false);
    bool aborting_imu(false);
    bool isready_asg(false);
    bool isready_ati(false);
    bool aborting_ati(false);
    bool isready_log(false);
    bool isready_gsn(false);
    do{ 
        {   // Espera IMU / ASGD / FT Sensor (ATI) estarem prontos:
            unique_lock<mutex> _(*data_struct.mtx_);
            isready_imu = *data_struct.param0A_;
            aborting_imu = *data_struct.param1A_;
            isready_asg = *data_struct.param0B_;
            isready_ati = *data_struct.param0E_;
            aborting_ati = *data_struct.param1E_;
            isready_gsn = *data_struct.param0F_;
            if(aborting_imu || aborting_ati)
            {
                control_abort = *data_struct.param1C_ = true;
                break;
            }
        } 
    //} while (!isready_imu || !isready_asg || !isready_ati);
    } while (!isready_imu || !isready_asg || !isready_gsn);
    //} while (!isready_imu || !isready_asg);

    if (control_abort){
        return;
    }

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
      sensor_filters[i].set_cutoff_frequency(thread_frequency, LPF_CUTOFF);
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
            isready_log  = *data_struct.param0D_;
            if(*data_struct.param1A_ || *data_struct.param1E_)
            {
                control_abort = *data_struct.param1C_ = true;
                break;
            }
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
            setpoint = controle_acc(sendto_control, gains_data, acc_bffr, data_struct.sampletime_);
            SetEPOSTorque(setpoint);
            break;
        case ZTC:
            setpoint = controle_adm(sendto_control, gains_data, acc_bffr, data_struct.sampletime_);
            SetEPOSVelocity(setpoint);
            break;
        case ITC:
            setpoint = controle_junta(sendto_control, gains_data, acc_bffr, data_struct.sampletime_);
            SetEPOSVelocity(setpoint);
            break;
        case LTC:
            setpoint = controle_lpshap(sendto_control, gains_data, lp_buffer, data_struct.sampletime_);
            SetEPOSTorque(setpoint);
            break;
        case SEA:
            setpoint = controle_sea(sendto_control, gains_data, sea_bffr, data_struct.sampletime_);
            SetEPOSVelocity(setpoint);
            break;
        case (11*IMUBYPASS):
            // In this case 'hum_rgtknee_vel' is GyroscopeX from the 3º IMU!!!
            // Useful for qASGD debug....
            setpoint = sendto_control.hum_rgtknee_vel;
            SetEPOSVelocity(setpoint);
            break;
        default:
            break;
        }

        // Share states with Logging:
        if(isready_log) 
        {
            // sessao critica:
            unique_lock<mutex> _(*data_struct.mtx_);
            *(*data_struct.datavecA_ + 0) = *(*data_struct.datavecB_ + 0);
            *(*data_struct.datavecA_ + 1) = *(*data_struct.datavecB_ + 1);
            *(*data_struct.datavecA_ + 2) = *(*data_struct.datavecB_ + 2);
            *(*data_struct.datavecA_ + 3) = states_data[3];
            *(*data_struct.datavecA_ + 4) = states_data[4];
            *(*data_struct.datavecA_ + 5) = states_data[5];
            *(*data_struct.datavecA_ + 6) = states_data[6];
            *(*data_struct.datavecA_ + 7) = *(*data_struct.datavecB_ + 7);
            *(*data_struct.datavecA_ + 8) = setpoint; // check LS output
            *(*data_struct.datavecA_ + 9) = states_data[9];
            //memcpy(*data_struct.datavecA_, logging_data, sizeof(logging_data));
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

// Controladores, em essência! Trabalham com unidades no SI e sem precisar de conversoes do motor (Gear Ratio, Kt, Kw...)!
float controle_junta(const states input, const float gains[18], float buffer[10], const float sample_tm) {return 0;} 

float controle_acc(const states input, const float gains[18], float buffer[10], const float sample_tm)
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

float controle_adm(const states input, const float gains[18], float buffer[10], const float sample_tm) {return 0;} 

float controle_sea(const states input, const float gains[18], float buffer[10], const float sample_tm) {return 0;} 

float controle_lpshap(const states input, const float gains[18], float buffer[10], const float sample_tm)
{
    using namespace Eigen;
    const float Jr = 0.885;
    float a0 = gains[14];
    float b0 = gains[10];
    float b1 = gains[11];
    float b2 = gains[12];
    float b3 = gains[13];
    float a1 = gains[15];
    float a2 = gains[16];
    float a3 = gains[17];
    // not tired anymore!!
    if(a0 > 0.000001){
      a0 /= a0;
      b0 /= a0;
      b1 /= a0;
      b2 /= a0;
      b3 /= a0;
      a1 /= a0;
      a2 /= a0;
      a3 /= a0; 
    }

    // Def.:
    //        b0 s^3 + b1 s^2 + b2 s + b3
    // FT = --------------------------------, with a0 = 1!
    //        a0 s^3 + a1 s^2 + a2 s + a3
    // Forma Canonica Controlavel | (Ogata pg. 596):
    Matrix3f A; 
    A << 0, 1, 0, 0, 0, 1, -a3, -a2, -a1;
    Vector3f B(0, 0, 1);
    RowVector3f C(b3 - a3*b0, b2 - a2*b0, b1 - a1*b0);
    float D = b0;
    // Discretizacao 2 Ord:
    float Ts = sample_tm;
    Matrix3f Ak = Matrix3f::Identity() + A*Ts + (A*Ts).pow(2)/2;
    Vector3f Bk = (Matrix3f::Identity() + A*Ts/2 + (A*Ts).pow(2)/6)*B*Ts;

    float vel_error = input.hum_rgtknee_vel - input.rbt_rgtknee_vel;
    Vector3f xk(buffer[0], buffer[1], buffer[2]);
    xk = Ak*xk + Bk*vel_error;
    float yk = C*xk + D*vel_error;
    buffer[0] = xk(0);
    buffer[1] = xk(1);
    buffer[2] = xk(2);

    return input.mtr_rgtknee_tau + Jr*input.hum_rgtknee_acc + yk; // FF + FB
    //return yk;
}

// Low Level abstraction:

void SetEPOSTorque(float desired_torque){
    // Torque Constant: 0.0603 N.m/A
    // Gear Ratio: 150
    int setpoint_mA = 1000*(desired_torque/0.0603f)/150.0f; // -> '110.5583*desired_torque'
    float setpoint_mA_limited = constrain_float(setpoint_mA, -3100, 3100);
    kneeRightMotor.PDOsetCurrentSetpoint(setpoint_mA_limited);
    kneeRightMotor.WritePDO01();
}

void SetEPOSVelocity(float desired_rads){
    // Gear Ratio: 150!!
    float setpoint_rpm = constrain_float(RADS2RPM*150*desired_rads, -7590, 7590);
    kneeRightMotor.PDOsetVelocitySetpoint(setpoint_rpm);
    kneeRightMotor.WritePDO02();
}

float constrain_float(float val, float min, float max)
{
	if (_isnan(val)) return (min + max)/2;

	if (val < min) return min;

	if(val > max) return max;
	
	return val;
}
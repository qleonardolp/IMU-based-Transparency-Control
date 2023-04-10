//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#ifndef SHAREDSTRUCT_H
#define SHAREDSTRUCT_H
// File for structs definitions:

#include <stdio.h>
#include <thread>
#include <mutex>
#include <condition_variable>

constexpr int WIND_WIDTH = 1024;
constexpr int WIND_HEIGHT = 576;

#define CAN_ENABLE 0
#define IMU_DBG_LOG 0
//#define IMU_ATT_LOG
#define IMUBYPASS 3
#define READIMUS  4

constexpr int NUMBER_OF_IMUS = 6;
constexpr int IMU_DATA_SZ = 6;
//#define QASGD_THREAD_DEBUG

// Radians to Degrees
#define R2D (180 / M_PI)

#define DTVC_SZ 36
#define DTVCA_SZ 10
#define DTVCB_SZ 10
#define DTVCF_SZ 6

typedef struct shared_struct {
    float sampletime_;
    int     exectime_;
    float      *data_;
    std::mutex  *mtx_;
    std::mutex* mtx_vector_[NUMBER_OF_IMUS];
    std::condition_variable* cv_vector_[NUMBER_OF_IMUS];
    float *datavec_[DTVC_SZ];
    float *datavecA_[DTVCA_SZ];
    float *datavecB_[DTVCB_SZ];
    float *datavecF_[DTVCF_SZ];

    // 64 parameters for data/options/flags exchange:
    // parameters, 1� block:
    short  param00_; // thread priority
    short  param01_; //
    short  param02_;
    short  param03_;
    short  param04_;
    short  param05_;
    short  param06_;
    short  param07_;
    short  param08_;
    short  param09_;
    short *param0A_; // imu_isready      (flag)
    short *param0B_; // asgd_isready     (flag)
    short *param0C_; // control_isready  (flag)
    short *param0D_; // logging_isready  (flag)
    short *param0E_; // ftsensor_isready (flag)
    short *param0F_; // gscan_isready    (flag)

    // parameters, 2� block:
    short  param10_;
    short  param11_;
    short  param12_;
    short  param13_;
    short  param14_;
    short  param15_;
    short  param16_;
    short  param17_;
    short  param18_;
    short  param19_;
    short *param1A_; // imu_aborting      (flag)
    short *param1B_; // asgd_aborting     (flag)
    short *param1C_; // control_aborting  (flag)
    short *param1D_; // logging_aborting  (flag)
    short *param1E_; // ftsensor_aborting (flag)
    short *param1F_; // gscan_aborting    (flag)

    // parameters, 3� block:
    short  param20_;
    short  param21_;
    short  param22_;
    short  param23_;
    short  param24_;
    short  param25_;
    short  param26_;
    short  param27_;
    short  param28_;
    short  param29_;
    short *param2A_; // imu_holding      (flag)
    short *param2B_; // asgd_holding     (flag)
    short *param2C_; // control_holding  (flag)
    short *param2D_; // logging_holding  (flag)
    short *param2E_; // ftsensor_holding (flag)
    short *param2F_; // gscan_holding    (flag) 

    // parameters, 4� block:
    short  param30_;
    short  param31_;
    short  param32_;
    short  param33_;
    short  param34_;
    short  param35_;
    short  param36_;
    short  param37_;
    short  param38_;
    short  param39_;
    short *param3A_;
    short* param3B_;
    short* param3C_;
    short* param3D_;
    short* param3E_;
    short* param3F_; // _finished    (flag) 

} ThrdStruct;

typedef struct states{
    // MUST be used in SI units!
    // positions (rad)
    float rbt_rgthip_pos;
    float rbt_lfthip_pos;
    float rbt_rgtknee_pos;
    float rbt_lftknee_pos;
    float rbt_rgtankle_pos;
    float rbt_lftankle_pos;
    float hum_rgthip_pos;
    float hum_lfthip_pos;
    float hum_rgtknee_pos;
    float hum_lftknee_pos;
    float hum_rgtankle_pos;
    float hum_lftankle_pos;
    // velocities (rad/s)
    float rbt_rgthip_vel;
    float rbt_lfthip_vel;
    float rbt_rgtknee_vel;
    float rbt_lftknee_vel;
    float rbt_rgtankle_vel;
    float rbt_lftankle_vel;
    float hum_rgthip_vel;
    float hum_lfthip_vel;
    float hum_rgtknee_vel;
    float hum_lftknee_vel;
    float hum_rgtankle_vel;
    float hum_lftankle_vel;
    // accelerations (rad/ss)
    float rbt_rgthip_acc;
    float rbt_lfthip_acc;
    float rbt_rgtknee_acc;
    float rbt_lftknee_acc;
    float rbt_rgtankle_acc;
    float rbt_lftankle_acc;
    float hum_rgthip_acc;
    float hum_lfthip_acc;
    float hum_rgtknee_acc;
    float hum_lftknee_acc;
    float hum_rgtankle_acc;
    float hum_lftankle_acc;

    // Forces/Torques (N.m) (SEA)
    float sea_lftthigh;
    float sea_rgtthigh;
    float sea_lftshank;
    float sea_rgtshank;
    float sea_lftfoot;
    float sea_rgtfoot;
    float inter_lftthigh;
    float inter_rgtthigh;
    float inter_lftshank;
    float inter_rgtshank;
    float inter_lftfoot;
    float inter_rgtfoot;

    // Motor Torque (N.m)
    float mtr_lfthip_tau;
    float mtr_rgthip_tau;
    float mtr_lftknee_tau;
    float mtr_rgtknee_tau;
    // Motor Speed (rad/s)
    float mtr_lfthip_omg;
    float mtr_rgthip_omg;
    float mtr_lftknee_omg;
    float mtr_rgtknee_omg;

    // metodo de atribuicao:
    void assign(const float vector[10]){
        // por enquanto so para o joelho direito:
        hum_rgtknee_pos = vector[0];
        hum_rgtknee_vel = vector[1];
        hum_rgtknee_acc = vector[2];
        rbt_rgtknee_pos = vector[3];
        rbt_rgtknee_vel = vector[4];
        rbt_rgtknee_acc = vector[5];
        sea_rgtshank    = vector[6];
        inter_rgtshank  = vector[7];
        mtr_rgtknee_tau = vector[8];
        mtr_rgtknee_omg = vector[9];
    }
} States;

enum OPMODE {
    NO_MODE,                // 0
    FULL_SENSORS_CONTROL,   // 1
    IMUS_ONLY_CONTROL,      // 2
    IMU_BYPASS_CONTROL,     // 3
    IMUS_READ,              // 4
    FT_READ,                // 5
    PARAMS_READ,            // 6
    RESET_CAN,              // 7
    SEA_ONLY_CONTROL        // 8
};

#endif // SHAREDSTRUCT_H

/////////////////////////////\_____///\///\////\/\|//\
// Leonardo Felipe Lima Santos dos Santos /\____//|\//\
// leonardo.felipe.santos@usp.br /	_____ ___  ___  \//|
// github/bitbucket qleonardolp /   | |  | . \/   \  \/|
// *Copyright 2021-2026* \//// //\ 	| |   \ \   |_|   \|
//\////////\/\/\/\/\/\/\//\// ////\	\_'_/\_`_/__|     /
///\_]//////\/\/\/\/\/\////\_////\////|////////\//[__/
//"Because this is dedicated to the kids            /
// Dedicated to wherever music lives               /
// Dedicated to those tired of the same old same  /
// And dedicated to the people advancin' the game"
// _________________________'Dedicated' (1999)__/
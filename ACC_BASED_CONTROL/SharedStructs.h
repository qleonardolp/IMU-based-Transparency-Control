#ifndef SHAREDSTRUCT_H
#define SHAREDSTRUCT_H
// File for structs definitions:

#include <stdio.h>
#include <thread>
#include <mutex>

typedef struct shared_struct {
    float sampletime_;
    int     exectime_;
    float      *data_;
    std::mutex  *mtx_;
    float *datavec_[18];

    // parameters, 1º block:
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
    short *param0A_; // imu_isready     (flag)
    short *param0B_; // asgd_isready    (flag)
    short *param0C_; // control_isready (flag)
    short *param0D_; // logging_isready (flag)
    short *param0E_; // abort_flag
    short *param0F_; // ...

    // parameters, 2º block:
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
    short *param1A_;
    short *param1B_;
    short *param1C_;
    short *param1D_;
    short *param1E_;
    short *param1F_; 

    // parameters, 3º block:
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
    short *param2A_;
    short *param2B_;
    short *param2C_;
    short *param2D_;
    short *param2E_;
    short *param2F_; 

    // parameters, 4º block:
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
    short *param3B_;
    short *param3C_;
    short *param3D_;
    short *param3E_;
    short *param3F_; 

} ThrdStruct;

#endif // SHAREDSTRUCT_H

//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br  	_____ ___  ___  //|
// github/bitbucket qleonardolp /	  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/
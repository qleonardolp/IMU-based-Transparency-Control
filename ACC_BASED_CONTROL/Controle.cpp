//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/

#ifdef _WIN32
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#else
#include <windows.h>
#endif
#include "XsensEpos.h"
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <iostream>

void Controle(ThrdStruct &data_struct){
    using namespace std;
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);

    // Sincroniza as epos
    epos.sync();

    // Habilita o controle de velocidade
    eixo_in.VCS_SetOperationMode(VELOCITY_MODE);
    // eixo_out.ReadPDO01();
    // eixo_in.ReadPDO01();
    
    // setup stuff...

    control_isready = true;
    do
    {
        // wait!
    } while (!imu_isready || !asgd_isready || !logging_isready);
    

    looptimer Timer(data_struct.sampletime_);
    auto exec_time_micros = data_struct.exectime_*MILLION;
    auto t_begin = Timer.micro_now();
    do
    {
        Timer.tik();
        /* code */

        Timer.tak();
    } while (Timer.micro_now() - t_begin <= exec_time_micros);

    // ending stuff...
}
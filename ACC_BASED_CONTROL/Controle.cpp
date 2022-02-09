/////////////////////////////\_____///\///\////\/\|//\//
// Leonardo Felipe Lima Santos dos Santos /\____//|\//\/
// leonardo.felipe.santos@usp.br /	_____ ___  ___  \//|
// github/bitbucket qleonardolp /	| |  | . \/   \  \/|
// *Copyright 2021-2026* \//// //\ 	| |   \ \   |_|   \|
//\////////\/\/\/\/\/\/\//\// ////\	\_'_/\_`_/__|     //
///\_]//////\/\/\/\/\/\////\_////\////|////////\//[__/\/

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
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

    // Sincroniza as epos
    epos.sync();

    // Habilita o controle de velocidade
    eixo_in.VCS_SetOperationMode(VELOCITY_MODE);
    // eixo_out.ReadPDO01();
    // eixo_in.ReadPDO01();
    
    // setup stuff...
    do
    {
        // wait!
    } while (!imu_isready || !asgd_isready);
    control_isready = true;
    

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
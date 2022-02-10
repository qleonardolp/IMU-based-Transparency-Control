//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br  	_____ ___  ___  //|
// github/bitbucket qleonardolp /	  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/

#include "XsensEpos.h"
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include <processthreadsapi.h>
#include <stdexcept>
#include <iostream>
#include <conio.h>

void readIMUs(ThrdStruct &data_struct);
void readFTSensor(ThrdStruct &data_struct);
void qASGD(ThrdStruct &data_struct);
void Controle(ThrdStruct &data_struct);
void Logging(ThrdStruct &data_struct);

// DEBUGGING DEFINES:
#define PRIORITY     0
#define ISREADY_WAIT 0
#define EXEC_TIME   10

// Threads Sample Time:
#define IMU_SMPLTM  0.0100 // "@100 Hz", actually is defined by Xsens 'desiredUpdateRate'
#define ASGD_SMPLTM 0.0050 //  @200  Hz
#define CTRL_SMPLTM 0.0005 //  @2000 Hz
#define LOG_SMPLTM  0.0040 //  @250  Hz 
// Threads Priority:
#define IMU_PRIORITY   -1 //
#define ASGD_PRIORITY  -1 //
#define CTRL_PRIORITY  -2 //
#define LOG_PRIORITY    0 // 

int main()
{
  using namespace std;
#if PRIORITY
  DWORD dwPriority, dwError;
  if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)){
      dwError = GetLastError();
      printf("Error %lu! \n", dwError);
  } else{
      dwPriority = GetPriorityClass( GetCurrentProcess() );
      printf("Process priority class: %lu \n", dwPriority);
  }
#endif

  cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << endl;
  IniciaRedeCan();
  HabilitaEixo(2);

  mutex imu_mtx;
  float shared_data[18];
  //initialize shared_data:
  for (int i = 0; i < sizeof(shared_data)/sizeof(float); i++){
    shared_data[i] = i*i;
  }

  short imu_isready(false);
  short asgd_isready(false);
  short control_isready(false);
  short logging_isready(false);
  ThrdStruct imu_struct, asgd_struct, control_struct, logging_struct;
  int execution_time = EXEC_TIME; // just for test

  imu_struct.sampletime_ = IMU_SMPLTM;
  imu_struct.param00_  = IMU_PRIORITY;
  *(imu_struct.datavec_) = shared_data;

  asgd_struct.sampletime_ = ASGD_SMPLTM;
  asgd_struct.param00_  = ASGD_PRIORITY;
  *(asgd_struct.datavec_) = shared_data;

  control_struct.sampletime_ = CTRL_SMPLTM;
  control_struct.param00_  = CTRL_PRIORITY;
  *(control_struct.datavec_) = shared_data;

  logging_struct.sampletime_ = LOG_SMPLTM;
  logging_struct.param00_  = LOG_PRIORITY;
  *(logging_struct.datavec_) = shared_data;

  imu_struct.mtx_ = asgd_struct.mtx_ = control_struct.mtx_ = logging_struct.mtx_ = &imu_mtx;
  imu_struct.exectime_ = asgd_struct.exectime_ = control_struct.exectime_ = logging_struct.exectime_ = execution_time;
  // Readiness Flags:
  imu_struct.param0A_ = asgd_struct.param0A_ = control_struct.param0A_ = logging_struct.param0A_ = &imu_isready;
  imu_struct.param0B_ = asgd_struct.param0B_ = control_struct.param0B_ = logging_struct.param0B_ = &asgd_isready;
  imu_struct.param0C_ = asgd_struct.param0C_ = control_struct.param0C_ = logging_struct.param0C_ = &control_isready;
  imu_struct.param0D_ = asgd_struct.param0D_ = control_struct.param0D_ = logging_struct.param0D_ = &logging_isready;

  thread thr_imus;
  thread thr_qasgd;
  thread thr_controle;
  thread thr_logging;

  /*
    - Programar "menu" com opcoes
    Aqui vem um grande switch com opcoes def pelo usuario.
    As threads serao disparadas de forma condicional.
    Para simplificar, as threads que nao forem disparadas e que travam as outras
    serao consideradas "isready = true"  dispensando a skip_flag. 
    Programar tbm para voltar ao "menu principal" depois que acabar a execucao
  */

  // Threads fired:
  thr_imus     = thread(readIMUs, imu_struct);
  thr_qasgd    = thread(qASGD, asgd_struct);
  thr_logging  = thread(Logging, logging_struct);
  thr_controle = thread(Controle, control_struct);
  // main waits while the threads execute thier tasks...
  thr_controle.join();
  thr_logging.join();
  thr_qasgd.join();
  thr_imus.join();

  DesabilitaEixo(0);
  epos.StopPDOS(1);
  this_thread::sleep_for(chrono::milliseconds(1234));
  cout << "Successful exit. Press [ENTER] to quit." << endl;
  cin.get();
  return 0;
}
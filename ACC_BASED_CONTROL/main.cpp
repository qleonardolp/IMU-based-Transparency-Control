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

void HabilitaEixo(int ID);
void DesabilitaEixo(int ID);
void readIMUs(ThrdStruct &data_struct);
void qASGD(ThrdStruct &data_struct);
void Controle(ThrdStruct &data_struct);
void Logging(ThrdStruct &data_struct);

// DEBUGGING DEFINES:
#define EXEC_TIME   10
#define ISREADY_WAIT 0

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
  // START DA REDE CAN
  epos.StartPDOS(1);
  epos.StartPDOS(2);
  epos.StartPDOS(3);
  epos.StartPDOS(4);
  epos.StartPDOS(5);
  // epos.StartPDOS(1);
  // epos.StartPDOS(2);
  // epos.StartPDOS(3);
  // epos.StartPDOS(4);
  // epos.StartPDOS(5);

  cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << endl;

  for (int i = 0; i < 10; i++)
  {
    // Aguarda tempo
    auto endwait = clock() + 1 * CLOCKS_PER_SEC;
    while (clock() < endwait){  }

    // Sincroniza as epos
    epos.sync();

    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();

    printf(".");
  }
  // EPOS 01
  eixo_out.PDOsetControlWord_FaultReset(true);
  eixo_out.WritePDO01();
  auto endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }
  eixo_out.PDOsetControlWord_FaultReset(false);
  eixo_out.WritePDO01();
  cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }

  // EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(true);
  eixo_in.WritePDO01();
  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }
  eixo_in.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();
  cout << ".. OK!" << endl;

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

  epos.StopPDOS(1);
  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }

  cout << "Successful exit. Press [ENTER] to quit.\n";
  cin.get();
  return 0;
}

/* EPOS FUNCTIONS */

void HabilitaEixo(int ID)
{

  if ((ID == 2) | (ID == 0))
  {

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");

    auto endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait){  }

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait){  }

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(true);
    eixo_in.WritePDO01();
  }
}

void DesabilitaEixo(int ID)
{

  if ((ID == 2) | (ID == 0))
  {
    printf("\nDESABILITANDO O MOTOR E CONTROLE\n\n");

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    auto endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait){  }

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();
  }
}
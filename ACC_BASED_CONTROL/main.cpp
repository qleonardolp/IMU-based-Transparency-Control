// Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide. All rights reserved.

///////////////////////////////////////////////////////////////////////////
//  Leonardo Felipe Lima Santos dos Santos, 2019-2023 (@qleonardolp)     //
///////////////////////////////////////////////////////////////////////////
// #undef UNICODE

// #define WIN32_LEAN_AND_MEAN

#ifdef _WIN32
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#else
#include <windows.h>
#endif
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <processthreadsapi.h>
#include <iostream>
#include <stdexcept>
// #include <sstream>
// #include <utility>
// #include <string>
// #include <list>
// #include <set>
#include <conio.h>
// #include <chrono>

// ENDERECAMENTO DA BASE DE DADOS CAN
char* CAN_INTERFACE = "CAN1";
char* CAN_DATABASE  = "database";
char* CAN_CLUSTER   = "NETCAN";
char* NET_ID_SERVO_01 = "1";
char* NET_ID_SERVO_02 = "2";
char* NET_ID_SERVO_03 = "3";
char* NET_ID_SERVO_04 = "4";
char* NET_ID_SERVO_05 = "5";
char* NET_ID_SERVO_06 = "6";

//DECLARACAO DA REDE CAN:
EPOS_NETWORK  epos(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);
//DECLARACAO DAS EPOS:
AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);
AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

void HabilitaEixo(int ID);
void DesabilitaEixo(int ID);
void readIMUs(ThrdStruct &data_struct);
void qASGD(ThrdStruct &data_struct);
void Controle(ThrdStruct &data_struct);
void Logging(ThrdStruct &data_struct);

int main()
{
  using namespace std;
  
  DWORD dwPriority, dwError;
  if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)){
      dwError = GetLastError();
      printf("Error %lu! \n", dwError);
  } else{
      dwPriority = GetPriorityClass( GetCurrentProcess() );
      printf("Process priority class: %lu \n", dwPriority);
  }
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

  auto endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }

  // EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(true);
  eixo_in.WritePDO01();
  auto endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }
  eixo_in.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();
  cout << ".. OK!" << endl;

  mutex imu_mtx;
  float shared_data[18];
  ThrdStruct imu_struct, asgd_struct, control_struct, logging_struct;
  
  thread thr_imus;
  thread thr_qasgd;
  thread thr_controle;
  thread thr_logging;

  thr_imus     = thread(readIMUs, imu_struct);
  thr_qasgd    = thread(qASGD, asgd_struct);
  thr_controle = thread(Controle, control_struct);
  thr_logging  = thread(Logging, logging_struct);


  thr_controle.join();
  thr_logging.join();
  thr_qasgd.join();
  thr_imus.join();

  epos.StopPDOS(1);
  auto endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait){  }

  cout << "Successful exit." << endl;
  cout << "Press [ENTER] to continue." << endl;
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
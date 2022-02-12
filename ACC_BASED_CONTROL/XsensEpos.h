//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#ifndef XSENSEPOS_H
#define XSENSEPOS_H

#include <iostream>
#include <thread>
#include <chrono>
#include "AXIS.h"
#include "EPOS_NETWORK.h"

// ENDERECAMENTO DA BASE DE DADOS CAN
char* CAN_INTERFACE = "CAN1";
char* CAN_DATABASE  = "database";
char* CAN_CLUSTER   = "NETCAN";
// Right Knee
char* NET_ID_SERVO_01  = "1";
char* NET_ID_SENSOR_01 = "2";
// Left Knee
char* NET_ID_SERVO_02 = "3";
// Right Hip 
char* NET_ID_SERVO_03 = "4";
// Left Hip
char* NET_ID_SERVO_04 = "5";

//EPOS USB
//USB_Network * USB_1;
//WORD _eposID = 6;

//DECLARACAO DA REDE CAN:
EPOS_NETWORK  epos(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);
//DECLARACAO DAS EPOS:
AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SENSOR_01);
AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

/* "EPOS" FUNCTIONS */

void IniciaRedeCan()
{
  epos.StartPDOS(1);
  epos.StartPDOS(2);
  epos.StartPDOS(3);
  epos.StartPDOS(4);
  epos.StartPDOS(5);

  for (int i = 0; i < 10; i++)
  {
    // Aguarda tempo
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Sincroniza as epos
    epos.sync();
    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();
    std::cout << ".";
  }

  // EPOS 01
  eixo_out.PDOsetControlWord_FaultReset(true);
  eixo_out.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  eixo_out.PDOsetControlWord_FaultReset(false);
  eixo_out.WritePDO01();

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(true);
  eixo_in.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  eixo_in.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();

  std::cout << " ok!" << std::endl;
}

void HabilitaEixo(int ID)
{
  if ((ID == 2) | (ID == 0)) {

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(true);
    eixo_in.WritePDO01();

    std::cout << "Motor habilitado!" << std::endl;
  }
}

void DesabilitaEixo(int ID)
{
  if ((ID == 2) | (ID == 0)) {

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    std::cout << "Motor desabilitado!" << std::endl;
  }
}

#endif // XSENSEPOS_H

//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\
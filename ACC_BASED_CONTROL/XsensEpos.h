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
AXIS hipRightMotor(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_03);
AXIS hipLeftMotor(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_04);
AXIS kneeRightMotor(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);
AXIS kneeRightEncoder(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SENSOR_01);
AXIS kneeLeftMotor(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);


/* "EPOS" FUNCTIONS */

void IniciaRedeCan()
{
  epos.StartPDOS(1);
  epos.StartPDOS(2);
  epos.StartPDOS(3);
  epos.StartPDOS(4);
  epos.StartPDOS(5);
  epos.StartPDOS(1); // ?
  epos.StartPDOS(2); // ?
  epos.StartPDOS(3); // ?
  epos.StartPDOS(4); // ?
  epos.StartPDOS(5); // ?
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (int i = 0; i < 10; i++)
  {
    // Aguarda tempo
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Sincroniza as epos
    epos.sync();
    kneeRightEncoder.ReadPDO01();
    kneeRightMotor.ReadPDO01();
    //kneeLeftMotor.ReadPDO01();
    //hipRightMotor.ReadPDO01();
    //hipLeftMotor.ReadPDO01();

    std::cout << ".";
  }
  std::cout << " OK!" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
}

void HabilitaEixo(int ID)
{
  if ((ID == 2) | (ID == 0)) {

    kneeRightMotor.PDOsetControlWord_SwitchOn(false);
    kneeRightMotor.PDOsetControlWord_EnableVoltage(true);
    kneeRightMotor.PDOsetControlWord_QuickStop(true);
    kneeRightMotor.PDOsetControlWord_EnableOperation(false);
    kneeRightMotor.WritePDO01();
    /*
    kneeLeftMotor.PDOsetControlWord_SwitchOn(false);
    kneeLeftMotor.PDOsetControlWord_EnableVoltage(true);
    kneeLeftMotor.PDOsetControlWord_QuickStop(true);
    kneeLeftMotor.PDOsetControlWord_EnableOperation(false);
    kneeLeftMotor.WritePDO01();

    hipRightMotor.PDOsetControlWord_SwitchOn(false);
    hipRightMotor.PDOsetControlWord_EnableVoltage(true);
    hipRightMotor.PDOsetControlWord_QuickStop(true);
    hipRightMotor.PDOsetControlWord_EnableOperation(false);
    hipRightMotor.WritePDO01();

    hipLeftMotor.PDOsetControlWord_SwitchOn(false);
    hipLeftMotor.PDOsetControlWord_EnableVoltage(true);
    hipLeftMotor.PDOsetControlWord_QuickStop(true);
    hipLeftMotor.PDOsetControlWord_EnableOperation(false);
    hipLeftMotor.WritePDO01();
    */
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    kneeRightMotor.PDOsetControlWord_SwitchOn(true);
    kneeRightMotor.PDOsetControlWord_EnableVoltage(true);
    kneeRightMotor.PDOsetControlWord_QuickStop(true);
    kneeRightMotor.PDOsetControlWord_EnableOperation(false);
    kneeRightMotor.WritePDO01();
    /*
    kneeLeftMotor.PDOsetControlWord_SwitchOn(true);
    kneeLeftMotor.PDOsetControlWord_EnableVoltage(true);
    kneeLeftMotor.PDOsetControlWord_QuickStop(true);
    kneeLeftMotor.PDOsetControlWord_EnableOperation(false);
    kneeLeftMotor.WritePDO01();

    hipRightMotor.PDOsetControlWord_SwitchOn(true);
    hipRightMotor.PDOsetControlWord_EnableVoltage(true);
    hipRightMotor.PDOsetControlWord_QuickStop(true);
    hipRightMotor.PDOsetControlWord_EnableOperation(false);
    hipRightMotor.WritePDO01();

    hipLeftMotor.PDOsetControlWord_SwitchOn(true);
    hipLeftMotor.PDOsetControlWord_EnableVoltage(true);
    hipLeftMotor.PDOsetControlWord_QuickStop(true);
    hipLeftMotor.PDOsetControlWord_EnableOperation(false);
    hipLeftMotor.WritePDO01();
    */
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    kneeRightMotor.PDOsetControlWord_SwitchOn(true);
    kneeRightMotor.PDOsetControlWord_EnableVoltage(true);
    kneeRightMotor.PDOsetControlWord_QuickStop(true);
    kneeRightMotor.PDOsetControlWord_EnableOperation(true);
    kneeRightMotor.WritePDO01();
    /*
    kneeLeftMotor.PDOsetControlWord_SwitchOn(true);
    kneeLeftMotor.PDOsetControlWord_EnableVoltage(true);
    kneeLeftMotor.PDOsetControlWord_QuickStop(true);
    kneeLeftMotor.PDOsetControlWord_EnableOperation(true);
    kneeLeftMotor.WritePDO01();

    hipRightMotor.PDOsetControlWord_SwitchOn(true);
    hipRightMotor.PDOsetControlWord_EnableVoltage(true);
    hipRightMotor.PDOsetControlWord_QuickStop(true);
    hipRightMotor.PDOsetControlWord_EnableOperation(true);
    hipRightMotor.WritePDO01();

    hipLeftMotor.PDOsetControlWord_SwitchOn(true);
    hipLeftMotor.PDOsetControlWord_EnableVoltage(true);
    hipLeftMotor.PDOsetControlWord_QuickStop(true);
    hipLeftMotor.PDOsetControlWord_EnableOperation(true);
    hipLeftMotor.WritePDO01();
    */
    std::cout << "Motores habilitados!" << std::endl;
  }
}

void DesabilitaEixo(int ID)
{
  if ((ID == 2) | (ID == 0)) {

    kneeRightMotor.PDOsetControlWord_SwitchOn(true);
    kneeRightMotor.PDOsetControlWord_EnableVoltage(true);
    kneeRightMotor.PDOsetControlWord_QuickStop(true);
    kneeRightMotor.PDOsetControlWord_EnableOperation(false);
    kneeRightMotor.WritePDO01();
    /*
    kneeLeftMotor.PDOsetControlWord_SwitchOn(true);
    kneeLeftMotor.PDOsetControlWord_EnableVoltage(true);
    kneeLeftMotor.PDOsetControlWord_QuickStop(true);
    kneeLeftMotor.PDOsetControlWord_EnableOperation(false);
    kneeLeftMotor.WritePDO01();

    hipRightMotor.PDOsetControlWord_SwitchOn(true);
    hipRightMotor.PDOsetControlWord_EnableVoltage(true);
    hipRightMotor.PDOsetControlWord_QuickStop(true);
    hipRightMotor.PDOsetControlWord_EnableOperation(false);
    hipRightMotor.WritePDO01();

    hipLeftMotor.PDOsetControlWord_SwitchOn(true);
    hipLeftMotor.PDOsetControlWord_EnableVoltage(true);
    hipLeftMotor.PDOsetControlWord_QuickStop(true);
    hipLeftMotor.PDOsetControlWord_EnableOperation(false);
    hipLeftMotor.WritePDO01();
    */
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    kneeRightMotor.PDOsetControlWord_SwitchOn(false);
    kneeRightMotor.PDOsetControlWord_EnableVoltage(true);
    kneeRightMotor.PDOsetControlWord_QuickStop(true);
    kneeRightMotor.PDOsetControlWord_EnableOperation(false);
    kneeRightMotor.WritePDO01();
    /*
    kneeLeftMotor.PDOsetControlWord_SwitchOn(false);
    kneeLeftMotor.PDOsetControlWord_EnableVoltage(true);
    kneeLeftMotor.PDOsetControlWord_QuickStop(true);
    kneeLeftMotor.PDOsetControlWord_EnableOperation(false);
    kneeLeftMotor.WritePDO01();

    hipRightMotor.PDOsetControlWord_SwitchOn(false);
    hipRightMotor.PDOsetControlWord_EnableVoltage(true);
    hipRightMotor.PDOsetControlWord_QuickStop(true);
    hipRightMotor.PDOsetControlWord_EnableOperation(false);
    hipRightMotor.WritePDO01();

    hipLeftMotor.PDOsetControlWord_SwitchOn(false);
    hipLeftMotor.PDOsetControlWord_EnableVoltage(true);
    hipLeftMotor.PDOsetControlWord_QuickStop(true);
    hipLeftMotor.PDOsetControlWord_EnableOperation(false);
    hipLeftMotor.WritePDO01();
    */
    std::cout << "Motores desabilitados!" << std::endl;
  }
}

void ResetRedeCan()
{
  // EPOS 04
  std::cout << " 0%";
  hipRightMotor.PDOsetControlWord_FaultReset(true);
  hipRightMotor.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  hipRightMotor.PDOsetControlWord_FaultReset(false);
  hipRightMotor.WritePDO01();

  std::cout << "  10%";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // EPOS 05
  std::cout << "  20%";
  hipLeftMotor.PDOsetControlWord_FaultReset(true);
  hipLeftMotor.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  hipLeftMotor.PDOsetControlWord_FaultReset(false);
  hipLeftMotor.WritePDO01();

  std::cout << "  30%";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  // EPOS 02
  std::cout << "  40%";
  kneeRightEncoder.PDOsetControlWord_FaultReset(true);
  kneeRightEncoder.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  kneeRightEncoder.PDOsetControlWord_FaultReset(false);
  kneeRightEncoder.WritePDO01();

  std::cout << "  50%";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  // EPOS 03
  std::cout << "  60%";
  kneeLeftMotor.PDOsetControlWord_FaultReset(true);
  kneeLeftMotor.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  kneeLeftMotor.PDOsetControlWord_FaultReset(false);
  kneeLeftMotor.WritePDO01();

  std::cout << "  70%";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  // EPOS 01
  std::cout << "  80%";
  kneeRightMotor.PDOsetControlWord_FaultReset(true);
  kneeRightMotor.WritePDO01();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  kneeRightMotor.PDOsetControlWord_FaultReset(false);
  kneeRightMotor.WritePDO01();

  std::cout << "  90%";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << " Reset OK!" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
}

#endif // XSENSEPOS_H

//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\
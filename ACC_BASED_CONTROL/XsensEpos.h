#ifndef XSENSEPOS_H
#define XSENSEPOS_H

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <atomic>

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

// Threads readiness flags: (globais)
std::atomic<bool> imu_isready(false);
std::atomic<bool> asgd_isready(false);
std::atomic<bool> control_isready(false);
std::atomic<bool> logging_isready(false);

#endif // XSENSEPOS_H

//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\////\//
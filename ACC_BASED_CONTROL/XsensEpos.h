#ifndef XSENSEPOS_H
#define XSENSEPOS_H

#define PRIORITY  0

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include <atomic>

typedef long long int llint;

// ENDERECAMENTO DA BASE DE DADOS CAN
static char* CAN_INTERFACE = "CAN1";
static char* CAN_DATABASE  = "database";
static char* CAN_CLUSTER   = "NETCAN";
static char* NET_ID_SERVO_01 = "1";
static char* NET_ID_SERVO_02 = "2";
static char* NET_ID_SERVO_03 = "3";
static char* NET_ID_SERVO_04 = "4";
static char* NET_ID_SERVO_05 = "5";
static char* NET_ID_SERVO_06 = "6";

//DECLARACAO DA REDE CAN:
static EPOS_NETWORK  epos(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);
//DECLARACAO DAS EPOS:
static AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);
static AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

// Threads readiness flags: (globais)
static short imu_isready(false);
static short asgd_isready(false);
static short control_isready(false);
static short logging_isready(false);

#endif // XSENSEPOS_H

//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/
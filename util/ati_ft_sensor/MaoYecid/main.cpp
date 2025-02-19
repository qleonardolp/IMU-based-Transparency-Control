﻿/*
Programa para controle do exo
v1 creada por Juan Carlos Perez Ibarra
25/Oct/2020
v2 modificado por Wilian
24/11/2021
*/

#define ITK_NOEXCEPT noexcept

#ifdef _WIN32
#include <WinSock2.h>
//#include <conio.h>
#endif

#define verbose_m 0

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <stdio.h>
#include <time.h>

#include <stdlib.h>
#include <stdint.h>

//---------------------------------------//
// Headers for EMGs Delsys
//---------------------------------------//
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

//---------------------------------------//
// Headers for XSens
//---------------------------------------//
#include "include/xsensdeviceapi.h" // The Xsens device API header
#include "conio.h"                  // For non ANSI _kbhit() and _getch()

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include "xsens/xsmutex.h"
//---------------------------------------//

#include <vector>
#include <algorithm>
#include <array>
#include <math.h>

//---------------------------------------//
// Headers for CAN/EPOS
//---------------------------------------//
#include "SerialPort.h"

//---------------------------------------//
// Headers for CAN/EPOS
//---------------------------------------//
#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "FORCE_SENSOR.h"
#include <DefinitionsEPOS.h>
#include <ReabRob_EPOS.h>
//---------------------------------------//

#include "declarations.h"
#include "declarations_epos.h"
#include "declarations_xsens.h"
#include "declarations_emgs.h"


#include<Utils_Axia80M50.h>



ValuesAxia80M50 valuesAxia;
std::mutex mtx_valuesAxia;


using namespace std;

void print_cabecalho(char *titulo);
void esperar_n_seg(int sec);
void start_transmissao_rede_epos();
void init_comm_eixos();
void Habilita_Eixo(int ID);
void Desabilita_Eixo(int ID);
void reset_falhas();
void define_origen();
void read_sensors();

void inicializa_controle();

void leitura_arduino(int T_ard);
void controle_exo(int T_exo, int com_setpoint, int com_controller);
void leitura_xsens(int T_imu);
void leitura_emg(int T_emg);
void leitura_esp32(int T_esp);
void leitura_sensores(int T_sensors2);
void leitura_atimx(int T_sensors2);


int salva_dataloggers(double (*datalog_ard)[n_datalogs_ard], size_t rows);


int main()
{
    /*
    printf("Flag 1");
    
    WORD _epos4 = 6;

    float position = 0.0f;
    USB_1 = new USB_Network("EPOS4","MAXON SERIAL V2","USB","USB0");
    if(!USB_1->connect()){
      printf("Não foi possivel conetar \n");
      Sleep(1000);
      return 0;
    }
    USB_1->configure(_epos4);
    USB_1->setState(_epos4,1);
    for(int i = 0 ; i < 100 ; i++){
      position = USB_1->positionRead(_epos4);
      printf("\n pos: %.3f",position);
      Sleep(500);
    }
    */

    //INICIALIZACAO DE TELA
    print_cabecalho("");

    //START DE TRANSMISS�O DA REDE
    cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << endl;
    start_transmissao_rede_epos();
    init_comm_eixos();
    
    USB_1 = new USB_Network("EPOS4","MAXON SERIAL V2","USB","USB0");
    if(!USB_1->connect()){
      printf("Não foi possivel conetar \n");
      Sleep(1000);
    }
    USB_1->configure(_eposID);
    USB_1->setState(_eposID,1);

    //LOOP DE CONTROLE DE COMANDOS
    do
    {
        // MENU PRINCIPAL

        print_cabecalho("");
        cout << ")SELECCIONA UMA OPCAO:" << endl;
        cout << endl;
        cout << "[  0 ] - ENCERRAR PROGRAMA" << endl;
        cout << "[  1 ] - DEFINIR POSIÇÃO DE ORIGEM" << endl;
        cout << "[  2 ] - RESET DE FALHAS" << endl;
        cout << "[  3 ] - LEITURA DADOS ESP 32" << endl;
        cout << "[  4 ] - LEITURA DADOS EMG" << endl;
        cout << "[  5 ] - LEITURA DADOS ARDUINO" << endl;
        cout << "[  6 ] - LEITURA DADOS IMUs XSens" << endl;
        cout << "[  7 ] - CONTROLE EXO" << endl;
        cout << "[  8 ] - CONTROLE + SENSORES" << endl;
        cout << "[  9 ] - CONTROLE + POFS" << endl;
        cout << "[ 10 ] - LEITURA EPOS + Arduino" << endl;
        cout << endl;
        cout << "OPCAO: ";

        //VERIFICA O COMANDO DO OPERADOR
        scanf("%d", &COMMAND_KEY);
        cout << endl;

        if (COMMAND_KEY == 0)
        {
            cout << "Programa finalizado por el usuario" << endl;
            break;
        }

        if (COMMAND_KEY == 1)
        {
            print_cabecalho("DEFINE POSIÇÃO DE ORIGEN - EPOS CANOpen");

            define_origen();
            read_sensors();

            continue;
        }

        if (COMMAND_KEY == 2)
        {
            print_cabecalho("RESET DE FALHAS - EPOS CANOpen");

            reset_falhas();
            
            continue;
        }

        int arr[] = {3, 4, 5, 6, 7, 8, 9,10};
        std::vector<int> vect(arr, arr + 8);
        if (std::count(vect.begin(), vect.end(), COMMAND_KEY) == 0)
        {
            cout << "OPCION NO ENCONTRADA" << endl;
            esperar_n_seg(1);
            continue;
        }
        else
        {

            if (COMMAND_KEY == 3)
            {
                print_cabecalho("PROGRAMA PARA LEITURA DO ESP32");

                flag_arduino_multi_ard = true;
                flag_arduino_multi_esp = false;
                flag_arduino_multi_exo = true;
                flag_arduino_multi_imu = true;
            }

            if (COMMAND_KEY == 4)
            {
                print_cabecalho("PROGRAMA PARA LEITURA DAS EMGs (Delsys)");

                flag_arduino_multi_emg = false;
                flag_arduino_multi_ard = true;
                flag_arduino_multi_esp = true;
                flag_arduino_multi_exo = true;
                flag_arduino_multi_imu = true;
            }

            if (COMMAND_KEY == 5)
            {
                print_cabecalho("PROGRAMA PARA LEITURA DO ARDUINO");

                flag_arduino_multi_ard = false;
                flag_arduino_multi_esp = true;
                flag_arduino_multi_exo = true;
                flag_arduino_multi_imu = true;
            }

            if (COMMAND_KEY == 6)
            {
                print_cabecalho("PROGRAMA PARA LEITURA DAS IMUs (XSens)");

                flag_arduino_multi_ard = true;
                flag_arduino_multi_esp = true;
                flag_arduino_multi_exo = true;
                flag_arduino_multi_imu = false;
            }

            if (COMMAND_KEY == 7)
            {
                print_cabecalho("INTERFACE DE CONTROLE EXO - TAU");

                cout << "SELECCIONA UMA OPCAO DE CONTROL:" << endl;
                cout << endl;
                cout << "[1] - IMPEDANCIA" << endl;
                cout << "[2] - TORQUE" << endl;
                cout << "[3] - IMP. ZERO + RUIDO" << endl;
                cout << endl;
                cout << "OPCAO: ";

                //VERIFICA O COMANDO DO OPERADOR
                scanf("%d", &COMMAND_CONTROLLER);
                cout << endl;
                cout << endl;

                cout << "SELECCIONA UMA OPCAO DE SETPOINT:" << endl;
                cout << endl;
                cout << "[1] - TRIANGULAR" << endl;
                cout << "[2] - SENOIDAL" << endl;
                cout << "[3] - RAMPAS" << endl;
                cout << "[4] - STEPS" << endl;
                cout << endl;
                cout << "OPCAO: ";

                //VERIFICA O COMANDO DO OPERADOR
                scanf("%d", &COMMAND_SETPOINT);
                cout << endl;
                cout << endl;

                flag_arduino_multi_ard = true;
                flag_arduino_multi_esp = true;
                flag_arduino_multi_exo = false;
                flag_arduino_multi_imu = true;
                flag_arduino_multi_emg = true;
            
            }

            if (COMMAND_KEY == 8)
            {
                print_cabecalho("PROGRAMA PARA CONTROLE E LEITURA DOS SENSORES");

                cout << "SELECCIONA UMA OPCAO DE CONTROL:" << endl;
                cout << endl;
                cout << "[1] - IMPEDANCIA" << endl;
                cout << "[2] - TORQUE" << endl;
                cout << "[3] - IMP. ZERO + RUIDO" << endl;
                cout << endl;
                cout << "OPCAO: ";

                //VERIFICA O COMANDO DO OPERADOR
                scanf("%d", &COMMAND_CONTROLLER);
                cout << endl;
                cout << endl;

                cout << "SELECCIONA UMA OPCAO DE SETPOINT:" << endl;
                cout << endl;
                cout << "[1] - TRIANGULAR" << endl;
                cout << "[2] - SENOIDAL" << endl;
                cout << "[3] - RAMPAS" << endl;
                cout << "[4] - STEPS" << endl;
                cout << endl;
                cout << "OPCAO: ";

                //VERIFICA O COMANDO DO OPERADOR
                scanf("%d", &COMMAND_SETPOINT);
                cout << endl;
                cout << endl;

                 if(COMMAND_CONTROLLER == 1){
                        std::cout<<"\n [Kv;Bv] 0) [0;0]";
                        std::cout<<"\n         1) [60;1]";
                        std::cout<<"\n         2) [60;1]";
                        std::cout<<"\n         3) [0;2] - [0;4]";
                        cout << "\nOPCAO: ";
                        std::cin>>OPC_K;

                 }
                
            
                flag_arduino_multi_ard = false;
                flag_arduino_multi_esp = false;
                flag_arduino_multi_exo = false;
                flag_arduino_multi_imu = false;
            
            }

        
            if (COMMAND_KEY == 9)
            {
                print_cabecalho("PROGRAMA PARA CONTROLE E POFs");
            
                flag_arduino_multi_ard = true;
                flag_arduino_multi_esp = false;
                flag_arduino_multi_exo = false;
                flag_arduino_multi_imu = true;

                // cout << "Corregir que pasa cuando no se inicializa el otro sensor..." << endl;

                //esperar_n_seg(5);

                //continue;
            }

            if (COMMAND_KEY == 10)
            {
                print_cabecalho("Leitura sensores");



                
                flag_arduino_multi_esp = !false;
                flag_arduino_multi_exo = !false;
                flag_arduino_multi_imu = !false;
                
                flag_arduino_multi_ard     = !false;
                flag_arduino_multi_atimx   = false;
                flag_arduino_multi_sensors = false;
           
            }


            cout << endl;
            cout << "DEFINA O TEMPO DE EXECUCAO (s): ";
            scanf("%d", &exec_time);
            cout << endl;
        }

        std::thread thr_emgdelsys;
        std::thread thr_arduino;
        std::thread thr_esp32;
        std::thread thr_control;
        std::thread thr_imuxsens;
        std::thread thr_sensores;
        std::thread thr_atimx;

        // Funcion de leitura do arduino
        if (!flag_arduino_multi_ard)
            thr_arduino   = std::thread(leitura_arduino, exec_time * samples_per_second_ard);

        // Funcion de leitura do esp32
        if (!flag_arduino_multi_esp)
            thr_esp32     = std::thread(leitura_esp32, exec_time * samples_per_second_esp);

        // Funcion de controle do exo
        if (!flag_arduino_multi_exo)
            thr_control   = std::thread(controle_exo, exec_time * samples_per_second_exo, COMMAND_SETPOINT, COMMAND_CONTROLLER);

        // Funcion de leitura das EMGs
        if (!flag_arduino_multi_emg)
            thr_emgdelsys = std::thread(leitura_emg, exec_time * samples_per_second_emg);

        // Funcion de leitura das XSens
        if (!flag_arduino_multi_imu)
            thr_imuxsens  = std::thread(leitura_xsens, exec_time * samples_per_second_imu);

        if (!flag_arduino_multi_sensors)
            thr_sensores  = std::thread(leitura_sensores, exec_time);

        if (!flag_arduino_multi_atimx)
            thr_atimx  = std::thread(leitura_atimx, exec_time * samples_per_second_atimx);

        cout << "ARD: " << flag_arduino_multi_ard << endl;
        cout << "ESP: " << flag_arduino_multi_esp << endl;
        cout << "EXO: " << flag_arduino_multi_exo << endl;
        cout << "IMU: " << flag_arduino_multi_imu << endl;
        cout << "EMG: " << flag_arduino_multi_emg << endl;
        cout << "SENSORS: " << flag_arduino_multi_sensors << endl;
        cout << "ATIMX: " << flag_arduino_multi_atimx << endl;

        if (thr_arduino.joinable())
            thr_arduino.join();

        if (thr_esp32.joinable())
            thr_esp32.join();

        if (thr_control.joinable())
            thr_control.join();

        if (thr_emgdelsys.joinable())
            thr_emgdelsys.join();

        if (thr_imuxsens.joinable())
            thr_imuxsens.join();

        if (thr_sensores.joinable())
            thr_sensores.join();

        if (thr_atimx.joinable())
            thr_atimx.join();

    } while (COMMAND_KEY != 0);

    //FINALIZA A COMUNICA��O COM AS EPOS
    epos.StopPDOS(1);

    esperar_n_seg(1);
    cout << "FIM DE PROGRAMA" << endl;
    esperar_n_seg(1);

    system(CLC);
}

void print_cabecalho(char *titulo)
{
    system(CLC);

    cout << endl;
    cout << "********************************************" << endl;
    cout << "*       INTERFACE DE CONTROLE EXO-TAU      *" << endl;
    cout << "* ESCOLA DE ENGENHARIA DE SAO CARLOS - USP *" << endl;
    cout << "*   LABORATORIO DE REABILITACAO ROBOTICA   *" << endl;
    cout << "********************************************" << endl;
    cout << endl;
    if (titulo != "")
    {
        cout << "********************************************" << endl;
        cout << titulo << endl;
        cout << "********************************************" << endl;
    }
}

void esperar_n_seg(int sec)
{
    endwait = clock() + sec * CLOCKS_PER_SEC;
    while (clock() < endwait)
    {
    }
}

void start_transmissao_rede_epos()
{
    epos.StartPDOS(1);
    epos.StartPDOS(2);
    epos.StartPDOS(3);
    epos.StartPDOS(4);
    epos.StartPDOS(5);
    epos.StartPDOS(1);
    epos.StartPDOS(2);
    epos.StartPDOS(3);
    epos.StartPDOS(4);
    epos.StartPDOS(5);

}

void init_comm_eixos()
{
    for (int i = 0; i < 10; i++)
    {

        //Aguarda tempo
        endwait = clock() + 1 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        //Sincroniza as epos
        epos.sync();
        encoder_knee_right.ReadPDO01();
        servo_knee_right.ReadPDO01();
        
//        encoder_knee_left.ReadPDO01();
        servo_knee_left.ReadPDO01();

        servo_hip_right.ReadPDO01();
        servo_hip_left.ReadPDO01();

        printf(".");
    }
}

void Habilita_Eixo(int ID)
{

    if ((ID == 2) | (ID == 0))
    {
        
        servo_hip_right.PDOsetControlWord_SwitchOn(false);
        servo_hip_right.PDOsetControlWord_EnableVoltage(true);
        servo_hip_right.PDOsetControlWord_QuickStop(true);
        servo_hip_right.PDOsetControlWord_EnableOperation(false);
        servo_hip_right.WritePDO01();

        servo_hip_left.PDOsetControlWord_SwitchOn(false);
        servo_hip_left.PDOsetControlWord_EnableVoltage(true);
        servo_hip_left.PDOsetControlWord_QuickStop(true);
        servo_hip_left.PDOsetControlWord_EnableOperation(false);
        servo_hip_left.WritePDO01();

        servo_knee_right.PDOsetControlWord_SwitchOn(false);
        servo_knee_right.PDOsetControlWord_EnableVoltage(true);
        servo_knee_right.PDOsetControlWord_QuickStop(true);
        servo_knee_right.PDOsetControlWord_EnableOperation(false);
        servo_knee_right.WritePDO01();

        servo_knee_left.PDOsetControlWord_SwitchOn(false);
        servo_knee_left.PDOsetControlWord_EnableVoltage(true);
        servo_knee_left.PDOsetControlWord_QuickStop(true);
        servo_knee_left.PDOsetControlWord_EnableOperation(false);
        servo_knee_left.WritePDO01();

        printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");

        endwait = clock() + 0.5 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        servo_hip_right.PDOsetControlWord_SwitchOn(true);
        servo_hip_right.PDOsetControlWord_EnableVoltage(true);
        servo_hip_right.PDOsetControlWord_QuickStop(true);
        servo_hip_right.PDOsetControlWord_EnableOperation(false);
        servo_hip_right.WritePDO01();

        servo_hip_left.PDOsetControlWord_SwitchOn(true);
        servo_hip_left.PDOsetControlWord_EnableVoltage(true);
        servo_hip_left.PDOsetControlWord_QuickStop(true);
        servo_hip_left.PDOsetControlWord_EnableOperation(false);
        servo_hip_left.WritePDO01();

        servo_knee_right.PDOsetControlWord_SwitchOn(true);
        servo_knee_right.PDOsetControlWord_EnableVoltage(true);
        servo_knee_right.PDOsetControlWord_QuickStop(true);
        servo_knee_right.PDOsetControlWord_EnableOperation(false);
        servo_knee_right.WritePDO01();

        servo_knee_left.PDOsetControlWord_SwitchOn(true);
        servo_knee_left.PDOsetControlWord_EnableVoltage(true);
        servo_knee_left.PDOsetControlWord_QuickStop(true);
        servo_knee_left.PDOsetControlWord_EnableOperation(false);
        servo_knee_left.WritePDO01();

        endwait = clock() + 0.5 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        servo_hip_right.PDOsetControlWord_SwitchOn(true);
        servo_hip_right.PDOsetControlWord_EnableVoltage(true);
        servo_hip_right.PDOsetControlWord_QuickStop(true);
        servo_hip_right.PDOsetControlWord_EnableOperation(true);
        servo_hip_right.WritePDO01();

        servo_hip_left.PDOsetControlWord_SwitchOn(true);
        servo_hip_left.PDOsetControlWord_EnableVoltage(true);
        servo_hip_left.PDOsetControlWord_QuickStop(true);
        servo_hip_left.PDOsetControlWord_EnableOperation(true);
        servo_hip_left.WritePDO01();

        servo_knee_right.PDOsetControlWord_SwitchOn(true);
        servo_knee_right.PDOsetControlWord_EnableVoltage(true);
        servo_knee_right.PDOsetControlWord_QuickStop(true);
        servo_knee_right.PDOsetControlWord_EnableOperation(true);
        servo_knee_right.WritePDO01();

        servo_knee_left.PDOsetControlWord_SwitchOn(true);
        servo_knee_left.PDOsetControlWord_EnableVoltage(true);
        servo_knee_left.PDOsetControlWord_QuickStop(true);
        servo_knee_left.PDOsetControlWord_EnableOperation(true);
        servo_knee_left.WritePDO01();

    }
}

void Desabilita_Eixo(int ID)
{

    if ((ID == 2) | (ID == 0))
    {
        printf("\nDESABILITANDO O MOTOR E CONTROLE");

        servo_hip_right.PDOsetControlWord_SwitchOn(true);
        servo_hip_right.PDOsetControlWord_EnableVoltage(true);
        servo_hip_right.PDOsetControlWord_QuickStop(true);
        servo_hip_right.PDOsetControlWord_EnableOperation(false);
        servo_hip_right.WritePDO01();

        servo_hip_left.PDOsetControlWord_SwitchOn(true);
        servo_hip_left.PDOsetControlWord_EnableVoltage(true);
        servo_hip_left.PDOsetControlWord_QuickStop(true);
        servo_hip_left.PDOsetControlWord_EnableOperation(false);
        servo_hip_left.WritePDO01();

        servo_knee_right.PDOsetControlWord_SwitchOn(true);
        servo_knee_right.PDOsetControlWord_EnableVoltage(true);
        servo_knee_right.PDOsetControlWord_QuickStop(true);
        servo_knee_right.PDOsetControlWord_EnableOperation(false);
        servo_knee_right.WritePDO01();

        servo_knee_left.PDOsetControlWord_SwitchOn(true);
        servo_knee_left.PDOsetControlWord_EnableVoltage(true);
        servo_knee_left.PDOsetControlWord_QuickStop(true);
        servo_knee_left.PDOsetControlWord_EnableOperation(false);
        servo_knee_left.WritePDO01();

        endwait = clock() + 0.5 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        servo_hip_right.PDOsetControlWord_SwitchOn(false);
        servo_hip_right.PDOsetControlWord_EnableVoltage(true);
        servo_hip_right.PDOsetControlWord_QuickStop(true);
        servo_hip_right.PDOsetControlWord_EnableOperation(false);
        servo_hip_right.WritePDO01();

        servo_hip_left.PDOsetControlWord_SwitchOn(false);
        servo_hip_left.PDOsetControlWord_EnableVoltage(true);
        servo_hip_left.PDOsetControlWord_QuickStop(true);
        servo_hip_left.PDOsetControlWord_EnableOperation(false);
        servo_hip_left.WritePDO01();

        servo_knee_right.PDOsetControlWord_SwitchOn(false);
        servo_knee_right.PDOsetControlWord_EnableVoltage(true);
        servo_knee_right.PDOsetControlWord_QuickStop(true);
        servo_knee_right.PDOsetControlWord_EnableOperation(false);
        servo_knee_right.WritePDO01();

        servo_knee_left.PDOsetControlWord_SwitchOn(false);
        servo_knee_left.PDOsetControlWord_EnableVoltage(true);
        servo_knee_left.PDOsetControlWord_QuickStop(true);
        servo_knee_left.PDOsetControlWord_EnableOperation(false);
        servo_knee_left.WritePDO01();

    }
}

void reset_falhas()
{

      //EPOS 04
    servo_hip_right.PDOsetControlWord_FaultReset(true);
    servo_hip_right.WritePDO01();

    printf("\nResetando as falhas.");

    esperar_n_seg(1);

    printf("..");

    //EPOS 04
    servo_hip_right.PDOsetControlWord_FaultReset(false);
    servo_hip_right.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");


    //EPOS 05
    servo_hip_left.PDOsetControlWord_FaultReset(true);
    servo_hip_left.WritePDO01();

    printf("\nResetando as falhas.");

    esperar_n_seg(1);

    printf("..");

    //EPOS 05
    servo_hip_left.PDOsetControlWord_FaultReset(false);
    servo_hip_left.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");
  
   
  //EPOS 02
    encoder_knee_right.PDOsetControlWord_FaultReset(true);
    encoder_knee_right.WritePDO01();

    printf("\nResetando as falhas.");

    esperar_n_seg(1);

    printf("..");

    //EPOS 02
    encoder_knee_right.PDOsetControlWord_FaultReset(false);
    encoder_knee_right.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");

    //EPOS 03
    servo_knee_left.PDOsetControlWord_FaultReset(true);
    servo_knee_left.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");

    //EPOS 03
    servo_knee_left.PDOsetControlWord_FaultReset(false);
    servo_knee_left.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");

    //EPOS 01
    servo_knee_right.PDOsetControlWord_FaultReset(true);
    servo_knee_right.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");

    //EPOS 01
    servo_knee_right.PDOsetControlWord_FaultReset(false);
    servo_knee_right.WritePDO01();

    printf("..");


    esperar_n_seg(1);

    printf("OK");
}

void define_origen()
{

 int total_time = 0;

  SerialPort arduino(port_name);
     if (arduino.isConnected()) cout << "Connection Established" << endl;
    else cout << "ERROR, check port name";

        arduino.writeSerialPort( "X", 1 );

           int read_result = 0;
        do {
          read_result += arduino.readSerialPort(incomingData + read_result, ENCODERS_NUMBER*sizeof(int16_t));
        } while(  read_result < ENCODERS_NUMBER*sizeof(int16_t) );

        int16_t *encoderdata = (int16_t*) incomingData;

         total_time=0;
        
         do{  
                   arduino.writeSerialPort( "X", 1 );
        int read_result = 0;
        do {
          read_result += arduino.readSerialPort(incomingData + read_result, ENCODERS_NUMBER*sizeof(int16_t));
        } while(  read_result < ENCODERS_NUMBER*sizeof(int16_t) );

        int16_t *encoderdata = (int16_t*) incomingData;

       total_time+=1;
        
       } while(total_time < 10);
      
      
         /*
     float enc1 = 0.0f;
     float enc2 = 0.0f;  

      ArduinoData fromArduino;

       //   int total_time=0;
        
      //   do{  

      mtx_readSensors.lock();
      fromArduino.enc1 = arduinoData.enc1;
      fromArduino.enc2 = arduinoData.enc2;
      fromArduino.fsr1 = arduinoData.fsr1;
      fromArduino.fsr2 = arduinoData.fsr2;
      mtx_readSensors.unlock();
      */
     //        total_time+=1;
        
    //   } while(total_time < 10);

   		ZERO_SENSOR_HIP_right = encoderdata[0];
    //   printf("\nPOSICAO CALIBRADA ENCODER hip right: %f deg", ZERO_SENSOR_HIP_right); 
   		ZERO_SENSOR_HIP_left = encoderdata[1];
     //  printf("\nPOSICAO CALIBRADA ENCODER hip left: %f deg", ZERO_SENSOR_HIP_left);
            /////////////////////

         epos.sync();

    printf("Definindo Origem... ");

    esperar_n_seg(1);

    printf("...");

    servo_hip_right.ReadPDO01();
    ZERO_SERVO_HIP_right = servo_hip_right.PDOgetActualPosition();

    servo_hip_left.ReadPDO01();
    ZERO_SERVO_HIP_left = servo_hip_left.PDOgetActualPosition();

    encoder_knee_right.ReadPDO01();
    ZERO_SENSOR_KNEE_right = -encoder_knee_right.PDOgetActualPosition();

    servo_knee_right.ReadPDO01();
    ZERO_SERVO_KNEE_right = servo_knee_right.PDOgetActualPosition();
    
    ZERO_SENSOR_KNEE_left = -USB_1->positionRead(_eposID);

    servo_knee_left.ReadPDO01();
    ZERO_SERVO_KNEE_left = servo_knee_left.PDOgetActualPosition();


    printf(" ... origem definida.\n");
}

void read_sensors()
  {	

 //************************************************************************************************************************
	// LEITURA DA POSIÇÃO ATUAL
	//************************************************************************************************************************
	
	
		//ZERA CONTADOR DE CICLOS
	 int total_time = 0;
		
		epos.sync();
		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
		epos.sync();
		
//////////////////////////////

        //Habilita a porta serial (Arduino)
    
    SerialPort arduino(port_name);
      if (arduino.isConnected()) cout << "Connection Established" << endl;
      else cout << "ERROR, check port name";

  //////////////////////

		do{
            
      total_time += 1;

/////////////////////////
      
            // Leitura Serial (Arduino)
    
      arduino.writeSerialPort( "X", 1 );
            
      int read_result = 0;
      
      do {
          read_result += arduino.readSerialPort(incomingData + read_result, ENCODERS_NUMBER*sizeof(int16_t));
        } while(  read_result < ENCODERS_NUMBER*sizeof(int16_t) );
      
      int16_t *encoderdata = (int16_t*) incomingData;
   

////////////////////////


		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
		epos.sync();
	
		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");
    printf("\n********************************************************************************");
		encoder_knee_right.ReadPDO01();
    printf("\nPOSICAO CALIBRADA ENCODER knee right: %d ppr", (-encoder_knee_right.PDOgetActualPosition())); 
	  servo_knee_right.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR knee right: %d ppr", (servo_knee_right.PDOgetActualPosition()));
    printf("\n********************************************************************************");
    printf("\nPOSICAO CALIBRADA ENCODER knee left: %.2f ppr", (-USB_1->positionRead(_eposID))); 
	  servo_knee_left.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR knee left: %d ppr", (servo_knee_left.PDOgetActualPosition()));
    printf("\n********************************************************************************");
    printf("\nPOSICAO CALIBRADA ENCODER hip right: %d  bits", -(encoderdata[0])); 
    servo_hip_right.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR hip right: %d ppr", (servo_hip_right.PDOgetActualPosition()));
    printf("\nPOSICAO CALIBRADA ENCODER hip left: %d bits", -(encoderdata[1])); 
    servo_hip_left.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR hip left: %d ppr", (servo_hip_left.PDOgetActualPosition()));
    printf("\n\n********************************************************************************");


    printf("\nPOSIÇÃO DO ROBO");

		theta_l_right = ((-encoder_knee_right.PDOgetActualPosition()-ZERO_SENSOR_KNEE_right)*2*pi)/encoder_out;
		theta_c_right = ((servo_knee_right.PDOgetActualPosition()-ZERO_SERVO_KNEE_right)*2*pi)/(encoder_in*N);

    theta_l_left = ((-USB_1->positionRead(_eposID)-ZERO_SENSOR_KNEE_left)*2*pi)/encoder_out;

		theta_c_left = ((servo_knee_left.PDOgetActualPosition()-ZERO_SERVO_KNEE_left)*2*pi)/(encoder_in*N);

    theta_l_hip_right = -((encoderdata[0]-ZERO_SENSOR_HIP_right)*pi/32768);
    theta_c_hip_right = ((servo_hip_right.PDOgetActualPosition()-ZERO_SERVO_HIP_right)*2*pi)/(encoder_hip*N_hip);

    theta_l_hip_left = -((encoderdata[1]-ZERO_SENSOR_HIP_left)*pi/32768);
    theta_c_hip_left = ((servo_hip_left.PDOgetActualPosition()-ZERO_SERVO_HIP_left)*2*pi)/(encoder_hip*N_hip);


		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA knee right: %.4f graus", theta_l_right*(180/pi));
		printf("\n               POSICAO DA COROA knee right: %.4f graus", theta_c_right*(180/pi));
		printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA knee left: %.4f graus", theta_l_left*(180/pi));
		printf("\n               POSICAO DA COROA knee left: %.4f graus", theta_c_left*(180/pi));
		printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA hip right: %.4f graus", theta_l_hip_right*(180/pi));
		printf("\n               POSICAO DA COROA hip right: %.4f graus", theta_c_hip_right*(180/pi));
    printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA hip left: %.4f graus", theta_l_hip_left*(180/pi));
		printf("\n               POSICAO DA COROA hip left: %.4f graus", theta_c_hip_left*(180/pi));
		printf("\n*********************************************************************************");

    torque_l_right = ks_right*(theta_c_right - theta_l_right);
    
    torque_l_left = ks_left*(theta_c_left - theta_l_left);

    torque_l_hip_right = ks_hip_right*(theta_c_hip_right - theta_l_hip_right);

    torque_l_hip_left = ks_hip_left*(theta_c_hip_left - theta_l_hip_left);

		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque knee right: %.4f Nm", torque_l_right);
		printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque knee left: %.4f Nm", torque_l_left);
    printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
    printf("\n                    Torque hip right: %.4f Nm", torque_l_hip_right);
    printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque hip left: %.4f Nm", torque_l_hip_left);
		printf("\n*********************************************************************************");

		}while (total_time < 10);

   		total_time = 0;
	}

void leitura_sensores(int T_sensors2){


     while (!flag_arduino_multi_exo || !flag_arduino_multi_imu || !flag_arduino_multi_esp || !flag_arduino_multi_ard)
    {
       //std::cout<<"#";
      if (aborting_exo || aborting_imu || aborting_esp || aborting_ard)
        return;
    }

     define_origen();

     float fsr1 = 0.0f;
     float fsr2 = 0.0f;
     float enc1 = 0.0f;
     float enc2 = 0.0f;  

		//ZERA CONTADOR DE CICLOS
	 int total_time = 0;
		
		epos.sync();
		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
		epos.sync();

    ArduinoData fromArduino;
    ValuesAxia80M50 fromATIMX;
//////////////////////

		do{
            
      total_time += 1;

      mtx_readSensors.lock();
      fromArduino.enc1 = arduinoData.enc1;
      fromArduino.enc2 = arduinoData.enc2;
      fromArduino.fsr1 = arduinoData.fsr1;
      fromArduino.fsr2 = arduinoData.fsr2;
      mtx_readSensors.unlock();

      mtx_valuesAxia.lock();
            fromATIMX = valuesAxia;
      mtx_valuesAxia.unlock();

		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
		epos.sync();
	
		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");
    printf("\n********************************************************************************");
		encoder_knee_right.ReadPDO01();
    printf("\nPOSICAO CALIBRADA ENCODER knee right: %d ppr", (-encoder_knee_right.PDOgetActualPosition())); 
	  servo_knee_right.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR knee right: %d ppr", (servo_knee_right.PDOgetActualPosition()));
    printf("\n********************************************************************************");
    printf("\nPOSICAO CALIBRADA ENCODER knee left: %.2f ppr", (-USB_1->positionRead(_eposID))); 
	  servo_knee_left.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR knee left: %d ppr", (servo_knee_left.PDOgetActualPosition()));
    printf("\n********************************************************************************");
  
    printf("\nPOSICAO CALIBRADA ENCODER hip right: %f deg", -(fromArduino.enc1)); 
    
    servo_hip_right.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR hip right: %d ppr", (servo_hip_right.PDOgetActualPosition()));
 
    printf("\nPOSICAO CALIBRADA ENCODER hip left: %f deg", -(fromArduino.enc2)); 
    
    servo_hip_left.ReadPDO01();
    printf("\nPOSICAO CALIBRADA MOTOR hip left: %d ppr", (servo_hip_left.PDOgetActualPosition()));
    printf("\n\n********************************************************************************");


    printf("\nPOSIÇÃO DO ROBO");

		theta_l_right = ((-encoder_knee_right.PDOgetActualPosition()-ZERO_SENSOR_KNEE_right)*2*pi)/encoder_out;
		theta_c_right = ((servo_knee_right.PDOgetActualPosition()-ZERO_SERVO_KNEE_right)*2*pi)/(encoder_in*N);

    theta_l_left = ((-USB_1->positionRead(_eposID)-ZERO_SENSOR_KNEE_left)*2*pi)/encoder_out;

		theta_c_left = ((servo_knee_left.PDOgetActualPosition()-ZERO_SERVO_KNEE_left)*2*pi)/(encoder_in*N);


    theta_l_hip_right = -((fromArduino.enc1-ZERO_SENSOR_HIP_right)*pi/180.0f);
   
    theta_c_hip_right = ((servo_hip_right.PDOgetActualPosition()-ZERO_SERVO_HIP_right)*2*pi)/(encoder_hip*N_hip);

    theta_l_hip_left = -((fromArduino.enc2-ZERO_SENSOR_HIP_left)*pi/180.0f);
 
    theta_c_hip_left = ((servo_hip_left.PDOgetActualPosition()-ZERO_SERVO_HIP_left)*2*pi)/(encoder_hip*N_hip);


		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA knee right: %.4f graus", theta_l_right*(180/pi));
		printf("\n               POSICAO DA COROA knee right: %.4f graus", theta_c_right*(180/pi));
		printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA knee left: %.4f graus", theta_l_left*(180/pi));
		printf("\n               POSICAO DA COROA knee left: %.4f graus", theta_c_left*(180/pi));
		printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA hip right: %.4f graus", theta_l_hip_right*(180/pi));
		printf("\n               POSICAO DA COROA hip right: %.4f graus", theta_c_hip_right*(180/pi));
    printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA hip left: %.4f graus", theta_l_hip_left*(180/pi));
		printf("\n               POSICAO DA COROA hip left: %.4f graus", theta_c_hip_left*(180/pi));
		printf("\n*********************************************************************************");

    torque_l_right = ks_right*(theta_c_right - theta_l_right);
    
    torque_l_left = ks_left*(theta_c_left - theta_l_left);

    torque_l_hip_right = ks_hip_right*(theta_c_hip_right - theta_l_hip_right);

    torque_l_hip_left = ks_hip_left*(theta_c_hip_left - theta_l_hip_left);

		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque knee right: %.4f Nm", torque_l_right);
		printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque knee left: %.4f Nm", torque_l_left);
    printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
    printf("\n                    Torque hip right: %.4f Nm", torque_l_hip_right);
    printf("\n*********************************************************************************");
    printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque hip left: %.4f Nm", torque_l_hip_left);
		printf("\n*********************************************************************************");

    //fromATIMX
    printf("\n");
    printf("\t %f", valuesAxia.Fx );
    printf("\t %f", valuesAxia.Fy );
    printf("\t %f", valuesAxia.Fz );
    printf("\t %f", valuesAxia.Tx );
    printf("\t %f", valuesAxia.Ty );
    printf("\t %f", valuesAxia.Tz );
    printf("\n");

		}while (total_time < T_sensors2);

   		total_time = 0;
	}

 void leitura_arduino(int T_ard)
{
    // inicializar dataloggers
    //double datalog_ard[T_ard][n_datalogs_ard];

    vector< vector<double> > datalog_ard(T_ard, vector<double>(n_datalogs_ard));

    cout << " Inicializando datalogs ARDUINO... " << endl;

    int i_datalogs_ard = 0;
    int i_dt_ard = 0;
    for (i_datalogs_ard = 0; i_datalogs_ard < n_datalogs_ard; i_datalogs_ard++)
    {
        for (i_dt_ard = 0; i_dt_ard < T_ard; i_dt_ard++)
        {
            datalog_ard[i_dt_ard][i_datalogs_ard] = 0.0;
        }
    }
    i_dt_ard = 0;

    // Inicializar variables Arduino
    bool ARD_CONN = false;
	  char *ard_cod = (char *)"g";
    int ard_data_size = 1;
    int read_result = 0;
    clock_t tempo_sec_1;
    clock_t tempo_sec_2;
    char incomingData[MAX_DATA_LENGTH]; //String for incoming data
    ostringstream error_ard;

    float fsr1;
    float fsr2;
    float enc1;
    float enc2;

    ard_cod = "f";

	  if (ard_cod == "g")
		  ard_data_size = 1;
	  if (ard_cod == "t")
		  ard_data_size = 6;
	  if (ard_cod == "f")
		  ard_data_size = 4;
	  if (ard_cod == "p")
		  ard_data_size = 6;

    // { crear un objeto nuevo Arduino }
    SerialPort* arduino_loop;

    try
    {
      // { crear un objeto nuevo Arduino }
      arduino_loop = new SerialPort(port_name);

      cout << " Inicializando ARDUINO... " << endl;
          
      // { verificar si esta conectado este Arduino }
      if (!arduino_loop->isConnected()) 
      {
          error_ard << "Error: Check port name";
          throw runtime_error(error_ard.str());
      }

      cout << "Connection with Arduino: Established" << endl;
    
      // { verificar lectura del Arduino }
      arduino_loop->writeSerialPort( ard_cod, 1 );
      tempo_sec_1 = clock();
      do
      {
        read_result += arduino_loop->readSerialPort(incomingData + read_result, ard_data_size * sizeof(uint8_t ));
        tempo_sec_2 = clock();
        if (tempo_sec_2 - tempo_sec_1 > 20)
        {                      
          error_ard << "Error: Arduino connection lost";
          throw runtime_error(error_ard.str());
        }
      } while (read_result < ard_data_size * sizeof(uint8_t ));

    }
    catch (exception const &e)
    {
      cerr << e.what() << '\n';
      cout << "A fatal error has occured before Arduino recording. Aborting." << endl;
      cout << "****ABORT****" << endl;

      aborting_ard = true;

      esperar_n_seg(3);

      return;

    }

    ARD_CONN = true;
    flag_arduino_multi_ard = true;
    cout << " Arduino ready " << endl;

    // wait until other components are ready
    // **ojo con el acceso de los otros threads a estas variables**
    while (!flag_arduino_multi_exo || !flag_arduino_multi_imu || !flag_arduino_multi_esp || !flag_arduino_multi_atimx)
    {
       //std::cout<<"#";
      if (aborting_exo || aborting_imu || aborting_esp || aborting_atimx )
        return;
    }

    //Inicializa o loop
    total_time_ard = 0;

    //Cria os temporizadores (SAMPLE_TIME)
    loop_timers timer_ard(SAMPLE_TIME_ARD);

    // inicio sincronizacion
    arduino_loop->writeSerialPort( "h", 1 );
    
    try
    {
      // LOOP ARDUINO
      do
      {
          // Inicializa o temporizador de execucao do loop
          timer_ard.start_timer();

          // -------------------------------- //
          // Aqui o Codigo do loop

          if (!ARD_CONN)
          {
          // { si Arduino no conectado, conectar }
            cout << "No Arduino data" << endl;
          
            arduino_loop->closePort();
            arduino_loop = new SerialPort(port_name);
            if (arduino_loop->isConnected()) 
            { 
              cout << "Arduino Reconnected" << endl;
              ARD_CONN = 1;    
            }

          }
          else
          {
          // { Leitura Serial (Arduino) }

          // { enviar mensaje al Arduino para pedir datos}
            arduino_loop->writeSerialPort(ard_cod, 1);

          // { hacer lectura del Arduino }
            read_result = 0;
            tempo_sec_1 = clock();
            do
            {
              read_result += arduino_loop->readSerialPort(incomingData + read_result, ard_data_size * sizeof(float) );
              tempo_sec_2 = clock();
              if (tempo_sec_2 - tempo_sec_1 > 20)
              {
                cout << "Error: Arduino connection lost" << endl;
                ARD_CONN = 0;
                arduino_loop->closePort();
                break;
              }
            } while (read_result < ard_data_size * sizeof(float));

            float *arduinodata = (float *)incomingData;
          
            // asignar lecturas a variables
          
            if (ard_cod == "f")
            {
              fsr1 = arduinodata[0];
              fsr2 = arduinodata[1];
              enc1 = arduinodata[2];
              enc2 = arduinodata[3];

              mtx_readSensors.lock();
              arduinoData.fsr1 = fsr1;
              arduinoData.fsr2 = fsr2;
              arduinoData.enc1 = enc1;
              arduinoData.enc2 = enc2;
              mtx_readSensors.unlock();

//              printf("\n%10.10f %10.10f %10.10f %10.10f",fsr1 ,fsr2 ,enc1 ,enc2 );

            }

            /*
            if (ard_cod == "g")
              gait_phase = arduinodata[0];
            
            if (ard_cod == "t")
            {
              gait_phase = arduinodata[0];
              pof1 = arduinodata[1] * 1.00;
              pof2 = arduinodata[2] * 1.00;
              enc1 = arduinodata[3] * 1.00;
              enc2 = arduinodata[4] * 1.00;
              trigger_imus = arduinodata[5];
            }
            if (ard_cod == "p")
            {
              gait_phase = arduinodata[0];
              pof1 = arduinodata[1] * 1.00;
              pof2 = arduinodata[2] * 1.00;
              enc1 = arduinodata[3] * 1.00;
              enc2 = arduinodata[4] * 1.00;
              trigger_imus = arduinodata[5];
            }
            */

            // mostrar datos en pantalla si necesario
            /*
            cout << "Data Arduino: "
            cout << " | 1: " << gait_phase;
            cout << " | 2: " << pof1;
            cout << " | 3: " << pof2;
            cout << " | 4: " << enc1;
            cout << " | 5: " << enc2;
            cout << " | 6: " << trigger_imus;
            cout << endl;
            */

            /*
            std::cout << "Data Arduino: "
                      << " | 1: " << arduinodata[0]
                      << " | 2: " << arduinodata[1]
                      << " | 3: " << arduinodata[2]
                      << " | 4: " << arduinodata[3]
                      << " | 5: " << arduinodata[4]
                      << " | 6: " << arduinodata[5]
                      << std::endl;
                      */
            /*
            std::cout << "Data Arduino: "
                      << " | 1: " << arduinodata[0]
                      << " | 2: " << arduinodata[1]
                      << " | 3: " << arduinodata[2]
                      << " | 4: " << arduinodata[3]
                      << std::endl;
                      */
          #if verbose_m == 1
            std::cout << "Arduino OK" << std::endl;
          #endif

          }

          // Salvar dados em dataloggers
          datalog_ard[total_time_ard][0] = timer_ard.tempo2;
          datalog_ard[total_time_ard][1] = total_time_ard;
          datalog_ard[total_time_ard][2] = fsr1;
          datalog_ard[total_time_ard][3] = fsr2;
          datalog_ard[total_time_ard][4] = enc1;
          datalog_ard[total_time_ard][5] = enc2;

          //incrementa contador de tempo
          total_time_ard = total_time_ard + 1;

          // -------------------------------- //

          //AGUARDA FIM DE TEMPO DE EXECUCAO DE TAREFA
          timer_ard.wait_final_time();

          //cout << timer_ard.tempo2 << endl;

      } while (total_time_ard < T_ard);

    }
    catch (std::exception const &ex)
    {
        std::cout << ex.what() << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }

    // fechar arduino
    arduino_loop->writeSerialPort((char *)"z", 1);
    arduino_loop->closePort();

    // SALVA OS DATALOGGERS
    time_t ttt;
    time(&ttt);
    char fecha[50];
    struct tm *tmp = localtime(&ttt);
    strftime(fecha, sizeof(fecha), "datos/datos_arduino_%m%d%Y_%H%M%S.dat", tmp);

    FILE *pFile_ard = fopen(fecha, "w");
    FILE *pFile_ld_ard = fopen("datos/last_data_arduino.dat", "w");

    for (i_dt_ard = 0; i_dt_ard < T_ard; i_dt_ard++)
    {
        for (i_datalogs_ard = 0; i_datalogs_ard < n_datalogs_ard; i_datalogs_ard++)
        {
            // Salva no arquivo com data e hora
            fprintf(pFile_ard, "%.5f \t", datalog_ard[i_dt_ard][i_datalogs_ard]);
            // Salva no arquivo last data
            fprintf(pFile_ld_ard, "%.5f \t", datalog_ard[i_dt_ard][i_datalogs_ard]);
        }
        fprintf(pFile_ard, "\n");
        fprintf(pFile_ld_ard, "\n");
    }

    fclose(pFile_ard);
    fclose(pFile_ld_ard);

    // Zera contador
    total_time_ard = 0;
}

void controle_exo(int T_exo, int com_setpoint, int com_controller)
{
    
    // carregar setpoints
    
    float DADOS = 0.0; 
    int i = 0;
    

  	// setpoint de angulo
    setpoints_theta = new double[T_exo];
    //double *setpoints_theta = new double[T]; // intentar así

    FILE *pFile_theta = fopen("setpoints/setpoints_theta.dat", "r");
    for (i = 0; i < T_exo; i++)
    {
      fscanf(pFile_theta, "%f", &DADOS);
      setpoints_theta[i] = DADOS;
    }
    fclose(pFile_theta);

    // setpoint de ruido
    // setpoints_ruido = new double[T_exo];
    double *setpoints_ruido = new double[T_exo]; // intentar así

    DADOS = 0.0; 
    FILE *pFile_ruido = fopen("setpoints/setpoints_ruido_new.dat", "r");
    for (i = 0; i < T_exo; i++)
    {
      fscanf(pFile_ruido, "%f", &DADOS);
      setpoints_ruido[i] = DADOS;
    }
    fclose(pFile_ruido);
    
    double *setpoints_torque = new double[T_exo]; // intentar así

    FILE *pFile_torque = fopen("setpoints/setpoints_torque.dat", "r");

    for (i = 0; i < T_exo; i++)
    {
      fscanf(pFile_torque, "%f", &DADOS);
      setpoints_torque[i] = DADOS;
    }
    fclose(pFile_torque);


    double *setpoints = new double[T_exo]; // intentar así

    FILE *pFile_setpoints = fopen("setpoints/setpoints_ruido.dat", "r");;

    if (com_setpoint == 0)
      pFile_setpoints = fopen("setpoints/setpoints_ruido.dat", "r");
    
    if (com_setpoint == 1)
      pFile_setpoints = fopen("setpoints/setpoints_tria.dat", "r");
    
    if (com_setpoint == 2)
      pFile_setpoints = fopen("setpoints/setpoints_sine.dat", "r");
    
    if (com_setpoint == 3)
      pFile_setpoints = fopen("setpoints/setpoints_ramp.dat", "r");
    
    if (com_setpoint == 4)
      pFile_setpoints = fopen("setpoints/setpoints_step.dat", "r");
    
    if (com_setpoint == 5)
      pFile_setpoints = fopen("setpoints/setpoints_ruido_new.dat", "r");
    
    for (i = 0; i < T_exo; i++)
    {
      fscanf(pFile_setpoints, "%f", &DADOS);
      setpoints[i] = DADOS;
    }
    fclose(pFile_setpoints);

    // inicializar dataloggers
    //const int T_exo_const = T_exo;
    //double datalog_exo[T_exo_const][n_datalogs_exo];

    vector< vector<double> > datalog_exo(T_exo, vector<double>(n_datalogs_exo));

    cout << "Inicializando datalogs EXO..." << endl;

    int i_datalogs_exo = 0;
    int i_dt_exo = 0;
    for (i_datalogs_exo = 0; i_datalogs_exo < n_datalogs_exo; i_datalogs_exo++)
    {
        for (i_dt_exo = 0; i_dt_exo < T_exo; i_dt_exo++)
        {
            datalog_exo[i_dt_exo][i_datalogs_exo] = 0.0;
        }
    }
    i_dt_exo = 0;

    cout << "Inicializando controle ..." << endl;

    try
    {

        reset_falhas();  

        // Sincroniza as epos
        epos.sync();

        esperar_n_seg(1);

        // DEFININDO A POSICAO DE ORIGEM //
        define_origen();

        // Habilita o controle de velocidade
        servo_knee_right.VCS_SetOperationMode(VELOCITY_MODE);
        servo_knee_left.VCS_SetOperationMode(VELOCITY_MODE);
        servo_hip_left.VCS_SetOperationMode(VELOCITY_MODE);
        servo_hip_right.VCS_SetOperationMode(VELOCITY_MODE);

        encoder_knee_right.ReadPDO01();
        servo_knee_right.ReadPDO01();

        servo_knee_left.ReadPDO01();

        servo_hip_right.ReadPDO01();
        servo_hip_left.ReadPDO01();

        esperar_n_seg(2);

        // Habilitação do eixos
        Habilita_Eixo(2);

        cout << "... controle inicializado." << endl;

        flag_arduino_multi_exo = true;
        cout << " Robot ready " << endl;
    }
    catch (std::exception const &e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "A fatal error has occured during robot initialization. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;

        aborting_exo = true;

        esperar_n_seg(5);
        return;
    }

    double theta_ld_right = 0;
    double theta_ld_left = 0;
    double theta_ld_hip_right = 0;
    double theta_ld_hip_left = 0;
   
    double omega_ld_right = 0;
    double omega_ld_left = 0;
    double omega_ld_hip_right = 0;
    double omega_ld_hip_left = 0;
   
    double kv_right = 0;
    double bv_right = 0;
   
    double kv_left = 0;
    double bv_left = 0;

    double kv_hip_right = 0;
    double bv_hip_right = 0;

    double kv_hip_left = 0;
    double bv_hip_left = 0;

    double torque_d_right = 0;
    double torque_l_right = 0;
    double torque_d_left = 0;
    double torque_l_left = 0;

    double torque_d_hip_right = 0;
    double torque_l_hip_right = 0;

    double torque_d_hip_left = 0;
    double torque_l_hip_left = 0;

    double kp_right = 380;
    double ki_right = 35;
    double kd_right = 3;

    double kp_left = 200;
    double ki_left = 4;
    double kd_left = 0;

    double kp_hip_right = 120;
    double ki_hip_right = 1.2;
    double kd_hip_right = 0;

    double kp_hip_left = 120;
    double ki_hip_left = 1.2;
    double kd_hip_left = 0;

    double erro_0_right = 0;
    double erro_1_right = 0;
    double erro_2_right = 0;

    double erro_0_left = 0;
    double erro_1_left = 0;
    double erro_2_left = 0;

    double erro_0_hip_right = 0;
    double erro_1_hip_right = 0;
    double erro_2_hip_right = 0;

    double erro_0_hip_left = 0;
    double erro_1_hip_left = 0;
    double erro_2_hip_left = 0;

    double controle_right = 0; // entrada de controle
    double controle_ant_right = 0; // entrada de controle anterior

    double controle_left = 0; // entrada de controle
    double controle_ant_left = 0; // entrada de controle anterior
    
    double controle_hip_right = 0; // entrada de controle
    double controle_ant_hip_right = 0; // entrada de controle anterior

    double controle_hip_left = 0; // entrada de controle
    double controle_ant_hip_left = 0; // entrada de controle anterior

    double Ts = SAMPLE_TIME_EXO;

    double controle_final_right = 0; // entrada de controle definitivo
    double controle_final_left = 0; // entrada de controle definitivo

    double controle_final_hip_right = 0; // entrada de controle definitivo
    double controle_final_hip_left = 0; // entrada de controle definitivo
    
    // wait until other components are ready
    while (!flag_arduino_multi_ard || !flag_arduino_multi_imu || !flag_arduino_multi_esp)
   {
       //std::cout<<"#";
       if (aborting_ard || aborting_imu || aborting_esp)
          return;
  }
    
    //////////////////////////////

        //Habilita a porta serial (Arduino)
    
    SerialPort arduino(port_name);

      if (arduino.isConnected()) cout << "Connection Established" << endl;
      else cout << "ERROR, check port name";

  //////////////////////
 //     ArduinoData fromArduino;

    //Inicializa o loop
    total_time_exo = 0;

    //Cria os temporizadores (SAMPLE_TIME)
    loop_timers timer_exo(SAMPLE_TIME_EXO);



    try
    {
        // LOOP EXO
        do
        {

            // Leituras da arduino disponiveis para o controle
            // falta olhar se os dados da arduino são corretos
/*
            mtx_readSensors.lock();
            fromArduino.enc1 = arduinoData.enc1;
            fromArduino.enc2 = arduinoData.enc2;
            fromArduino.fsr1 = arduinoData.fsr1;
            fromArduino.fsr2 = arduinoData.fsr2;
            mtx_readSensors.unlock();
 */
            // Calcula o tempo de execucao do loop
            timer_exo.start_timer();

            // std::ostringstream error;
            // error << "Este es un error ficticio de prueba";
            // throw std::runtime_error(error.str());

            // -------------------------------- //
            // AQUI O CODIGO DO LOOP

           // Sincroniza a CAN
            epos.sync();

            // realizar leituras (Im, theta, omega, )
            servo_knee_right.ReadPDO01();
            Im_right = servo_knee_right.PDOgetActualCurrent();
            theta_c_right = ((servo_knee_right.PDOgetActualPosition() - ZERO_SERVO_KNEE_right) * 2 * pi) / (encoder_in * N);
            theta_m_right = ((servo_knee_right.PDOgetActualPosition() - ZERO_SERVO_KNEE_right) * 2 * pi) / (encoder_in);

            servo_knee_right.ReadPDO02();
            omega_m_right = servo_knee_right.PDOgetActualVelocity();

            encoder_knee_right.ReadPDO01();
            theta_l_right = ((-encoder_knee_right.PDOgetActualPosition() - ZERO_SENSOR_KNEE_right) * 2 * pi) / encoder_out;

            // realizar leituras (Im, theta, omega, )
            servo_knee_left.ReadPDO01();
            Im_left = servo_knee_left.PDOgetActualCurrent();
            theta_c_left = ((servo_knee_left.PDOgetActualPosition() - ZERO_SERVO_KNEE_left) * 2 * pi) / (encoder_in * N);
            theta_m_left = ((servo_knee_left.PDOgetActualPosition() - ZERO_SERVO_KNEE_left) * 2 * pi) / (encoder_in);

            servo_knee_left.ReadPDO02();
            omega_m_left = servo_knee_left.PDOgetActualVelocity();

            theta_l_left = ((-USB_1->positionRead(_eposID) - ZERO_SENSOR_KNEE_left) * 2 * pi) / encoder_out;
            
            // realizar leituras HIP (Im, theta, omega, )
            servo_hip_right.ReadPDO01();
            theta_c_hip_right = ((servo_hip_right.PDOgetActualPosition()-ZERO_SERVO_HIP_right)*2*pi)/(encoder_hip*N_hip);

            servo_hip_left.ReadPDO01();
            theta_c_hip_left = ((servo_hip_left.PDOgetActualPosition()-ZERO_SERVO_HIP_left)*2*pi)/(encoder_hip*N_hip);
        
            /////////////////////////

            // Leitura Serial (Arduino)
   
            arduino.writeSerialPort( "X", 1 );
            
            int read_result = 0;
      
            do {
            read_result += arduino.readSerialPort(incomingData + read_result, ENCODERS_NUMBER*sizeof(int16_t));
             } while(  read_result < ENCODERS_NUMBER*sizeof(int16_t) );
      
            int16_t *encoderdata = (int16_t*) incomingData;
  
            ////////////////////////

            theta_l_hip_right = -((encoderdata[0]-ZERO_SENSOR_HIP_right)*pi/32768);
            theta_l_hip_left = -((encoderdata[1]-ZERO_SENSOR_HIP_left)*pi/32768);


            //encoder_knee_right.ReadPDO02();
            //omega_l = -encoder_knee_right.PDOgetActualVelocity();

            /*
            omega_l = (theta_l - theta_l_ant) / Ts;
            omega_lf = -c2 * omega_lfant - c3 * omega_lfant2 + d1 * omega_l + d2 * omega_lant + d3 * omega_lant2;

            servo_knee_right.ReadPDO02();
            analog1 = servo_knee_right.PDOgetAnalogValue_01();
            analog2 = servo_knee_right.PDOgetAnalogValue_02(); // mV
            */

     //       controle_final_right = 0;
     //       controle_final_left = 0;
    //        controle_final_hip_right = 0;
     //       controle_final_hip_left = 0;
/*
            if (com_controller == 1 || com_controller == 3)
            {
                // -------------- CONTROLE DE IMPEDANCIA ----------------- //
                if (com_controller == 1)
                {
                    theta_ld = 0.3*setpoints[total_time_exo];
                    omega_ld = 0;

                    if(OPC_K == 0){
                      kv =0;
                      bv =0;
                    }

                    if(OPC_K == 1){
                      kv =30;
                      bv =0;
                    }

                    if(OPC_K == 2){
                      kv =60;
                      bv =2;
                    }

                    if(OPC_K == 3){
                      if(total_time_exo < T_exo/2){
                        kv =0;
                        bv =2;
                      }else{
                        kv =0;
                        bv =4;
                      }
                    }

  */                  

                    //kv =60;
                    //bv =2;

                    //
                    
                /*   Para teste damping                    
                if(total_time_exo > T_exo/2){
                kv =0;
                bv =0;
                    }
                   
                }
                 */
/*                if (com_controller == 3)
                {
                    theta_ld = 0*setpoints[total_time_exo];
                    omega_ld = 0;
                
                    kv = 0;
                    bv = 0;
                }
 */               

               kv_right = 0;
               bv_right = 0;
               torque_d_right = kv_right * (theta_ld_right - theta_l_right) + bv_right * (omega_ld_right - omega_l_right);

               kv_left = 0;
               bv_left = 0;
               torque_d_left = kv_left * (theta_ld_left - theta_l_left) + bv_left * (omega_ld_left - omega_l_left);

               kv_hip_right = 0;
               bv_hip_right = 0;
               torque_d_hip_right = kv_hip_right * (theta_ld_hip_right - theta_l_hip_right) + bv_hip_right * (omega_ld_hip_right);

               kv_hip_left = 0;
               bv_hip_left = 0;
               torque_d_hip_left = kv_hip_left * (theta_ld_hip_left - theta_l_hip_left) + bv_hip_left * (omega_ld_hip_left);

                // ---------------- CONTROLE DE TORQUE ------------------- //
                torque_l_right = ks_right * (theta_c_right - theta_l_right);

                torque_l_left = ks_left * (theta_c_left - theta_l_left);

                torque_l_hip_right = ks_hip_right * (theta_c_hip_right - theta_l_hip_right);

                torque_l_hip_left = ks_hip_left * (theta_c_hip_left - theta_l_hip_left);
                
                erro_0_right = (torque_d_right - torque_l_right);
                erro_0_left = (torque_d_left - torque_l_left);

                erro_0_hip_right = (torque_d_hip_right - torque_l_hip_right);
                erro_0_hip_left = (torque_d_hip_left - torque_l_hip_left);

                // Control para  1 ms //
                //kp = 380;
               // ki = 35;
                //kd = 3;
                // ------------------//

                // Controle P
                //controle = kp*(erro_0);

                // Controle PI
                //controle = controle_ant + kp*(erro_0 - erro_1) + ki*Ts*erro_0;

                // Controle PID
               controle_right = controle_ant_right + kp_right * (erro_0_right - erro_1_right) + ki_right * Ts * erro_0_right + (kd_right / Ts) * (erro_0_right - 2 * erro_1_right + erro_2_right);

                // Controle PID
               controle_left = controle_ant_left + kp_left * (erro_0_left - erro_1_left) + ki_left * Ts * erro_0_left + (kd_left / Ts) * (erro_0_left - 2 * erro_1_left + erro_2_left);

                // Controle PID
               controle_hip_right = controle_ant_hip_right + kp_hip_right * (erro_0_hip_right - erro_1_hip_right) + ki_hip_right * Ts * erro_0_hip_right + (kd_hip_right / Ts) * (erro_0_hip_right - 2 * erro_1_hip_right + erro_2_hip_right);

                // Controle PID
                controle_hip_left = controle_ant_hip_left + kp_hip_left * (erro_0_hip_left - erro_1_hip_left) + ki_hip_left * Ts * erro_0_hip_left + (kd_hip_left / Ts) * (erro_0_hip_left - 2 * erro_1_hip_left + erro_2_hip_left);

                /*
                // ---------------- CONTROLE DE VELOCIDAD ------------------- //
                if (com_controller == 1)
                {
                  controle_final = controle;
                }

                if (com_controller == 3)
                {
                  controle_final = controle + 200*setpoints_ruido[total_time_exo];
                }
            }

            if (com_controller == 2)
            {
                // ---------------- CONTROLE DE TORQUE ------------------- //
                torque_d = 3*setpoints[total_time_exo];
                // torque_lf = filtrar(torque_l)
            
                torque_l = ks * (theta_c - theta_l);
            
                erro_0 = (torque_d - torque_l);

                // Control para  1 ms //
                kp = 380;
                ki = 35;
                kd = 3;
                            // ------------------//

                // Controle P
                //controle = kp*(erro_0);

                // Controle PI
                //controle = controle_ant + kp*(erro_0 - erro_1) + ki*Ts*erro_0;

                // Controle PID
                controle = controle_ant + kp * (erro_0 - erro_1) + ki * Ts * erro_0 + (kd / Ts) * (erro_0 - 2 * erro_1 + erro_2);

                // ------------------------------------------------------- //

                //--------------------------------------------------------//
                controle_final = controle;
            }
            */
		        //--------------------------------------------------------//

            // enviar o valor de controle ao motor
            servo_knee_right.PDOsetVelocitySetpoint(int(controle_right));
            servo_knee_right.WritePDO02();

            // enviar o valor de controle ao motor
            servo_knee_left.PDOsetVelocitySetpoint(int(controle_left));
            servo_knee_left.WritePDO02();

            // enviar o valor de controle ao motor
            servo_hip_right.PDOsetVelocitySetpoint(int(controle_hip_right));
            servo_hip_right.WritePDO02();

            // enviar o valor de controle ao motor
            servo_hip_left.PDOsetVelocitySetpoint(int(controle_hip_left));
            servo_hip_left.WritePDO02();

            // atualizar registros (ej. erro_ant = erro)
            // {...}
            erro_2_right = erro_1_right;
            erro_1_right = erro_0_right;

            controle_ant_right = controle_right;

            erro_2_left = erro_1_left;
            erro_1_left = erro_0_left;

            controle_ant_left = controle_left;

            erro_2_hip_right = erro_1_hip_right;
            erro_1_hip_right = erro_0_hip_right;

            controle_ant_hip_right = controle_hip_right;

            erro_2_hip_left = erro_1_hip_left;
            erro_1_hip_left = erro_0_hip_left;

            controle_ant_hip_left = controle_hip_left;

            // Mostrar na tela dados
            // {...}
            /*
            std::cout << "Data exo: "
                      << " | Wm: " << controle
                      << " | Tl: " << torque_l
                      << " | Ol: " << theta_l
                      << std::endl;
                      */

            // Salvar dados em dataloggers
            datalog_exo[total_time_exo][0] = timer_exo.tempo2;
            datalog_exo[total_time_exo][1] = total_time_exo;
            datalog_exo[total_time_exo][2] = controle_right;
            datalog_exo[total_time_exo][3] = torque_l_right;
            datalog_exo[total_time_exo][4] = torque_d_right;
            datalog_exo[total_time_exo][5] = theta_l_right;
            datalog_exo[total_time_exo][6] = theta_ld_right;
           
            datalog_exo[total_time_exo][7] = controle_left;
            datalog_exo[total_time_exo][8] = torque_l_left;
            datalog_exo[total_time_exo][9] = torque_d_left;
            datalog_exo[total_time_exo][10] = theta_l_left;
            datalog_exo[total_time_exo][11] = theta_ld_left;

            datalog_exo[total_time_exo][12] = controle_hip_right;
            datalog_exo[total_time_exo][13] = torque_l_hip_right;
            datalog_exo[total_time_exo][14] = torque_d_hip_right;
            datalog_exo[total_time_exo][15] = theta_l_hip_right;
            datalog_exo[total_time_exo][16] = theta_ld_hip_right;
           
            datalog_exo[total_time_exo][17] = controle_hip_left;
            datalog_exo[total_time_exo][18] = torque_l_hip_left;
            datalog_exo[total_time_exo][19] = torque_d_hip_left;
            datalog_exo[total_time_exo][20] = theta_l_hip_left;
            datalog_exo[total_time_exo][21] = theta_ld_hip_left;

            //incrementa contador de tempo
            total_time_exo = total_time_exo + 1;

            // -------------------------------- //

            // cout << timer_exo.tempo2 << endl;

            #if verbose_m == 1
            std::cout << "Exo OK" << std::endl;
            #endif
  
            //AGUARDA FIM DE TEMPO DE EXECUCAO DE TAREFA
            timer_exo.wait_final_time();


        } while (total_time_exo < T_exo);

        // Zera o comando do motor
        servo_knee_right.PDOsetVelocitySetpoint(0);
        servo_knee_right.WritePDO02();

        servo_knee_left.PDOsetVelocitySetpoint(0);
        servo_knee_left.WritePDO02();

        servo_hip_right.PDOsetVelocitySetpoint(0);
        servo_hip_right.WritePDO02();

        servo_hip_left.PDOsetVelocitySetpoint(0);
        servo_hip_left.WritePDO02();

    }
    catch (std::exception const &e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "A fatal error has occured during robot operation. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;

        // Zera o comando do motor
        servo_knee_right.PDOsetVelocitySetpoint(0);
        servo_knee_right.WritePDO02();

        servo_knee_left.PDOsetVelocitySetpoint(0);
        servo_knee_left.WritePDO02();

        servo_hip_right.PDOsetVelocitySetpoint(0);
        servo_hip_right.WritePDO02();

        servo_hip_left.PDOsetVelocitySetpoint(0);
        servo_hip_left.WritePDO02();

        T_exo = total_time_exo;
        esperar_n_seg(5);
    }

    // Desabilita o eixo
    Desabilita_Eixo(0);

    // SALVA OS DATALOGGERS
    time_t ttt;
    time(&ttt);
    char fecha[50];
    struct tm *tmp = localtime(&ttt);
    strftime(fecha, sizeof(fecha), "datos/datos_robot_%m%d%Y_%H%M%S.dat", tmp);

    FILE *pFile_exo = fopen(fecha, "w");
    FILE *pFile_ld_exo = fopen("datos/last_data_robot.dat", "w");

    for (i_dt_exo = 0; i_dt_exo < T_exo; i_dt_exo++)
    {
        for (i_datalogs_exo = 0; i_datalogs_exo < n_datalogs_exo; i_datalogs_exo++)
        {
            // Salva no arquivo com data e hora
            fprintf(pFile_exo, "%.5f \t", datalog_exo[i_dt_exo][i_datalogs_exo]);
            // Salva no arquivo last data
            fprintf(pFile_ld_exo, "%.5f \t", datalog_exo[i_dt_exo][i_datalogs_exo]);
        }
        fprintf(pFile_exo, "\n");
        fprintf(pFile_ld_exo, "\n");
    }

    fclose(pFile_exo);
    fclose(pFile_ld_exo);

    // Zera contador
    total_time_exo = 0;
}

void leitura_xsens(int T_imu)
{
    // inicializar dataloggers
    vector< vector<double> > datalog_imu(T_imu, vector<double>(n_datalogs_imu));

    std::cout << " Inicializando datalogs IMUs XSens... " << endl;

    int i_datalogs_imu = 0;
    int i_dt_imu = 0;
    for (i_datalogs_imu = 0; i_datalogs_imu < n_datalogs_imu; i_datalogs_imu++)
    {
        for (i_dt_imu = 0; i_dt_imu < T_imu; i_dt_imu++)
        {
            datalog_imu[i_dt_imu][i_datalogs_imu] = 0.0;
        }
    }
    i_dt_imu = 0;

    // --------- //

    int nm = 0;
    vector<int> imu_headers(4);

    // --------- //

    const int desiredUpdateRate = 100;  // Use 75 Hz update rate for MTWs
    const int desiredRadioChannel = 19; // Use radio channel 19 for wireless master.

    WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
    std::vector<MtwCallback *> mtwCallbacks;       // Callbacks for mtw devices

    std::cout << "Constructing XsControl..." << std::endl;
    XsControl *control = XsControl::construct();
    if (control == 0)
    {
        std::cout << "Failed to construct XsControl instance." << std::endl;
    }

    // --------- //

    try
    {

        // --------- //

        std::cout << "Scanning ports..." << std::endl;
        XsPortInfoArray detectedDevices = XsScanner::scanPorts();

        std::cout << "Finding wireless master..." << std::endl;
        XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
        while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
        {
            ++wirelessMasterPort;
        }
        if (wirelessMasterPort == detectedDevices.end())
        {
            throw std::runtime_error("No wireless masters found");
        }
        std::cout << "Wireless master found @ " << *wirelessMasterPort << std::endl;

        std::cout << "Opening port..." << std::endl;
        if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
        {
            std::ostringstream error;
            error << "Failed to open port " << *wirelessMasterPort;
            throw std::runtime_error(error.str());
        }

        std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
        XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
        if (wirelessMasterDevice == 0)
        {
            std::ostringstream error;
            error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
            throw std::runtime_error(error.str());
        }

        std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

        std::cout << "Setting config mode..." << std::endl;
        if (!wirelessMasterDevice->gotoConfig())
        {
            std::ostringstream error;
            error << "Failed to goto config mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        std::cout << "Attaching callback handler..." << std::endl;
        wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

        std::cout << "Getting the list of the supported update rates..." << std::endl;
        const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

        std::cout << "Supported update rates: ";
        for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
        {
            std::cout << *itUpRate << " ";
        }
        std::cout << std::endl;

        const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

        std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
        if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
        {
            std::ostringstream error;
            error << "Failed to set update rate: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        std::cout << "Disabling radio channel if previously enabled..." << std::endl;
        if (wirelessMasterDevice->isRadioEnabled())
        {
            if (!wirelessMasterDevice->disableRadio())
            {
                std::ostringstream error;
                error << "Failed to disable radio channel: " << *wirelessMasterDevice;
                throw std::runtime_error(error.str());
            }
        }

        std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
        if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
        {
            std::ostringstream error;
            error << "Failed to set radio channel: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        std::cout << "Waiting for MTW to wirelessly connect...\n"
                  << std::endl;

        bool waitForConnections = true;
        size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
        do
        {
            XsTime::msleep(100);

            while (true)
            {
                size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
                if (nextCount != connectedMTWCount)
                {
                    std::cout << "Number of connected MTWs: " << nextCount << ". Press 'Y' to start measurement." << std::endl;
                    connectedMTWCount = nextCount;
                }
                else
                {
                    break;
                }
            }
            if (_kbhit())
            {
                waitForConnections = (toupper((char)_getch()) != 'Y');
            }
        } while (waitForConnections);

        // --------- //

        // Start measurement
        std::cout << std::endl;

        std::cout << "Starting measurement..." << std::endl;
        if (!wirelessMasterDevice->gotoMeasurement())
        {
            std::ostringstream error;
            error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
        XsDeviceIdArray allDeviceIds = control->deviceIds();
        XsDeviceIdArray mtwDeviceIds;
        for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
        {
            if (i->isMtw())
            {
                mtwDeviceIds.push_back(*i);
            }
        }
        XsDevicePtrArray mtwDevices;
        for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
        {
            XsDevicePtr mtwDevice = control->device(*i);
            if (mtwDevice != 0)
            {
                mtwDevices.push_back(mtwDevice);
                cout<<"\nBattery level " + mtwDevice->batteryLevel();
            }
            else
            {
                throw std::runtime_error("Failed to create an MTW XsDevice instance");
            }
        }

        std::cout << "Attaching callback handlers to MTWs..." << std::endl;
        mtwCallbacks.resize(mtwDevices.size());
        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
            mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
        }

        flag_arduino_multi_imu = true;
        std::cout << " XSens ready " << endl;

        // wait until other components are ready
        while (!flag_arduino_multi_ard || !flag_arduino_multi_exo || !flag_arduino_multi_esp)
        {
          //std::cout<<"#";
          if (aborting_ard || aborting_exo || aborting_esp)
            return;
        }

        //Inicializa o loop
        total_time_imu = 0;

        //Cria os temporizadores (SAMPLE_TIME)
        loop_timers timer_imu(SAMPLE_TIME_IMU);

        // --------- //

        // Measurement loop
        std::cout << "\nMain loop. Press any key to quit\n"
                  << std::endl;
        std::cout << "Waiting for data available..." << std::endl;

        nm = mtwCallbacks.size();

        // --------- //        
        std::vector<XsEuler> eulerData(mtwCallbacks.size()); // Vector to store euler data for each mtw
        std::vector<XsVector> accData(mtwCallbacks.size());  // Vector to store acc data for each mtw
        std::vector<XsVector> gyroData(mtwCallbacks.size()); // Vector to store gyro data for each mtw
        std::vector<XsVector> magData(mtwCallbacks.size()); // Vector to store magnetic data for each mtw
        // --------- //

        imu_headers.clear();
        for (size_t i = 0; i < mtwCallbacks.size(); ++i)
        {
          imu_headers.push_back(mtwCallbacks[i]->device().deviceId().toInt());
        }

        /*
        IDs IMUs:
        1: 11801311
        2: 11800786
        3: 11801156
        4: 11800716
        */

        // LOOP IMU
        do
        {
            // Inicializa o temporizador de execucao do loop
            timer_imu.start_timer();

            // -------------------------------- //
            // Aqui o Codigo do loop

            // Leitura IMU
            bool newDataAvailable = false;
            
            for (size_t i = 0; i < nm; ++i)
            {
                if (mtwCallbacks[i]->dataAvailable())
                {
                    newDataAvailable = true;
                    XsDataPacket const *packet = mtwCallbacks[i]->getOldestPacket();
                    
                    eulerData[i] = packet->orientationEuler();
                    accData[i]   = packet->calibratedAcceleration();
                    gyroData[i]  = packet->calibratedGyroscopeData();
                    magData[i]   = packet->calibratedMagneticField();
                    
                    mtwCallbacks[i]->deleteOldestPacket();

                    // mostrar datos en pantalla si necesario
                    if (newDataAvailable)
                    {
                        
                        datalog_imu[total_time_imu][2+0+0*3+i*12] = gyroData[i].value(0); // Wix
                        datalog_imu[total_time_imu][2+1+0*3+i*12] = gyroData[i].value(1); // Wiy
                        datalog_imu[total_time_imu][2+2+0*3+i*12] = gyroData[i].value(2); // Wiz
                                                              
                        datalog_imu[total_time_imu][2+0+1*3+i*12] = accData[i].value(0); // Aix
                        datalog_imu[total_time_imu][2+1+1*3+i*12] = accData[i].value(1); // Aiy
                        datalog_imu[total_time_imu][2+2+1*3+i*12] = accData[i].value(2); // Aiz
                                                              
                        datalog_imu[total_time_imu][2+0+2*3+i*12] = magData[i].value(0); // Mix
                        datalog_imu[total_time_imu][2+1+2*3+i*12] = magData[i].value(1); // Miy
                        datalog_imu[total_time_imu][2+2+2*3+i*12] = magData[i].value(2); // Miz
                                                              
                        datalog_imu[total_time_imu][2+0+3*3+i*12] = eulerData[i].roll();  
                        datalog_imu[total_time_imu][2+1+3*3+i*12] = eulerData[i].pitch(); 
                        datalog_imu[total_time_imu][2+2+3*3+i*12] = eulerData[i].yaw();

                        #if verbose_m == 1
                          cout << "XSens " << i << " OK" << endl;
                        #endif
                        

                    }
                }
            }

            

            
            // Salvar dados em dataloggers
            datalog_imu[total_time_imu][0] = timer_imu.tempo2;
            datalog_imu[total_time_imu][1] = total_time_imu;
            

            //incrementa contador de tempo
            total_time_imu = total_time_imu + 1;

            // -------------------------------- //
            #if verbose_m == 1
                std::cout << timer_imu.tempo2 << endl;
            #endif
            //AGUARDA FIM DE TEMPO DE EXECUCAO DE TAREFA
            timer_imu.wait_final_time();


        } while (total_time_imu < T_imu);
        //} while ((total_time_imu < T) && (!_kbhit()));
        //(void)_getch();

        // --------- //

        std::cout << "Setting config mode..." << std::endl;
        if (!wirelessMasterDevice->gotoConfig())
        {
            std::ostringstream error;
            error << "Failed to goto config mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        std::cout << "Disabling radio... " << std::endl;
        if (!wirelessMasterDevice->disableRadio())
        {
            std::ostringstream error;
            error << "Failed to disable radio: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        // --------- //


    }
    catch (std::exception const &ex)
    {
        aborting_imu = true;
        std::cout << ex.what() << std::endl;
        std::cout << "****ABORT****" << std::endl;

        esperar_n_seg(15);
    }
    catch (...)
    {
        aborting_imu = true;
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;

       esperar_n_seg(15);
    }

    // --------- //

    std::cout << "Closing XsControl..." << std::endl;
    control->close();

    std::cout << "Deleting mtw callbacks..." << std::endl;
    for (std::vector<MtwCallback *>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
    {
        delete (*i);
    }

        
    std::cout << "Successful exit." << std::endl;
    //std::cout << "Press [ENTER] to continue." << std::endl;
    //std::cin.get();

    // --------- //
    
    /// INCLUIR FUNCION PARA SALVAR DATALOGGERS
  
    time_t ttt;
    time(&ttt);
    char fecha[50];
    struct tm *tmp = localtime(&ttt);
    strftime(fecha, sizeof(fecha), "datos/datos_imu_xsens_%m%d%Y_%H%M%S.dat", tmp);

    FILE *pFile_imu = fopen(fecha, "w");
    FILE *pFile_ld_imu = fopen("datos/last_data_xsens.dat", "w");
    
    for (size_t i = 0; i < nm; ++i)
    {
        int temp = imu_headers.at(i);
        fprintf(pFile_imu, "%d \t", temp);
        fprintf(pFile_ld_imu, "%d \t", temp);
    }

    for (i_datalogs_imu = nm; i_datalogs_imu < n_datalogs_imu; i_datalogs_imu++)
    {
      fprintf(pFile_imu, "%d \t", 0);
      fprintf(pFile_ld_imu, "%d \t", 0);
    }
    fprintf(pFile_imu, "\n");
    fprintf(pFile_ld_imu, "\n");
    
    for (i_dt_imu = 0; i_dt_imu < T_imu-1; i_dt_imu++)
    {
        for (i_datalogs_imu = 0; i_datalogs_imu < n_datalogs_imu; i_datalogs_imu++)
        {
            // Salva no arquivo com data e hora
            fprintf(pFile_imu, "%.5f \t", datalog_imu[i_dt_imu][i_datalogs_imu]);
            // Salva no arquivo last data
            fprintf(pFile_ld_imu, "%.5f \t", datalog_imu[i_dt_imu][i_datalogs_imu]);
        }
        fprintf(pFile_imu, "\n");
        fprintf(pFile_ld_imu, "\n");
    }
    
    fclose(pFile_imu);
    
    fclose(pFile_ld_imu);
    
    // Zera contador
    total_time_imu = 0;
    
}

void leitura_emg(int T_emg)
{

  // { crear una struct para datos EMG nueva }
  // structEMG_vars* EMG_vars;


   

}

void leitura_esp32(int T_esp)
{
    // inicializar dataloggers
    //double datalog_ard[T_ard][n_datalogs_ard];

    vector< vector<double> > datalog_esp(T_esp, vector<double>(n_datalogs_esp));

    cout << " Inicializando datalogs ESP32... " << endl;

    int i_datalogs_esp = 0;
    int i_dt_esp = 0;
    for (i_datalogs_esp = 0; i_datalogs_esp < n_datalogs_esp; i_datalogs_esp++)
    {
        for (i_dt_esp = 0; i_dt_esp < T_esp; i_dt_esp++)
        {
            datalog_esp[i_dt_esp][i_datalogs_esp] = 0.0;
        }
    }
    i_dt_esp = 0;

    // Inicializar variables Arduino
    bool ESP_CONN = false;
	  char *esp_cod = (char *)"g";
    int esp_data_size = 1;
    int read_result = 0;
    clock_t tempo_sec_1;
    clock_t tempo_sec_2;
    char incomingData[MAX_DATA_LENGTH]; //String for incoming data
    ostringstream error_esp;

    float pof1;
    float pof2;
    float pof3;
    float pof4;

    esp_cod = "f";

	  if (esp_cod == "g")
		  esp_data_size = 1;
	  if (esp_cod == "t")
		  esp_data_size = 6;
	  if (esp_cod == "f")
		  esp_data_size = 4;
	  if (esp_cod == "p")
		  esp_data_size = 6;

    // { crear un objeto nuevo Arduino }
    SerialPort* esp32_loop;

    try
    {
      // { crear un objeto nuevo Arduino }
      esp32_loop = new SerialPort(port_name_esp);

      cout << " Inicializando ESP32... " << endl;
          
      // { verificar si esta conectado este Arduino }
      if (!esp32_loop->isConnected()) 
      {
          error_esp << "Error: Check port name";
          throw runtime_error(error_esp.str());
      }

      cout << "Connection with ESP32: Established" << endl;
    
      // { verificar lectura del Arduino }
      esp32_loop->writeSerialPort( esp_cod, 1 );
      tempo_sec_1 = clock();
      do
      {
        read_result += esp32_loop->readSerialPort(incomingData + read_result, esp_data_size * sizeof(float) );
        tempo_sec_2 = clock();
        if (tempo_sec_2 - tempo_sec_1 > 20)
        {                      
          error_esp << "Error: ESP32 connection lost";
          throw runtime_error(error_esp.str());
        }
      } while (read_result < esp_data_size * sizeof(float));

    }
    catch (exception const &e)
    {
      cerr << e.what() << '\n';
      cout << "A fatal error has occured before ESP32 recording. Aborting." << endl;
      cout << "****ABORT****" << endl;

      aborting_esp = true;

      esperar_n_seg(3);

      return;

    }

    ESP_CONN = true;
    flag_arduino_multi_esp = true;
    cout << " ESP32 ready " << endl;

    // wait until other components are ready
    // **ojo con el acceso de los otros threads a estas variables**
    while (!flag_arduino_multi_ard || !flag_arduino_multi_exo || !flag_arduino_multi_imu)
    {
      //std::cout<<"#";
      if (aborting_ard || aborting_exo || aborting_imu)
        return;
    }

    
    //Inicializa o loop
    total_time_esp = 0;

    //Cria os temporizadores (SAMPLE_TIME)
    loop_timers timer_esp(SAMPLE_TIME_ESP);

    // inicio sincronizacion
    //esp32_loop->writeSerialPort( "o", 1 );
    
    try 
    {
      // LOOP ARDUINO
      do
      {
      
        // Inicializa o temporizador de execucao do loop
        timer_esp.start_timer();
        
        // -------------------------------- //
        // Aqui o Codigo do loop
        
        if (!ESP_CONN)
        {
        // { si Arduino no conectado, conectar }
          cout << "No ESP32 data T:" << timer_esp.tempo2 << endl;
          
          esp32_loop->closePort();
          esp32_loop = new SerialPort(port_name_esp);
          if (esp32_loop->isConnected()) 
          { 
            cout << "ESP32 Reconnected T: "<< timer_esp.tempo2 << endl ;
            
            ESP_CONN = true;    
          }
          
        }
        else
        {
        // { Leitura Serial (Arduino) }

        // { enviar mensaje al Arduino para pedir datos}
          esp32_loop->writeSerialPort(esp_cod, 1);
          
        // { hacer lectura del Arduino }
          read_result = 0;
          tempo_sec_1 = clock();
          do
          {
            read_result += esp32_loop->readSerialPort(incomingData + read_result, esp_data_size * sizeof(float) );
            tempo_sec_2 = clock();
            if (tempo_sec_2 - tempo_sec_1 > 20)
            {                      
              error_esp << "Error: ESP32 connection lost";
              //throw runtime_error(error_esp.str());
              
              ESP_CONN = false;
              esp32_loop ->closePort();
              break;
            }
          } while (read_result < esp_data_size * sizeof(float));
          
          float *esp32data = (float  *)incomingData;
                    
          // asignar lecturas a variables
          
          if (esp_cod == "f")
          {
            pof1 = esp32data[0];
            pof2 = esp32data[1];
            pof3 = esp32data[2];
            pof4 = esp32data[3];
          }
          
          /*
          // mostrar datos en pantalla si necesario
          std::cout << "Data Arduino: "
                    << " | 1: " << esp32data[0]
                    << " | 2: " << esp32data[1]
                    << " | 3: " << esp32data[2]
                    << " | 4: " << esp32data[3]
                    << std::endl;
                    */
          /*
          std::cout << "Data POF: "
                    << " | 2: " << esp32data[1]
                    << std::endl;
                    */
      #if verbose_m == 1
          cout << "ESP32 OK" << endl;
      #endif

        }

        // Salvar dados em dataloggers
        datalog_esp[total_time_esp][0] = timer_esp.tempo2;
        datalog_esp[total_time_esp][1] = total_time_esp;
        datalog_esp[total_time_esp][2] = pof1;
        datalog_esp[total_time_esp][3] = pof2;
        datalog_esp[total_time_esp][4] = pof3;
        datalog_esp[total_time_esp][5] = pof4;
        
        //incrementa contador de tempo
        total_time_esp = total_time_esp + 1;

        pof1 = 0;
        pof2 = 0;
        pof3 = 0;
        pof4 = 0;

        // -------------------------------- //

        //AGUARDA FIM DE TEMPO DE EXECUCAO DE TAREFA
        timer_esp.wait_final_time();
        
        //cout << timer_esp.tempo2 << endl;

      } while (total_time_esp < T_esp);

    }
    catch (std::exception const &ex)
    {
        std::cout << ex.what() << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }

    // fechar arduino
    esp32_loop->closePort();

    // SALVA OS DATALOGGERS
    time_t ttt;
    time(&ttt);
    char fecha[50];
    struct tm *tmp = localtime(&ttt);
    strftime(fecha, sizeof(fecha), "datos/datos_esp32_%m%d%Y_%H%M%S.dat", tmp);

    FILE *pFile_esp = fopen(fecha, "w");
    FILE *pFile_ld_esp = fopen("datos/last_data_esp32.dat", "w");

    for (i_dt_esp = 0; i_dt_esp < T_esp-1; i_dt_esp++)
    {
        for (i_datalogs_esp = 0; i_datalogs_esp < n_datalogs_esp; i_datalogs_esp++)
        {
            // Salva no arquivo com data e hora
            fprintf(pFile_esp, "%.5f \t", datalog_esp[i_dt_esp][i_datalogs_esp]);
            // Salva no arquivo last data
            fprintf(pFile_ld_esp, "%.5f \t", datalog_esp[i_dt_esp][i_datalogs_esp]);
        }
        fprintf(pFile_esp, "\n");
        fprintf(pFile_ld_esp, "\n");
    }

    fclose(pFile_esp);
    fclose(pFile_ld_esp);

    // Zera contador
    total_time_esp = 0;

}



 void leitura_atimx(int T_ard)
{

    aborting_atimx = false;
    cout << " Inicializando fio ATI-MX... " << endl;

    flag_arduino_multi_atimx = true;

    vector< vector<double> > datalog_atimx(T_ard, vector<double>(n_datalogs_atimx));

    cout << " Inicializando datalogs ATIMX... " << endl;

    int i_datalogs_atimx = 0;
    int i_dt_atimx = 0;
    for (i_datalogs_atimx = 0; i_datalogs_atimx < n_datalogs_atimx; i_datalogs_atimx++)
    {
        for (i_dt_atimx = 0; i_dt_atimx < T_ard; i_dt_atimx++)
        {
            datalog_atimx[i_dt_atimx][i_datalogs_atimx] = 0.0;
        }
    }
    i_dt_atimx = 0;

    // iniciazlizar sensor
    Axia80M50 * sensor = new Axia80M50(IP_ATIMX);
    aborting_atimx = !sensor->init();
    if(aborting_atimx) {
      printf("\n\n ##ERROR##  ABORTING ATIMX, is not connected . . . \n\n");
      return;
    }
		
    sensor->bias();

    while (!flag_arduino_multi_exo || !flag_arduino_multi_imu || !flag_arduino_multi_ard)
    {
      if (aborting_exo || aborting_imu || aborting_ard)
        return;
    }
   //Inicializa o loop
    int total_time_atimx = 0;

    loop_timers timer_atimx(SAMPLE_TIME_ATIMX);

  std::ostringstream text;

  try
    {
      // LOOP ARDUINO
      do
      {

          timer_atimx.start_timer();

          // -------------------------------- //
          // Aqui o Codigo do loop


            sensor->peek();

            
            mtx_valuesAxia.lock();
            valuesAxia  = sensor->values;
            mtx_valuesAxia.unlock();
				 

            datalog_atimx[total_time_atimx][0] = valuesAxia.Fx;
            datalog_atimx[total_time_atimx][1] = valuesAxia.Fy;
            datalog_atimx[total_time_atimx][2] = valuesAxia.Fz;
            datalog_atimx[total_time_atimx][3] = valuesAxia.Tx;
            datalog_atimx[total_time_atimx][4] = valuesAxia.Ty;
            datalog_atimx[total_time_atimx][5] = valuesAxia.Tz;

          total_time_atimx = total_time_atimx + 1;

          // -------------------------------- //

          //AGUARDA FIM DE TEMPO DE EXECUCAO DE TAREFA
          timer_atimx.wait_final_time();
      } while (total_time_atimx < T_ard);

    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }


    time_t ttt;
    time(&ttt);
    char fecha[50];
    struct tm *tmp = localtime(&ttt);
    strftime(fecha, sizeof(fecha), "datos/datos_atimx_%m%d%Y_%H%M%S.dat", tmp);

    FILE *pFile_atimx = fopen(fecha, "w");
    FILE *pFile_ld_atimx = fopen("datos/last_data_atimx.dat", "w");

    for (i_dt_atimx = 0; i_dt_atimx < T_ard-1; i_dt_atimx++)
    {
        for (i_datalogs_atimx = 0; i_datalogs_atimx < n_datalogs_esp; i_datalogs_atimx++)
        {
            // Salva no arquivo com data e hora
            fprintf(pFile_atimx, "%.5f \t", datalog_atimx[i_dt_atimx][i_datalogs_atimx]);
            // Salva no arquivo last data
            fprintf(pFile_ld_atimx, "%.5f \t", datalog_atimx[i_dt_atimx][i_datalogs_atimx]);
        }
        fprintf(pFile_atimx, "\n");
        fprintf(pFile_ld_atimx, "\n");
    }

    fclose(pFile_atimx);
    fclose(pFile_ld_atimx);


    total_time_atimx = 0;
}



 void leitura_template(int T_ard)
{


    cout << " Inicializando fio ATI-MX... " << endl;

    flag_arduino_multi_atimx = true;

    // iniciazlizar sensor

    while (!flag_arduino_multi_exo || !flag_arduino_multi_imu || !flag_arduino_multi_esp)
    {
      if (aborting_exo || aborting_imu || aborting_esp)
        return;
    }
   //Inicializa o loop
    int total_time_atimx = 0;

    loop_timers timer_atimx(SAMPLE_TIME_ARD);

    
    try
    {
      // LOOP ARDUINO
      do
      {

          timer_atimx.start_timer();

          // -------------------------------- //
          // Aqui o Codigo do loop



          total_time_atimx = total_time_atimx + 1;

          // -------------------------------- //

          //AGUARDA FIM DE TEMPO DE EXECUCAO DE TAREFA
          timer_atimx.wait_final_time();
      } while (total_time_atimx < T_ard);

    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }

    total_time_atimx = 0;
}


/*
Controle do Exo Tau do Laboratório de Reabilitação Robótica da EESC-USP (2012 - 2022)
Criador por Juan Carlos Perez Ibarra (25/Oct/2020)
Contribuições: Maurício Félix Escalante, Jose Yecid, Wiliam Santos, Jonatan C. Jaimes, Leonardo Felipe dos Santos, Ícaro Ostan...
*/

#define ITK_NOEXCEPT noexcept

#ifdef _WIN32
#include <WinSock2.h>
#endif

#define verbose_m 0

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <condition_variable>

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
//---------------------------------------//

#include "declarations.h"
#include "declarations_epos.h"
#include "declarations_xsens.h"
#include "declarations_emgs.h"

//---------------------------------------//
// Headers for IMU-based angle estimation (qASGD)
//---------------------------------------//
#include "qASGD_KF.h"

using namespace std;
using namespace Eigen;

vector<float> imus_data(18);
condition_variable imus_condvar;
mutex imu_mtx;
int NumberOfImus;
qASGDKF estimador_angulo(samples_per_second_imu);


void print_cabecalho(char *titulo);
void esperar_n_seg(int sec);
void start_transmissao_rede_epos();
void init_comm_eixos();
void Habilita_Eixo(int ID);
void Desabilita_Eixo(int ID);
void reset_falhas();
void define_origen();
void inicializa_controle();

void leitura_arduino(int T_ard);
void controle_exo(int T_exo, int com_setpoint, int com_controller);
void leitura_xsens(int T_imu);
void leitura_emg(int T_emg);
void leitura_esp32(int T_esp);

int salva_dataloggers(double (*datalog_ard)[n_datalogs_ard], size_t rows);

float accBasedControl();


int main()
{
    //INICIALIZACAO DE TELA
    print_cabecalho("");

    //START DE TRANSMISS�O DA REDE
    cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << endl;
    start_transmissao_rede_epos();
    init_comm_eixos();
    

    //LOOP DE CONTROLE DE COMANDOS
    do
    {
        // MENU PRINCIPAL

        print_cabecalho("");
        cout << ")SELECCIONA UMA OPCAO:" << endl;
        cout << endl;
        cout << "[0] - ENCERRAR PROGRAMA" << endl;
        cout << "[1] - DEFINIR POSIÇÃO DE ORIGEM" << endl;
        cout << "[2] - RESET DE FALHAS" << endl;
        cout << "[3] - LEITURA DADOS ESP 32" << endl;
        cout << "[4] - LEITURA DADOS EMG" << endl;
        cout << "[5] - LEITURA DADOS ARDUINO" << endl;
        cout << "[6] - LEITURA DADOS IMUs XSens" << endl;
        cout << "[7] - CONTROLE EXO" << endl;
        cout << "[8] - CONTROLE + SENSORES" << endl;
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

            continue;
        }

        if (COMMAND_KEY == 2)
        {
            print_cabecalho("RESET DE FALHAS - EPOS CANOpen");

            reset_falhas();
            
            continue;
        }

        int arr[] = {3, 4, 5, 6, 7, 8, 9};
        std::vector<int> vect(arr, arr + 7);
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
                cout << "[4] - ACC - TRANSPARENCIA" << endl;
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
                flag_arduino_multi_esp = true;
                flag_arduino_multi_exo = false;
                flag_arduino_multi_imu = false;

                printf("\nflag_arduino_multi_ard : %d",flag_arduino_multi_ard );
                printf("\nflag_arduino_multi_esp : %d",flag_arduino_multi_esp );
                printf("\nflag_arduino_multi_exo : %d",flag_arduino_multi_exo );
                printf("\nflag_arduino_multi_imu : %d",flag_arduino_multi_imu );
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
        std::thread thr_qasgd;

        aborting_ard = false;
        aborting_esp = false;

         // Funcion de leitura das XSens
        if (!flag_arduino_multi_imu)
        {
            thr_imuxsens  = std::thread(leitura_xsens, exec_time * samples_per_second_imu);
            thr_qasgd     = std::thread(&qASGDKF::updateJointStates, &estimador_angulo, std::ref(imus_condvar), std::ref(imu_mtx));
        }

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



        cout << "ARD: " << flag_arduino_multi_ard << endl;
        cout << "ESP: " << flag_arduino_multi_esp << endl;
        cout << "EXO: " << flag_arduino_multi_exo << endl;
        cout << "IMU: " << flag_arduino_multi_imu << endl;
        cout << "EMG: " << flag_arduino_multi_emg << endl;

        if (thr_arduino.joinable())
            thr_arduino.join();

        if (thr_esp32.joinable())
            thr_esp32.join();

        if (thr_control.joinable())
            thr_control.join();

        if (thr_emgdelsys.joinable())
            thr_emgdelsys.join();

        if (thr_imuxsens.joinable())
        {
            thr_imuxsens.join();
            thr_qasgd.join();
        }

    } while (COMMAND_KEY != 0);

    //FINALIZA A COMUNICA��O COM AS EPOS
    epos.StopPDOS(1);

    estimador_angulo.~qASGDKF();

    esperar_n_seg(1);
    cout << "FIM DE PROGRAMA" << endl;
    esperar_n_seg(1);

    system(CLC);
}

void print_cabecalho(char *titulo)
{
    //system(CLC);

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
        eixo_out.ReadPDO01();
        eixo_in.ReadPDO01();

        printf(".");
    }
}

void Habilita_Eixo(int ID)
{

    if ((ID == 2) | (ID == 0))
    {
        eixo_in.PDOsetControlWord_SwitchOn(false);
        eixo_in.PDOsetControlWord_EnableVoltage(true);
        eixo_in.PDOsetControlWord_QuickStop(true);
        eixo_in.PDOsetControlWord_EnableOperation(false);
        eixo_in.WritePDO01();

        printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");

        endwait = clock() + 0.5 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        eixo_in.PDOsetControlWord_SwitchOn(true);
        eixo_in.PDOsetControlWord_EnableVoltage(true);
        eixo_in.PDOsetControlWord_QuickStop(true);
        eixo_in.PDOsetControlWord_EnableOperation(false);
        eixo_in.WritePDO01();

        endwait = clock() + 0.5 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        eixo_in.PDOsetControlWord_SwitchOn(true);
        eixo_in.PDOsetControlWord_EnableVoltage(true);
        eixo_in.PDOsetControlWord_QuickStop(true);
        eixo_in.PDOsetControlWord_EnableOperation(true);
        eixo_in.WritePDO01();
    }
}

void Desabilita_Eixo(int ID)
{

    if ((ID == 2) | (ID == 0))
    {
        printf("\nDESABILITANDO O MOTOR E CONTROLE");

        eixo_in.PDOsetControlWord_SwitchOn(true);
        eixo_in.PDOsetControlWord_EnableVoltage(true);
        eixo_in.PDOsetControlWord_QuickStop(true);
        eixo_in.PDOsetControlWord_EnableOperation(false);
        eixo_in.WritePDO01();

        endwait = clock() + 0.5 * CLOCKS_PER_SEC;
        while (clock() < endwait)
        {
        }

        eixo_in.PDOsetControlWord_SwitchOn(false);
        eixo_in.PDOsetControlWord_EnableVoltage(true);
        eixo_in.PDOsetControlWord_QuickStop(true);
        eixo_in.PDOsetControlWord_EnableOperation(false);
        eixo_in.WritePDO01();
    }
}

void reset_falhas()
{
    //EPOS 01
    eixo_out.PDOsetControlWord_FaultReset(true);
    eixo_out.WritePDO01();

    printf("\nResetando as falhas.");

    esperar_n_seg(1);

    printf("..");

    //EPOS 01
    eixo_out.PDOsetControlWord_FaultReset(false);
    eixo_in.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");

    //EPOS 02
    eixo_in.PDOsetControlWord_FaultReset(true);
    eixo_in.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("..");

    //EPOS 02
    eixo_in.PDOsetControlWord_FaultReset(false);
    eixo_in.WritePDO01();

    printf("..");

    esperar_n_seg(1);

    printf("OK");
}

void define_origen()
{
    epos.sync();

    printf("Definindo Origem... ");

    esperar_n_seg(1);

    printf("...");

    eixo_out.ReadPDO01();
    ZERO_01 = -eixo_out.PDOgetActualPosition();

    eixo_in.ReadPDO01();
    ZERO_02 = eixo_in.PDOgetActualPosition();

    printf(" ... origem definida.\n");
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
    while (!flag_arduino_multi_exo || !flag_arduino_multi_imu || !flag_arduino_multi_esp)
    {
       //std::cout<<"#";
      if (aborting_exo || aborting_imu || aborting_esp)
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
        eixo_in.VCS_SetOperationMode(VELOCITY_MODE);

        eixo_out.ReadPDO01();
        eixo_in.ReadPDO01();

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

    double theta_ld = 0;
    double omega_ld = 0;
    double kv = 0;
    double bv = 0;
    double torque_d = 0;
    double torque_l = 0;
    double kp = 380;
    double ki = 35;
    double kd = 3;
    double erro_0 = 0;
    double erro_1 = 0;
    double erro_2 = 0;
    double controle = 0; // entrada de controle
    double controle_ant = 0; // entrada de controle anterior
    double Ts = SAMPLE_TIME_EXO;

    double controle_final = 0; // entrada de controle definitivo

    int print_counter_4 = 0;
    
    
    // wait until other components are ready
    while (!flag_arduino_multi_ard || !flag_arduino_multi_imu || !flag_arduino_multi_esp)
    {
       //std::cout<<"#";
        if (aborting_ard || aborting_imu || aborting_esp)
          return;
    }
    
    //Inicializa o loop
    total_time_exo = 0;

    //Cria os temporizadores (SAMPLE_TIME)
    loop_timers timer_exo(SAMPLE_TIME_EXO);


    try
    {
        do
        {
            // LOOP EXO
            // Calcula o tempo de execucao do loop
            timer_exo.start_timer();

            // -------------------------------- //
            // AQUI O CODIGO DO LOOP

            // Sincroniza a CAN
            epos.sync();

            // realizar leituras (Im, theta, omega, )
            eixo_in.ReadPDO01();
            Im = eixo_in.PDOgetActualCurrent();
            theta_c = ((eixo_in.PDOgetActualPosition() - ZERO_02) * 2 * pi) / (encoder_in * N);
            theta_m = ((eixo_in.PDOgetActualPosition() - ZERO_02) * 2 * pi) / (encoder_in);

            eixo_in.ReadPDO02();
            omega_m = eixo_in.PDOgetActualVelocity();

            eixo_out.ReadPDO01();
            theta_l = ((-eixo_out.PDOgetActualPosition() - ZERO_01) * 2 * pi) / encoder_out;

            controle_final = 0;

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

                    

                    //kv =60;
                    //bv =2;

                    //
                    
                /*   Para teste damping                    
                if(total_time_exo > T_exo/2){
                kv =0;
                bv =0;
                    }
                    */
                }

                if (com_controller == 3)
                {
                    theta_ld = 0*setpoints[total_time_exo];
                    omega_ld = 0;
                
                    kv = 0;
                    bv = 0;
                }
                
                torque_d = kv * (theta_ld - theta_l) + bv * (omega_ld - omega_l);

                // ---------------- CONTROLE DE TORQUE ------------------- //
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

            if (com_controller == 4 && NumberOfImus == 3)
            {

              
              /// fprintf para teste..
              FILE* test_file;
              test_file = fopen("controle4_test.txt","w");
              if (test_file != NULL){
                fclose(test_file); 
              }

              print_counter_4++;
              if (!(print_counter_4 % 200)){
                unique_lock<mutex> Lk(imu_mtx); // esta dentro de um {}, OK
                imus_condvar.notify_one();
                test_file = fopen("controle4_test.txt", "a");
                if (test_file != NULL){
                  fprintf(test_file, "GyroZ IMU1: %.4f \n", imus_data[2]);
                  fclose(test_file);
                }
                print_counter_4 = 0;
              }
              
              controle_final = 0;
            }

            if (com_controller == 4 && NumberOfImus != 3)
            {
              controle_final = 0;
              throw std::runtime_error("Conecte 3 IMUs!");
            }

		        //--------------------------------------------------------//

            // enviar o valor de controle ao motor
            eixo_in.PDOsetVelocitySetpoint(int(controle_final));
            eixo_in.WritePDO02();

            // atualizar registros (ej. erro_ant = erro)
            // {...}
            erro_2 = erro_1;
            erro_1 = erro_0;

            controle_ant = controle;


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
            datalog_exo[total_time_exo][2] = controle;
            datalog_exo[total_time_exo][3] = torque_l;
            datalog_exo[total_time_exo][4] = torque_d;
            datalog_exo[total_time_exo][5] = theta_l;
            datalog_exo[total_time_exo][6] = theta_ld;

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
        eixo_in.PDOsetVelocitySetpoint(0);
        eixo_in.WritePDO02();
    }
    catch (std::exception const &e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "A fatal error has occured during robot operation. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;

        // Zera o comando do motor
        eixo_in.PDOsetVelocitySetpoint(0);
        eixo_in.WritePDO02();

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
    try{
   
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

        NumberOfImus = (int)mtwDevices.size();

        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
          if (i == 0){
            std::cout << "IMU na coxa da pessoa: " << mtwDevices[i]->deviceId().toString().toStdString() << "\n";
          }
          if (i == 1){
            std::cout << "IMU na canela da pessoa: "<< mtwDevices[i]->deviceId().toString().toStdString() << "\n";
          }
          if (i == 2){
            std::cout << "IMU na canela do exoesqueleto: "<< mtwDevices[i]->deviceId().toString().toStdString() << "\n";
          }
        }

        std::this_thread::sleep_for(std::chrono::seconds(5)); // talvez nao seja necessario

        flag_arduino_multi_imu = true;
        std::cout << " XSens ready " << endl;

        // wait until other components are ready
        while (!flag_arduino_multi_ard || !flag_arduino_multi_exo || !flag_arduino_multi_esp)
        {
          //std::cout<<"#";

          if(!flag_arduino_multi_ard && aborting_ard ) return;

          if(!flag_arduino_multi_exo && aborting_exo ) return;

          if(!flag_arduino_multi_esp && aborting_esp ) return;

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

                        unique_lock<mutex> Lck(imu_mtx); // dentro do {}, OK
                        imus_data[6*i+0] = gyroData[i].value(0);
                        imus_data[6*i+1] = gyroData[i].value(1);
                        imus_data[6*i+2] = gyroData[i].value(2);
                        imus_data[6*i+3] = accData[i].value(0);
                        imus_data[6*i+4] = accData[i].value(1);
                        imus_data[6*i+5] = accData[i].value(2);

                        if (i == 0){
                          Vector3f gyro( gyroData[i].toVector() ); 
                          Vector3f acc( accData[i].toVector() );
                          estimador_angulo.fetchIMUUpperLeg(gyro, acc);
                        }
                        if (i == 1){
                          Vector3f gyro( gyroData[i].toVector() ); 
                          Vector3f acc( accData[i].toVector() );
                          estimador_angulo.fetchIMULowerLeg(gyro, acc);
                        }
                        imus_condvar.notify_all();
                        imus_condvar.wait(Lck);

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

     }catch(...){
       printf("\n\n\nHubo un error\n\n\n");
    }
    
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


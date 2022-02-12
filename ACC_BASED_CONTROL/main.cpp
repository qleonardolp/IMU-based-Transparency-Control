//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "XsensEpos.h"
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include <processthreadsapi.h>
#include <stdexcept>
#include <iostream>
#include <conio.h>

void Interface();
void readIMUs(ThrdStruct &data_struct);
void readFTSensor(ThrdStruct &data_struct);
void qASGD(ThrdStruct &data_struct);
void Controle(ThrdStruct &data_struct);
void Logging(ThrdStruct &data_struct);
void updateGains(ThrdStruct &data_struct);

// DEBUGGING DEFINES:
#define PRIORITY     0
#define ISREADY_WAIT 0
#define EXEC_TIME   50

// Threads Sample Time:
#define IMU_SMPLTM  0.0100 // "@100 Hz", actually is defined by Xsens 'desiredUpdateRate'
#define ASGD_SMPLTM 0.0050 //  @200  Hz
#define CTRL_SMPLTM 0.0010 //  @1000 Hz
#define LOG_SMPLTM  0.0050 //  @200  Hz 
#define GSCN_SMPLTM 4.0000 //  @ leitura de ganhos do arquivo a cada 4s 
#define FT_SMPLTM   0.0010 //  @1000 Hz, pode chegar a 7kHz ... 
// Threads Priority:
#define IMU_PRIORITY       -1 //
#define ASGD_PRIORITY      -1 //
#define CTRL_PRIORITY      -2 //
#define LOG_PRIORITY        0 //
#define DEFAULT_PRIORITY    0 // 

// Global vars shared with Interface() funct
ThrdStruct imu_struct, asgd_struct, ftsensor_struct;
ThrdStruct control_struct, logging_struct, gscan_struct;
short imu_isready(false);
short asgd_isready(false);
short control_isready(false);
short logging_isready(false);
short ftsensor_isready(false);
short gscan_isready(false);

bool execution_end(false);
int  execution_time = EXEC_TIME;

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

  mutex comm_mtx;
  float imu_data[18];
  float gains_data[18];
  float logging_data[10];
  float states_data[10];
  float ati_data[6];
  //initialize data vectors (safety):
  for (int i = 0; i < sizeof(imu_data)/sizeof(float); i++){
    imu_data[i] = gains_data[i] = 0;
    if (i < 10) logging_data[i] = states_data[i] = 0;
    if (i < 6) ati_data[i] = 0;
  }
  
  // Function Structs definition:
  { // Apenas para colapsar e facilitar leitura do código

  // IMU Struct:
  imu_struct.sampletime_ = IMU_SMPLTM;
  imu_struct.param00_  = IMU_PRIORITY;
  imu_struct.param0A_  = &imu_isready;
  *(imu_struct.datavec_) = imu_data;
  imu_struct.mtx_ = &comm_mtx;

  // F/T Sensor Struct
  ftsensor_struct.sampletime_ = FT_SMPLTM;
  ftsensor_struct.param00_ = DEFAULT_PRIORITY;
  ftsensor_struct.param0E_ = &ftsensor_isready;
  *(ftsensor_struct.datavecF_) = ati_data;
  ftsensor_struct.mtx_ = &comm_mtx;

  // qASGD Struct:
  asgd_struct.sampletime_ = ASGD_SMPLTM;
  asgd_struct.param00_  = ASGD_PRIORITY;
  asgd_struct.param0B_  = &asgd_isready;
  asgd_struct.param0A_  = &imu_isready;
  *(asgd_struct.datavec_) = imu_data;
  *(asgd_struct.datavecB_) = states_data;
  asgd_struct.mtx_ = &comm_mtx;

  // Control Struct
  control_struct.sampletime_ = CTRL_SMPLTM;
  control_struct.param00_  = CTRL_PRIORITY;
  control_struct.param0C_  = &control_isready;
  control_struct.param0A_  = &imu_isready;
  control_struct.param0B_  = &asgd_isready;
  control_struct.param0D_  = &logging_isready;
  control_struct.param0E_  = &ftsensor_isready;
  *(control_struct.datavec_ ) = gains_data;
  *(control_struct.datavecA_) = logging_data;
  *(control_struct.datavecB_) = states_data;
  *(control_struct.datavecF_) = ati_data;
  control_struct.mtx_ = &comm_mtx;

  // Logging Struct
  logging_struct.sampletime_ = LOG_SMPLTM;
  logging_struct.param00_  = LOG_PRIORITY;
  logging_struct.param0D_  = &logging_isready;
  logging_struct.param0A_  = &imu_isready;
  logging_struct.param0B_  = &asgd_isready;
  logging_struct.param0C_  = &control_isready;
  logging_struct.param0E_  = &ftsensor_isready;
  *(logging_struct.datavec_ ) = gains_data;
  *(logging_struct.datavecA_) = logging_data;
  *(logging_struct.datavecF_) = ati_data;
  logging_struct.mtx_ = &comm_mtx;

  // updateGains Struct
  gscan_struct.sampletime_ = GSCN_SMPLTM;
  gscan_struct.param00_ = DEFAULT_PRIORITY;
  gscan_struct.param0F_ = &gscan_isready;
  gscan_struct.param0D_ = &logging_isready;
  gscan_struct.param0C_ = &control_isready;
  *(gscan_struct.datavec_ ) = gains_data;
  gscan_struct.mtx_ = &comm_mtx;
  }
  // Threads declaration
  thread thr_imus;
  thread thr_qasgd;
  thread thr_controle;
  thread thr_logging;
  thread thr_ftsensor;
  thread thr_gainscan;

  do
  {
    system("cls");
    Interface();
    // Fire Threads:
    if (!imu_isready)      thr_imus = thread(readIMUs, imu_struct);
    if (!asgd_isready)     thr_qasgd = thread(qASGD, asgd_struct);
    if (!logging_isready)  thr_logging = thread(Logging, logging_struct);
    if (!control_isready)  thr_controle = thread(Controle, control_struct);
    if (!ftsensor_isready) thr_ftsensor = thread(readFTSensor, ftsensor_struct);
    if (!gscan_isready)    thr_gainscan = thread(updateGains, gscan_struct);

    // main waits while the threads execute thier tasks...
    if (thr_controle.joinable()) thr_controle.join();
    if (thr_logging.joinable()) thr_logging.join();
    if (thr_ftsensor.joinable()) thr_ftsensor.join();
    if (thr_gainscan.joinable()) thr_gainscan.join();
    if (thr_qasgd.joinable()) thr_qasgd.join();
    if (thr_imus.joinable()) thr_imus.join();

  } while (!execution_end);

  epos.StopPDOS(1);
  cout << "Successful exit." << endl;
  this_thread::sleep_for(chrono::milliseconds(700));
  return 0;
}

// Funcao de interface com o usuario:
void Interface()
{
  short option = 0;
  using namespace std;

  cout << "//////////////////////////////////////////\\/////////\\/"   << endl;
  cout << "// INTERFACE DE CONTROLE EXO-TAU  /       /\\     ////\\"   << endl;
  cout << "// EESC-USP                      / _____ ___  ___  //|"     << endl;
  cout << "// RehabLab                     /  | |  | . \\/   \\  /|"   << endl;
  cout << "// *Copyright 2021-2026* \\//// //  | |   \\ \\   |_|  /|"  << endl;
  cout << "//\\///////////////////////\\// //// \\_'_/\\_`_/__|   ///" << endl;
  cout << "///\\///////////////////////\\ //////////////////\\/////\\" << endl;

  cout << " Escolha uma opcao: \n";
  cout << " [01]: Controle com IMUs + F/T \n";
  cout << " [02]: Controle com IMUs \n";
  cout << " [03]: Controle sem IMUs \n";
  cout << " [04]: Leitura IMUs \n";
  cout << " [05]: Leitura F/T  \n";
  cout << " Ou zero (0) para finalizar. \n ";
  cin >> option;

  switch (option)
  {  
  case 1:
    // tudo roda!
    break;
  case 2:
    ftsensor_isready = true;
    break;
  case 3:
    imu_isready = true;
    asgd_isready = true;
    ftsensor_isready = true;
    break;
  case 4:
    control_isready  = true;
    ftsensor_isready = true;
    gscan_isready = true;
    break;
  case 5:
    control_isready  = true;
    gscan_isready = true;
    imu_isready = true;
    break;
  case 0:
  default:
    imu_isready   = true;
    asgd_isready  = true;
    gscan_isready = true;
    logging_isready  = true;
    control_isready  = true;
    ftsensor_isready = true;
    execution_end = true;
    break;
  }

  if (!execution_end) {
    cout << "\n Defina o tempo de execucao em segundos: ";
    cin >> execution_time;
    imu_struct.exectime_  = execution_time;
    asgd_struct.exectime_ = execution_time;
    gscan_struct.exectime_ = execution_time;
    control_struct.exectime_ = execution_time;
    logging_struct.exectime_ = execution_time;
    ftsensor_struct.exectime_ = execution_time;

    // Escolher tipo de controle:
    system("cls");
    if (!control_isready) {
      cout << " Escolha uma controlador: \n";
      cout << " [01]: Acc-based Transparency (PD) \n";
      cout << " [02]: Acc-based Transparency Z(0) \n";
      cout << " [02]: Acc-based Transparency (LS) \n";
      cout << " [04]: Int-based Transparency (F/T)\n ";
      cin >> option;
      control_struct.param01_ = option;
      logging_struct.param10_ = option; // to logging the control option
    }
    cout << " Iniciando Threads..." << endl;
  } 
}


//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\
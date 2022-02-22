//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "SharedStructs.h"    // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#if CAN_ENABLE
#include "Axia80M50.h"        // Vem primeiro para nao conflitar com <windows.h>
#endif
#include "QpcLoopTimer.h"     // ja inclui <windows.h>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <chrono>
//#include <math.h>


void readFTSensor(ThrdStruct &data_struct){
  using namespace std;
#if PRIORITY
  SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

  // Setup stuff:
  char* atiSensorIp = "192.168.1.1";
  float sensor_data[6]; // Fx Fy Fz Tx Ty Tz
  float aux_data[10];
  float cutoff_freq = 16;
  bool  enable_filt = data_struct.param01_;

  // inicializa sensor F/T:
#if CAN_ENABLE
  Axia80M50* sensorAxia = new Axia80M50(atiSensorIp);
  bool sensor_init = sensorAxia->init();
#else
  bool sensor_init = true;
#endif
  { 
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0E_ = sensor_init;   // INVESTIGAR PQ RETORNA TRUE MSM COM SENSOR DESCONECTADO
    *data_struct.param1E_ = !sensor_init; // ftsensor abort flag
    if (sensor_init) cout << " Sensor ATI F/T iniciado! \n";
    else cout <<  " Falha ao iniciar Sensor ATI F/T !!! \n";
  }

  if(sensor_init)
  {

    bool isready_imu(false);
    do{ 
      {   // confere IMU:
        unique_lock<mutex> _(*data_struct.mtx_);
        isready_imu = *data_struct.param0A_;
      } 
    } while (!isready_imu);

#if CAN_ENABLE
    sensorAxia->bias();
    this_thread::sleep_for(chrono::milliseconds(500));
#endif

    // Filtros Passa Baixa para os dados:
    LowPassFilter2pFloat sensor_filters[6];
    for (int i = 0; i < sizeof(sensor_filters)/sizeof(LowPassFilter2pFloat); i++)
    {
      float thread_frequency = 1/(data_struct.sampletime_); // Hz !!!
      sensor_filters[i].set_cutoff_frequency(thread_frequency, cutoff_freq);
      sensor_filters[i].reset();
    }

    looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
    // inicializa looptimer
    Timer.start();
    do
    {
      Timer.tik();
#if CAN_ENABLE
      sensorAxia->peek();

      if (!enable_filt){
        sensor_data[0] = sensorAxia->values.Fx;
        sensor_data[1] = sensorAxia->values.Fy;
        sensor_data[2] = sensorAxia->values.Fz;
        sensor_data[3] = sensorAxia->values.Tx;
        sensor_data[4] = sensorAxia->values.Ty;
        sensor_data[5] = sensorAxia->values.Tz;
      } else {
        sensor_data[0] = sensor_filters[0].apply(sensorAxia->values.Fx);
        sensor_data[1] = sensor_filters[1].apply(sensorAxia->values.Fy);
        sensor_data[2] = sensor_filters[2].apply(sensorAxia->values.Fz);
        sensor_data[3] = sensor_filters[3].apply(sensorAxia->values.Tx);
        sensor_data[4] = sensor_filters[4].apply(sensorAxia->values.Ty);
        sensor_data[5] = sensor_filters[5].apply(sensorAxia->values.Tz);
      }
#else
      sensor_data[0] = 1.23;
      sensor_data[1] = 1.23;
      sensor_data[2] = 1.23;
      sensor_data[3] = 1.23;
      sensor_data[4] = 1.23;
      sensor_data[5] = 1.23;
#endif

      {   // sessao critica
        unique_lock<mutex> _(*data_struct.mtx_);
        *(*data_struct.datavecB_ + 7) = -sensor_data[0]; // "inter_rgtshank  = vector[7];"
        memcpy(*data_struct.datavecF_, sensor_data, sizeof(sensor_data)); // para log
      }   // fim da sessao critica

      Timer.tak();
    } while (!Timer.end());

    {   
      unique_lock<mutex> _(*data_struct.mtx_);
      *data_struct.param0E_ = false;
    }
  }
}
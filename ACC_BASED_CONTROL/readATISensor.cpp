//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\


#include "Axia80M50.h"        // Vem primeiro para nao conflitar com <windows.h>
#include "QpcLoopTimer.h"     // ja inclui <windows.h>
#include "SharedStructs.h"    // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
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

  // setup stuff...
  char* atiSensorIp = "192.168.1.1";
  float sensor_data[6]; // Fx Fy Fz Tx Ty Tz
  float aux_data[10];

  // inicializa sensor F/T:
  Axia80M50* sensorAxia = new Axia80M50(atiSensorIp);
  bool sensor_init = sensorAxia->init();
  { 
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0E_ = sensor_init;
    if (sensor_init) cout << "Sensor ATI F/T iniciado! \n";
  }

  bool isready_imu(false);
  do{ 
    {   // confere IMU:
      unique_lock<mutex> _(*data_struct.mtx_);
      isready_imu = *data_struct.param0A_;
    } 
  } while (!isready_imu);

  sensorAxia->bias();
  this_thread::sleep_for(chrono::milliseconds(500));

  looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
  // inicializa looptimer
  Timer.start();
  do
  {
    Timer.tik();
    sensorAxia->peek();
    sensor_data[0] = sensorAxia->values.Fx;
    sensor_data[1] = sensorAxia->values.Fy;
    sensor_data[2] = sensorAxia->values.Fz;
    sensor_data[3] = sensorAxia->values.Tx;
    sensor_data[4] = sensorAxia->values.Ty;
    sensor_data[5] = sensorAxia->values.Tz;

    {   // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      //(*data_struct.datavecB_)[9] = sensor_data[1]; // nao funciona, mas:
      memcpy(aux_data, *data_struct.datavecB_, sizeof(aux_data));
      aux_data[9] = sensor_data[1]; // só alteramos o último elemento
      memcpy(*data_struct.datavecB_, aux_data, sizeof(aux_data));
      memcpy(*data_struct.datavecF_, sensor_data, sizeof(sensor_data)); // para log
    }   // fim da sessao critica

    Timer.tak();
  } while (!Timer.end());

  {   
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0E_ = false;
  }
}
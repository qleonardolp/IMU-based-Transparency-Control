
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
//#include <Axia80M50.h>
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <chrono>


void readFTSensor(ThrdStruct &data_struct){
  char* atiSensorIp = "192.168.1.1";
  //ValuesAxia80M50 valuesAxia;
}
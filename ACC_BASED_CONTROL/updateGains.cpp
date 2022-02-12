//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#include "QpcLoopTimer.h"     // ja inclui <windows.h>
#include "SharedStructs.h"    // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <math.h>

float constrain_float(float val, float min, float max);

void updateGains(ThrdStruct &data_struct){
  looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
  using namespace std;
#if PRIORITY
  SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

  FILE* pFile;
  float gains[18];
  float rawgains[18];
  bool isready_ctr(false);
  bool isready_log(false);
  do{ 
    {   // Espera Controle e Log:
      unique_lock<mutex> _(*data_struct.mtx_);
      isready_ctr = *data_struct.param0C_;
      isready_log = *data_struct.param0D_;
    } 
  } while (!isready_ctr || !isready_log);

  { // Avisa que está pronto:
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0F_ = true;
  }

  // inicializa looptimer
  Timer.start();
  do
  {
    Timer.tik();

    // TODO: Leitura do arquivo com ganhos...
    pFile = fopen("gains.param.txt", "rt");
    if (pFile != NULL) {
      for (size_t i = 0; i < sizeof(gains)/sizeof(float); i++)
        fscanf(pFile, "%f\n", rawgains[i]);
      fclose(pFile);
    }

    // Saturação para limitar valores (segurança):
    for (size_t i = 0; i < sizeof(gains)/sizeof(float); i++) {
      gains[i] = constrain_float(rawgains[i], 0.0f, 1.0e4f);
    }
    
    {   // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      memcpy(*data_struct.datavec_, gains, sizeof(gains));
    }   // fim da sessao critica
    Timer.tak();
  } while (!Timer.end());

  {   
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0F_ = false;
  }
}

float constrain_float(float val, float min, float max)
{
	if (isnan(val)) return (min + max)/2;

	if (val < min) return min;

	if(val > max) return max;
	
	return val;
}
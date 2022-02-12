
#include "QpcLoopTimer.h"     // ja inclui <windows.h>
#include "SharedStructs.h"    // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include <processthreadsapi.h>
#include <iostream>
#include <string>
#include <math.h>

float constrain_float(float val, float min, float max);

void updateGains(ThrdStruct &data_struct){
  using namespace std;
#if PRIORITY
  SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

  // setup stuff...
  float gains[18];
  FILE* pFile;
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
  looptimer Timer(data_struct.sampletime_);
  llint exec_time_micros = data_struct.exectime_*MILLION;
  llint t_begin = Timer.micro_now();
  do
  {
    Timer.tik();

    // TODO: Leitura do arquivo com ganhos...
    pFile = fopen("gains.param.txt", "rt");
    if (pFile != NULL) {
      //fscanf(pFile, "mi0 %f\nbeta %f\nrho %f\n", &mi0, &Beta, &Rho);
      //fclose(pFile);
    }

    // TODO: Saturação para limitar valores por segurança:

    {   // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      memcpy(*data_struct.datavec_, gains, sizeof(gains));
    }   // fim da sessao critica
    Timer.tak();
  } while (Timer.micro_now() - t_begin <= exec_time_micros);

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
//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#include "QpcLoopTimer.h"     // ja inclui <windows.h>
#include "SharedStructs.h"    // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include <processthreadsapi.h>
#include <iostream>
#include <string>

extern float constrain_float(float val, float min, float max); // definida em Controle.cpp

void updateGains(ThrdStruct &data_struct){
  looptimer Timer(data_struct.sampletime_, data_struct.exectime_);
  using namespace std;
#if PRIORITY
  SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

  const char* paramSignature = "Exo Tau Parameters File (cd97732888c9cc66b0a650d2fb5edfdc07161a78)";
  char checkSignature[67];
  FILE* pFile;
  float gains[18];
  float rawgains[18];
  for (size_t i = 0; i < sizeof(rawgains)/sizeof(float); i++) rawgains[i] = 0;
  bool isready_ctr(false);
  bool isready_log(false);
  bool file_isvalid(false);

  pFile = fopen("gains.param.txt", "rt");
  if (pFile != NULL) {
    fgets(checkSignature, 67, pFile);
    if (strcmp(paramSignature, checkSignature) == 0){
      file_isvalid = true;
      cout << "Arquivo de Parametros valido!\n";
    } else{
      cout << "Arquivo de Parametros invalido!\n";
    }
    fclose(pFile);
  }

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
    if (pFile != NULL && file_isvalid) {
      fscanf(pFile, "%s\n", checkSignature);
      for (size_t i = 0; i < sizeof(rawgains)/sizeof(float); i++)
        fscanf(pFile, "%f\n", rawgains + i);
      // Scanning Transfer Function:
      fscanf(pFile, "-> LS Transfer Funct <-\n[Or(den) >= Or(num)] (Matlab style):\nnum = [%f %f %f %f]\nden = [%f %f %f %f]\n",\
        rawgains[10],rawgains[11],rawgains[12],rawgains[13],rawgains[14],rawgains[15],rawgains[16],rawgains[17]);
      fclose(pFile);
    }

    // Saturação para limitar valores (segurança):
    for (size_t i = 0; i < sizeof(gains)/sizeof(float); i++) {
      gains[i] = constrain_float(rawgains[i], -1.0e4f, 1.0e4f);
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
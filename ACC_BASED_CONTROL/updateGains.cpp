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
  float gains[DTVC_SZ];
  float rawgains[DTVC_SZ];
  for (size_t i = 0; i < DTVC_SZ; i++) rawgains[i] = 0;
  bool isready_ctr(false);
  bool isready_log(false);
  bool file_isvalid(false);

  pFile = fopen("gains.param.txt", "rt");
  if (pFile != NULL) {
    fgets(checkSignature, 67, pFile);
    if (strcmp(paramSignature, checkSignature) == 0){
      file_isvalid = true;
      cout << " Arquivo de Parametros valido!\n";
    } else{
      cout << " Arquivo de Parametros invalido!\n";
    }
    fclose(pFile);
  }

  {
      unique_lock<mutex> _(*data_struct.mtx_);
      *data_struct.param0F_ = true;
      *data_struct.param3F_ = false;
  }

  // inicializa looptimer
  Timer.start();
  do
  {
    Timer.tik();

    pFile = fopen("gains.param.txt", "rt");
    if (pFile != NULL && file_isvalid) {
      char header[67];
      auto _bla = fscanf(pFile, "%67c", header); // empurrando o ponteiro de leitura para depois do cabecalho 'paramSignature'
      for (size_t i = 0; i < 18; i++) // ATENCAO AQUI, LIMITADO EM 18 DEVIDO A FORMATACAO DO TXT
        auto _bla = fscanf(pFile, "%f\n", rawgains + i);
      // Scanning Transfer Function:
      _bla = fscanf(pFile, "Transfer Funct Matlab\nnum [%f %f %f %f]\nden [%f %f %f %f]\n", \
      rawgains+10, rawgains+11, rawgains+12, rawgains+13, rawgains+14, rawgains+15, rawgains+16, rawgains+17);
      fclose(pFile);
    }

    // Saturação para limitar valores (segurança):
    for (size_t i = 0; i < DTVC_SZ; i++) {
      gains[i] = constrain_float(rawgains[i], -1.0e4f, 1.0e4f);
    }

    {   // sessao critica
      unique_lock<mutex> _(*data_struct.mtx_);
      *data_struct.param0F_ = true;
      memcpy(*data_struct.datavec_, gains, DTVC_SZ*sizeof(float));
    }   // fim da sessao critica
    Timer.tak();
  } while (!Timer.end());

  {   
    unique_lock<mutex> _(*data_struct.mtx_);
    *data_struct.param0F_ = false;
    *data_struct.param3F_ = true;
  }
}
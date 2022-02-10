/* 
* File:   CAN_FRAME.h
* Author: Guilherme Fernandes
*
* ESTA CLASSE REALIZA A LEITURA E ESCRITA DE FRAMES CAN NAS PLACAS DA NATIONAL INSTRUMENTS
* A MONTAGEM DO PAYLOAD DE ACORDO COM O NECESSÁRIO DEVE SER REALIZADA EXTERNAMENTE, A 
* FUNÇÃO DA CLASSE É APENAS ESCREVER E REALIZAR A LEITURA DOS COBID SELECIONADOS.
* 
* 
* Created on 26 de Janeiro de 2012, 10:19
*/

#ifndef CAN_FRAME_H
#define	CAN_FRAME_H

#include "nixnet.h"
#include <stdio.h>
#include <string>


class CANFrame{

private:

  //Variáveis da classe
  nxSessionRef_t  g_SessionRef;

  // Declare all variables for function 
  int i, j, type;
  int l_TypedChar;
  char *l_pFrameArray;
  u8 l_Flags;
  u8 l_Type;
  u32 l_MyNumberOfBytesForFrames;
  u8 l_myBuffer[24];
  nxFrameVar_t * l_MyFramePtr;

public:

  //INICIALIZADOR DO FRAME CAN
  void initCANFrame(int tipo, char *l_pSelectedInterface, char *l_pSelectedDatabase, 
    char *l_pSelectedCluster, char *l_pSelectedFrame){

      //INICIALIZA VARIAVEIS
      nxStatus_t g_Status;
      type = tipo;
      i = 0;
      j = 0;
      g_Status = 0;
      l_pFrameArray = NULL;
      l_Flags = 0;
      l_MyNumberOfBytesForFrames = 24;
      l_Type = nxFrameType_CAN_Data;


      //Create the signal array to create the session 
      l_pFrameArray = (char *) malloc(strlen(l_pSelectedFrame) + 1); 
      l_pFrameArray[0] = 0;
      l_pFrameArray = strcat(l_pFrameArray,l_pSelectedFrame);

      //Create an xnet session for Signal Input
      if (type == 1) {
        g_Status = nxCreateSession (l_pSelectedDatabase,l_pSelectedCluster,
          l_pFrameArray,l_pSelectedInterface,nxMode_FrameOutSinglePoint, &g_SessionRef);
      }
      else{
        g_Status = nxCreateSession (l_pSelectedDatabase,l_pSelectedCluster,
          l_pFrameArray,l_pSelectedInterface,nxMode_FrameInSinglePoint, &g_SessionRef);
      }

  }

  //CONSTRUTOR DA CLASSE - default
  CANFrame() {

  }

  //CONSTRUTOR DA CLASSE
  CANFrame(int tipo, char *CAN_interface, char *CAN_database, char *CAN_cluster, char *CAN_frame){
    initCANFrame(tipo, CAN_interface, CAN_database, CAN_cluster, CAN_frame);
  }


  //=============================================================================  
  // Display Error Function 
  //============================================================================= 
  void PrintStat(nxStatus_t l_status, char *source) 
  {
    char statusString[1024];

    if (nxSuccess != l_status) 
    {  
      nxStatusToString (l_status,sizeof(statusString),statusString);
      printf("NI-XNET Status: %s",statusString);
      nxClear(g_SessionRef);
      printf("\n------------------\n");
    }
  }

  //Método de leitura de um frame
  void write(u8 payload[8], int payload_len){

    l_MyFramePtr=(nxFrameVar_t*)l_myBuffer;
    nxStatus_t g_Status;
    g_Status = 0;

    //Configura o valor dos FRAMES
    l_MyFramePtr->Timestamp=0;
    l_MyFramePtr->Flags=l_Flags;
    l_MyFramePtr->Identifier=66; 
    l_MyFramePtr->Type=l_Type;
    l_MyFramePtr->PayloadLength=payload_len;

    //Configura o valor do FRAME
    l_MyFramePtr->Payload[0]=payload[0];
    l_MyFramePtr->Payload[1]=payload[1];
    l_MyFramePtr->Payload[2]=payload[2];
    l_MyFramePtr->Payload[3]=payload[3];
    l_MyFramePtr->Payload[4]=payload[4];
    l_MyFramePtr->Payload[5]=payload[5];
    l_MyFramePtr->Payload[6]=payload[6];
    l_MyFramePtr->Payload[7]=payload[7];

    //ENVIA FRAME PARA A INTERFACE CAN
    g_Status = nxWriteFrame(g_SessionRef,&l_myBuffer,l_MyNumberOfBytesForFrames, 10);
    if(0 != g_Status)
    {
      PrintStat(g_Status,"nxWriteFrame");  
    }

  }


  //Método de leitura de um frame
  void read(u8 lpayload[]){

    //DECLARAÇÃO DE VARIAVEIS
    u32 temp;
    nxStatus_t g_Status;
    g_Status = 0;

    g_Status = nxReadFrame (g_SessionRef,l_myBuffer,sizeof(l_myBuffer),0, &temp);

    if(nxSuccess == g_Status)
    {
      l_MyFramePtr = (nxFrameVar_t *)(l_myBuffer);
      lpayload[0] = l_MyFramePtr->Payload[0]; 
      lpayload[1] = l_MyFramePtr->Payload[1]; 
      lpayload[2] = l_MyFramePtr->Payload[2]; 
      lpayload[3] = l_MyFramePtr->Payload[3]; 
      lpayload[4] = l_MyFramePtr->Payload[4]; 
      lpayload[5] = l_MyFramePtr->Payload[5]; 
      lpayload[6] = l_MyFramePtr->Payload[6]; 
      lpayload[7] = l_MyFramePtr->Payload[7];
    }
    else 
      PrintStat(g_Status,"nxReadFrame");  
  }
};
#endif	/* CAN_FRAME_H */


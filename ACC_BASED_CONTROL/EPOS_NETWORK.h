/* 
* File:   EPOS_NETWORK.h
* Author: GUILHERME FERNANDES
*
* Created on 26 de Janeiro de 2012, 10:26
*/

#ifndef EPOS_NETWORK_H
#define	EPOS_NETWORK_H

#include "CAN_FRAME.h"

class EPOS_NETWORK{

private:

  //FRAME NMT DA REDE
  CANFrame NMT;
  CANFrame SYNC;


public:

  //Construtor da classe
  EPOS_NETWORK(){
  }


  //Construtor da classe
  EPOS_NETWORK(char *l_pSelectedInterface, char *l_pSelectedDatabase, char *l_pSelectedCluster){

    //ENDERE«A E INICIALIZA O FRAME NMT
    char *FRAME = "NMT";
    NMT.initCANFrame(1, "CAN2", l_pSelectedDatabase, l_pSelectedCluster, FRAME); //2

    //ENDERE«A E INICIALIZA O FRAME NMT
    SYNC.initCANFrame(1, "CAN2", l_pSelectedDatabase, l_pSelectedCluster, "SYNC"); //2


  }

  //M…TODO QUE ENVIA O START PARA A TRANSMISS√O DOS PDOS PARA A REDE CAN
  void StartPDOS(int node_id){

    //MONTA O PAYLOAD DE WRITE
    u8 payload_start[8];
    payload_start[0] = 0x01;
    payload_start[1] = node_id;
    payload_start[2] = 0x0;
    payload_start[3] = 0x0;
    payload_start[4] = 0x0;
    payload_start[5] = 0x0;
    payload_start[6] = 0x0;
    payload_start[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.write(payload_start, 8);
  }

  void sync(){

    //MONTA O PAYLOAD DE WRITE
    u8 payload_start[8];
    payload_start[0] = 0x0;
    payload_start[1] = 0x0;
    payload_start[2] = 0x0;
    payload_start[3] = 0x0;
    payload_start[4] = 0x0;
    payload_start[5] = 0x0;
    payload_start[6] = 0x0;
    payload_start[7] = 0x0;

    SYNC.write(payload_start, 8);

  }

  //M…TODO QUE ENVIA O STOP PARA A TRANSMISS√O DOS PDOS PARA A REDE CAN
  void StopPDOS(int node_id){

    //ENVIA COMANDO DE STOP PDOS PARA A REDE
    //MONTA O PAYLOAD DE WRITE
    u8 payload_stop[8];
    payload_stop[0] = 0x80;
    payload_stop[1] = node_id;
    payload_stop[2] = 0x0;
    payload_stop[3] = 0x0;
    payload_stop[4] = 0x0;
    payload_stop[5] = 0x0;
    payload_stop[6] = 0x0;
    payload_stop[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.write(payload_stop, 8);

  }

  //M…TODO QUE ENVIA O RESET TODA A COMUNICA«√O DA REDE
  void ResetComm(){

    //ENVIA COMANDO DE RESET PDOS PARA A REDE
    //MONTA O PAYLOAD DE WRITE
    u8 payload_reset[8];
    payload_reset[0] = 0x82;
    payload_reset[1] = 0x0;
    payload_reset[2] = 0x0;
    payload_reset[3] = 0x0;
    payload_reset[4] = 0x0;
    payload_reset[5] = 0x0;
    payload_reset[6] = 0x0;
    payload_reset[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.write(payload_reset, 8);
  }

  //M…TODO QUE ENVIA O RESET PARA TODOS OS NOS DA REDE
  void ResetNodes(){

    //ENVIA COMANDO DE RESET PDOS PARA A REDE
    //MONTA O PAYLOAD DE WRITE
    u8 payload_reset[8];
    payload_reset[0] = 0x81;
    payload_reset[1] = 0x0;
    payload_reset[2] = 0x0;
    payload_reset[3] = 0x0;
    payload_reset[4] = 0x0;
    payload_reset[5] = 0x0;
    payload_reset[6] = 0x0;
    payload_reset[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.write(payload_reset, 8);
  }

  // Class Destroyer
  ~EPOS_NETWORK(){
  }
};

#endif	/* EPOS_NETWORK_H */


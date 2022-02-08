/* 
* File:   AXIS.h
* Author: GUILHERME FERNANDES
* Created on 26 de Janeiro de 2012, 19:34
*/
#include "CAN_FRAME.h"
// #include <stdlib.h>
// #include <sstream>
#include <time.h>

#define HOMMING_MODE			0x06
#define PROFILE_VELOCITY_MODE	0x03
#define PROFILE_POSITION_MODE	0x01
#define POSITION_MODE			0xFF
#define VELOCITY_MODE			0xFE
#define CURRENT_MODE			0xFD
#define MASTER_ENCODER_MODE		0xFB
#define STEP_MODE				0xFA

#ifndef AXIS_H
#define	AXIS_H

class AXIS
{

private:

	//FRAME NMT DA REDE
	CANFrame SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX;
	short int actualCurrent, currentSetpoint, analogInput_01, analogInput_02;
	short unsigned int statusWord, controlWord;
	int actualPosition, actualVelocity, positionSetpoint, velocitySetpoint;
	short int output;

public:

	int PDOgetAnalogValue_01(void)
	{
		return analogInput_01;
	}

	int PDOgetAnalogValue_02(void)
	{
		return analogInput_02;
	}


	int PDOgetActualPosition(void)
	{
		return actualPosition;
	}

	int PDOgetActualCurrent(void)
	{
		return actualCurrent;
	}

	int PDOgetActualVelocity(void)
	{
		return actualVelocity;
	}

	void PDOsetPositionSetpoint(int setpoint)
	{
		positionSetpoint = setpoint;
		currentSetpoint = 0;
	}

	void PDOsetVelocitySetpoint(int setpoint)
	{
		velocitySetpoint = setpoint;
	}

	void PDOsetCurrentSetpoint(short int setpoint)
	{
		currentSetpoint = setpoint;
		positionSetpoint = actualPosition;
	}

	void PDOsetOutput(int state)
	{
		if (state > 0) output = 0x2000;
		else output = 0x0000;
	}

	bool PDOgetStatusWord_Ready2SwitchOn()
	{
		if ((statusWord & 1) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_SwtchedOn()
	{
		if ((statusWord & 2) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_OperationEnabled()
	{
		if ((statusWord & 4) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_Fault()
	{
		if ((statusWord & 8) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_VoltageEnabled()
	{
		if ((statusWord & 2) > 16) return true;
		else return false;
	}

	bool PDOgetStatusWord_QuickStop()
	{
		if ((statusWord & 32) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_SwitchOnDisable()
	{
		if ((statusWord & 64) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_RemoteNMT()
	{
		if ((statusWord & 512) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_TargetReached()
	{
		if ((statusWord & 1024) > 0) return true;
		else return false;
	}

	bool PDOgetStatusWord_SetpointAck()
	{
		if ((statusWord & 4096) > 0) return true;
		else return false;
	}

	void PDOsetControlWord_SwitchOn(bool state)
	{
		if (state) controlWord = controlWord | 1;
		else controlWord = controlWord & (~1);
	}

	void PDOsetControlWord_EnableVoltage(bool state)
	{
		if (state) controlWord = controlWord | 2;
		else controlWord = controlWord & (~2);
	}

	void PDOsetControlWord_QuickStop(bool state)
	{
		if (state) controlWord = controlWord | 4;
		else controlWord = controlWord & (~4);
	}

	void PDOsetControlWord_EnableOperation(bool state)
	{
		if (state) controlWord = controlWord | 8;
		else controlWord = controlWord & (~8);
	}

	void PDOsetControlWord_NewSetpoint(bool state)
	{
		if (state) controlWord = controlWord | 16;
		else controlWord = controlWord & (~16);
	}

	void PDOsetControlWord_ChangeImmediatly(bool state)
	{
		if (state) controlWord = controlWord | 32;
		else controlWord = controlWord & (~32);
	}

	void PDOsetControlWord_AbsRel(bool state)
	{
		if (state) controlWord = controlWord | 64;
		else controlWord = controlWord & (~64);
	}

	void PDOsetControlWord_FaultReset(bool state)
	{
		if (state) controlWord = controlWord | 128;
		else controlWord = controlWord & (~128);
	}

	void PDOsetControlWord_Halt(bool state)
	{
		if (state) controlWord = controlWord | 256;
		else controlWord = controlWord & (~256);
	}


	AXIS()
	{
	}

	//Construtor da classe
	AXIS(char *l_pSelectedInterface, char *l_pSelectedDatabase, char *l_pSelectedCluster, char *NET_ADDRESS)
	{
		//Inicializa variaveis
		actualCurrent =		0;
		currentSetpoint =	0;
		statusWord =		0;
		controlWord =		0;
		actualPosition =	0;
		actualVelocity =	0;
		positionSetpoint =	0;
		velocitySetpoint =	0;

		//CRIA E INICIALIZA OS CAN FRAMES PARA O EIXO
		char *SDO_TX_FRAME = "SDO_TX_0";
		char *SDO_RX_FRAME = "SDO_RX_0";
		char *PDO01_TX_FRAME = "PDO01_TX_0";
		char *PDO01_RX_FRAME = "PDO01_RX_0";
		char *PDO02_TX_FRAME = "PDO02_TX_0";
		char *PDO02_RX_FRAME = "PDO02_RX_0";
		char temp[15];

		//Inicializa os CAN_FRAMES
		strcpy(temp, SDO_TX_FRAME);
		strcat(temp, NET_ADDRESS);
		SDO_TX.initCANFrame(1, "CAN2", l_pSelectedDatabase, l_pSelectedCluster, temp); //2

		strcpy(temp, SDO_RX_FRAME);
		strcat(temp, NET_ADDRESS);
		SDO_RX.initCANFrame(0, "CAN1", l_pSelectedDatabase, l_pSelectedCluster, temp);

		strcpy(temp, PDO01_TX_FRAME);
		strcat(temp, NET_ADDRESS);
		PDO01_TX.initCANFrame(1, "CAN2", l_pSelectedDatabase, l_pSelectedCluster, temp); //2

		strcpy(temp, PDO01_RX_FRAME);
		strcat(temp, NET_ADDRESS);
		PDO01_RX.initCANFrame(0, "CAN1", l_pSelectedDatabase, l_pSelectedCluster, temp);

		strcpy(temp, PDO02_TX_FRAME);
		strcat(temp, NET_ADDRESS);
		PDO02_TX.initCANFrame(1, "CAN2", l_pSelectedDatabase, l_pSelectedCluster, temp); //2

		strcpy(temp, PDO02_RX_FRAME);
		strcat(temp, NET_ADDRESS);
		PDO02_RX.initCANFrame(0, "CAN1", l_pSelectedDatabase, l_pSelectedCluster, temp);

	}

	//Realiza a leitura do PDO01 - Posição e Corrente
	void ReadPDO01()
	{

		u8 payload[8];

		PDO01_RX.read(payload);

		//ATUALIZA O VALOR DE POSIÇÃO
		actualPosition = (payload[3]*0x1000000) + (payload[2]*0x10000) + (payload[1]*0x100) + (payload[0]);

		//ATUALIZA O VALOR DE CORRENTE
		actualCurrent = (payload[5]*0x100) + (payload[4]);

		//ATUALIZA A STATUS WORD
		statusWord = (payload[7]*0x100) + (payload[6]);

	}

	//Realiza a leitura do PDO02 - Velocidade
	void ReadPDO02()
	{

		u8 payload[8];

		PDO02_RX.read(payload);

		//ATUALIZA O VALOR DE POSIÇÃO
		actualVelocity = (payload[3]*0x1000000) + (payload[2]*0x10000) + (payload[1]*0x100) + (payload[0]);

		//ATUALIZA O VALOR DA ANNALÓGICA 01
		analogInput_01 = (payload[5]*0x100) + (payload[4]);

		//ATUALIZA O VALOR DA ANALÓGIA 02
		analogInput_02 = (payload[7]*0x100) + (payload[6]);

	}

	//Método de escrita do PDO01 - Setpoint de Posicao e Corrente
	void WritePDO01()
	{

		//MONTA O PAYLOAD DE WRITE
		u8 payload[8];

		payload[0] = (positionSetpoint & 0x000000ff);
		payload[1] = (positionSetpoint & 0x0000ff00)/0x100;
		payload[2] = (positionSetpoint & 0x00ff0000)/0x10000;
		payload[3] = (positionSetpoint & 0xff000000)/0x1000000;
		payload[4] = (currentSetpoint & 0x000000ff);
		payload[5] = (currentSetpoint & 0x0000ff00)/0x100; 
		payload[6] = (controlWord & 0x000000ff);
		payload[7] = (controlWord & 0x0000ff00)/0x100; 

		//ENVIA COMANDO PARA A REDE
		PDO01_TX.write(payload, 8);
	}

	//Método de escrita do PDO02 - Setpoint de Velocidade
	void WritePDO02(){

		//MONTA O PAYLOAD DE WRITE
		u8 payload[8];

		payload[0] = (velocitySetpoint & 0x000000ff);
		payload[1] = (velocitySetpoint & 0x0000ff00)/0x100;
		payload[2] = (velocitySetpoint & 0x00ff0000)/0x10000;
		payload[3] = (velocitySetpoint & 0xff000000)/0x1000000;
		payload[4] = (output & 0x000000ff);
		payload[5] = (output & 0x0000ff00)/0x100; 
		payload[6] = (0 & 0x000000ff);
		payload[7] = (0 & 0x0000ff00)/0x100; 

		//ENVIA COMANDO PARA A REDE
		PDO02_TX.write(payload, 8);

	}

	//MÉTODO DE LEITURA DE SDO
	int ReadSDO(int Index, int SubIndex)
	{

		//MONTA O PAYLOAD DE WRITE requisiçãoo de leitura
		u8 payload[8];
		u8 payload_r[8];

		payload[0] = 0x40; 
		payload[1] = (Index & 0x000000ff);
		payload[2] = (Index & 0x0000ff00)/0xff;
		payload[3] = SubIndex;
		payload[4] = 0;
		payload[5] = 0;
		payload[6] = 0;
		payload[7] = 0;

		//ENVIA COMANDO DE START PDOS PARA A REDE
		SDO_TX.write(payload, 8);

		clock_t endwait;
		endwait = (clock_t) clock() + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		SDO_RX.read(payload_r);

		int resultado;

		resultado = (payload_r[7]*0x1000000) + (payload_r[6]*0x10000) + (payload_r[5]*0x100) + (payload_r[4]);

		return resultado;

	}

	//MÉTODO DE ESCRITA DE UM SDO
	void WriteSDO(int Index, u8 SubIndex, short int Value)
	{

		//MONTA O PAYLOAD DE WRITE
		u8 payload[8];

		payload[0] = 0x22; 


		payload[1] = (Index & 0x000000ff);
		payload[2] = (Index & 0x0000ff00)/0x100;
		payload[3] = SubIndex;
		payload[4] = (Value & 0x000000ff);
		payload[5] = (Value & 0x0000ff00)/0x100;
		payload[6] = (Value & 0x00ff0000)/0x10000;
		payload[7] = (Value & 0xff000000)/0x1000000;

		//ENVIA COMANDO PARA A REDE
		SDO_TX.write(payload, 8);		
	}


	void VCS_SetOperationMode (int Mode)
	{

		//Escreve o modo de operação desejado
		WriteSDO(0x6060, 0x00, Mode);
		return;  

	}


	void SetDigitalOutput()
	{

		//Escreve o modo de operação desejado
		WriteSDO(0x6060, 0x00, 0xff);
		return;  

	}

	void ResetDigitalOutput()
	{

		//Escreve o modo de operação desejado
		WriteSDO(0x6060, 0x00, 0x00);
		return;  

	}
};

#endif	/* AXIS_H */
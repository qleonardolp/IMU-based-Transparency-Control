
#include "CAN_FRAME.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string.h> // OJO EL UNICO CAMBIO FUE PONERLE .h
#include <time.h>

////
#define CALIBRACAO_CARGA 0.00030519
//#define PROFILE_VELOCITY_MODE  0x03
//#define PROFILE_POSITION_MODE  0x01
//#define POSITION_MODE  0xFF
//#define VELOCITY_MODE 0xFE
//#define CURRENT_MODE 0xFD
//#define MASTER_ENCODER_MODE 0xFB
//#define STEP_MODE 0xFA

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

class FORCE_SENSOR
{

private:
	//FRAME NMT DA REDE
	CANFrame PDO02_RX, PDO03_RX;
	double Fx, Fy, Fz, Tx, Ty, Tz;
	double sg0, sg1, sg2, sg3, sg4, sg5;
	double zsg0, zsg1, zsg2, zsg3, zsg4, zsg5;

public:
	double getFx(void)
	{
		return Fx;
	}

	double getFy(void)
	{
		return Fy;
	}

	double getFz(void)
	{
		return Fz;
	}

	double getTx(void)
	{
		return Tx;
	}

	double getTy(void)
	{
		return Ty;
	}

	double getTz(void)
	{
		return Tz;
	}

	FORCE_SENSOR()
	{
	}

	//Construtor da classe
	FORCE_SENSOR(char *l_pSelectedInterface, char *l_pSelectedDatabase, char *l_pSelectedCluster, char *NET_ADDRESS)
	{

		//Inicializa variaveis
		Fx = Fy = Fz = Tx = Ty = Tz = 0.0;
		sg0 = sg1 = sg2 = sg3 = sg4 = sg5 = 0.0;
		zsg0 = zsg1 = zsg2 = zsg3 = zsg4 = zsg5 = 0.0;

		//CRIA E INICIALIZA OS CAN FRAMES PARA O EIXO

		char *PDO02_RX_FRAME = "PDO02_RX_0";
		char *PDO03_RX_FRAME = "PDO03_RX_0";
		char temp[15];

		strcpy(temp, PDO02_RX_FRAME);
		strcat(temp, NET_ADDRESS);
		PDO02_RX.initCANFrame(0, "CAN1", l_pSelectedDatabase, l_pSelectedCluster, temp);

		strcpy(temp, PDO03_RX_FRAME);
		strcat(temp, NET_ADDRESS);
		PDO03_RX.initCANFrame(0, "CAN1", l_pSelectedDatabase, l_pSelectedCluster, temp);
	}

	void ZERA_TRANSDUTOR()
	{

		u8 payload[8];

		PDO02_RX.read(payload);

		sg0 = payload[0] + payload[1] * 0x100;

		sg1 = payload[2] + payload[3] * 0x100;

		sg2 = payload[4] + payload[5] * 0x100;

		sg3 = payload[6] + payload[7] * 0x100;

		PDO03_RX.read(payload);

		sg4 = payload[0] + payload[1] * 0x100;

		sg5 = payload[2] + payload[3] * 0x100;

		if (sg0 < 32767)
			sg0 = sg0 * CALIBRACAO_CARGA;
		else
			sg0 = (sg0 - 65536) * CALIBRACAO_CARGA;

		if (sg1 < 32767)
			sg1 = sg1 * CALIBRACAO_CARGA;
		else
			sg1 = (sg1 - 65536) * CALIBRACAO_CARGA;

		if (sg2 < 32767)
			sg2 = sg2 * CALIBRACAO_CARGA;
		else
			sg2 = (sg2 - 65536) * CALIBRACAO_CARGA;

		if (sg3 < 32767)
			sg3 = sg3 * CALIBRACAO_CARGA;
		else
			sg3 = (sg3 - 65536) * CALIBRACAO_CARGA;

		if (sg4 < 32767)
			sg4 = sg4 * CALIBRACAO_CARGA;
		else
			sg4 = (sg4 - 65536) * CALIBRACAO_CARGA;

		if (sg5 < 32767)
			sg5 = sg5 * CALIBRACAO_CARGA;
		else
			sg5 = (sg5 - 65536) * CALIBRACAO_CARGA;

		zsg0 = sg0;
		zsg1 = sg1;
		zsg2 = sg2;
		zsg3 = sg3;
		zsg4 = sg4;
		zsg5 = sg5;
	}

	//Realiza a leitura do PDO01 - Posi��o e Corrente
	void Update()
	{

		u8 payload[8];

		PDO02_RX.read(payload);

		sg0 = payload[0] + payload[1] * 0x100;

		sg1 = payload[2] + payload[3] * 0x100;

		sg2 = payload[4] + payload[5] * 0x100;

		sg3 = payload[6] + payload[7] * 0x100;

		PDO03_RX.read(payload);

		sg4 = payload[0] + payload[1] * 0x100;

		sg5 = payload[2] + payload[3] * 0x100;

		if (sg0 < 32767)
			sg0 = sg0 * CALIBRACAO_CARGA;
		else
			sg0 = (sg0 - 65536) * CALIBRACAO_CARGA;

		if (sg1 < 32767)
			sg1 = sg1 * CALIBRACAO_CARGA;
		else
			sg1 = (sg1 - 65536) * CALIBRACAO_CARGA;

		if (sg2 < 32767)
			sg2 = sg2 * CALIBRACAO_CARGA;
		else
			sg2 = (sg2 - 65536) * CALIBRACAO_CARGA;

		if (sg3 < 32767)
			sg3 = sg3 * CALIBRACAO_CARGA;
		else
			sg3 = (sg3 - 65536) * CALIBRACAO_CARGA;

		if (sg4 < 32767)
			sg4 = sg4 * CALIBRACAO_CARGA;
		else
			sg4 = (sg4 - 65536) * CALIBRACAO_CARGA;

		if (sg5 < 32767)
			sg5 = sg5 * CALIBRACAO_CARGA;
		else
			sg5 = (sg5 - 65536) * CALIBRACAO_CARGA;

		sg0 = sg0 - zsg0;
		sg1 = sg1 - zsg1;
		sg2 = sg2 - zsg2;
		sg3 = sg3 - zsg3;
		sg4 = sg4 - zsg4;
		sg5 = sg5 - zsg5;

		Fx = -1 * (-3.43670 * sg0 + 0.28116 * sg1 + 1.07828 * sg2 - 29.80109 * sg3 - 1.67570 * sg4 + 31.73500 * sg5);
		Fy = -1 * (-2.16732 * sg0 + 37.43346 * sg1 + 0.27676 * sg2 - 17.45348 * sg3 + 1.24554 * sg4 - 18.28849 * sg5);
		Fz = -1 * (18.33413 * sg0 + -0.62813 * sg1 + 18.48429 * sg2 - 0.54426 * sg3 + 18.66260 * sg4 - 0.59200 * sg5);
		Tx = -1 * (0.11001 * sg0 + -0.83890 * sg1 - 32.39104 * sg2 + 1.65870 * sg3 + 32.53940 * sg4 - 0.51374 * sg5);
		Ty = -1 * (36.67222 * sg0 - 1.41533 * sg1 - 18.71469 * sg2 + 0.17912 * sg3 - 18.93809 * sg4 + 1.09165 * sg5);
		Tz = 1 * (1.03815 * sg0 - 18.00959 * sg1 + 0.58368 * sg2 - 16.49001 * sg3 + 0.75244 * sg4 - 17.67885 * sg5);
	}
};

#endif /* AXIS_H */

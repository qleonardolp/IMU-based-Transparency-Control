#ifndef GENERAL_HEADER_H
#define GENERAL_HEADER_H

#define		  SAMPLE_TIME		  0.0083333		    // Tempo de Amostragem [s]

// ENDEREÇAMENTO DA BASE DE DADOS CAN

char* CAN_INTERFACE =		"CAN1";
char* CAN_DATABASE =	"database";
char* CAN_CLUSTER =		  "NETCAN";
char* NET_ID_SERVO_01 =		   "1";
char* NET_ID_SERVO_02 =		   "2";

//INICIALIZANDO O QUERY PERFORMANCE PARA CONTROLE DOS CICLOS DE 8.333 MS

LARGE_INTEGER tick_after, tick_before, TICKS_PER_SECOND;
long long int ticksSampleTime, final_time;

//INICIALIZAÇÃO DA REDE CAN

EPOS_NETWORK  epos (CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);

//INICIALIZAÇÃO DAS EPOS

AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);
AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

clock_t endwait;	// Clock de segundos

FILE * logger;		// Ponteiro para arquivo


#endif /* GENERAL_HEADER_H */
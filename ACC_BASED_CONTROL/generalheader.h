#ifndef GENERAL_HEADER_H
#define GENERAL_HEADER_H

// CONSTANTES

#define		GEAR_RATIO		150.0		// Redução do Sistema
#define		ENCODER_IN		4096		// Resolução do encoder do motor
#define		ENCODER_OUT		2048		// Resolução do encoder de saída
#define		STIFFNESS		104.0		// Constante da mola SEA [???]
#define		SAMPLE_TIME		0.005		// Tempo de Amostragem [s]

#define     GRAVITY         9.8066      // [m/s^2]
#define     INERTIA_EXO     0.0655      // [Kg.m^2], +- 0.0006, estimado em 2019-08-21
#define     KP              0.1000      // [Kg.m^2]
#define     KI              9.5238      // [Kg.m^2/s]
#define     RATE            168.00      // [Hz]		  ?? Ts = 0.005 -> 200 Hz ??
#define     LPF_FC          7.0000      // [Hz] Low Pass Filter Frequency Cutoff

#define     LPF_SMF         ( (2*M_PI / RATE) / (2*M_PI / RATE + 1 / LPF_FC) )    // Low Pass Filter Smoothing Factor


// ENDEREÇAMENTO DA BASE DE DADOS CAN

char* CAN_INTERFACE =		"CAN1";
char* CAN_DATABASE =	"database";
char* CAN_CLUSTER =		  "NETCAN";
char* NET_ID_SERVO_01 =		   "1";
char* NET_ID_SERVO_02 =		   "2";

//INICIALIZANDO O QUERY PERFORMANCE PARA CONTROLE DOS CICLOS DE 5MS

LARGE_INTEGER tick_after, tick_before, TICKS_PER_SECOND;
long long int ticksSampleTime, final_time;
int total_time;

//INICIALIZAÇÃO DA REDE CAN

EPOS_NETWORK  epos (CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);

//INICIALIZAÇÃO DAS EPOS

AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);
AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

clock_t endwait;	// Clock de segundos

FILE * logger;		// Ponteiro para arquivo


#endif /* GENERAL_HEADER_H */
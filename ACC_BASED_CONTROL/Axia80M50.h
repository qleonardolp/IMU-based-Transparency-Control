#pragma once

#include <sys/types.h>
#include <WS2tcpip.h>
#include <winsock2.h>
#include <windows.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma comment(lib,"ws2_32.lib")

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define BIAS_CMD 0x0042
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

#define byte char
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;

typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

typedef union valuesAxia80M50_struct{
  struct{
    float Fx;
    float Fy;
    float Fz;
    float Tx;
    float Ty;
    float Tz;
  };

    float raw[6];

}ValuesAxia80M50;



class Axia80M50{
    public:

        char * IP_;
        int err;
        int socketHandle;			
        struct sockaddr_in addr;	
        struct hostent *he;			
        byte request[8];			
        RESPONSE resp;				
        byte response[36];	
        bool __INIT__;		
        ValuesAxia80M50 values;
       

    
        Axia80M50(char * IP){
            this->IP_ = IP;
            __INIT__ = false;
        }

        bool init(){
            WSADATA Data;
            WSAStartup(MAKEWORD(2, 2), &Data);
            socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
            
            if (socketHandle == -1) {
                exit(1);
            }

            *(uint16*)&request[0] = htons(0x1234); /* standard header. */
            *(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
            *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
            
            /* Sending the request. */
            //he = gethostbyname(IP_);
            //memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);

            if(inet_pton(AF_INET, IP_ , &addr.sin_addr)<=0) 
            {
                printf("\nInvalid address/ Address not supported \n");
                return false;
            }

            addr.sin_family = AF_INET;
            addr.sin_port = htons(PORT);
            
            err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
            if (err == -1) {
                return false;
            }
            __INIT__ = true;
            return true;
        }

        RESPONSE peek(){

            *(uint16*)&request[0] = htons(0x1234); 
            *(uint16*)&request[2] = htons(COMMAND); 
            *(uint32*)&request[4] = htonl(NUM_SAMPLES);

            send( socketHandle, request, 8, 0 );

            recv( socketHandle, response, 36, 0 );
            resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
            resp.ft_sequence = ntohl(*(uint32*)&response[4]);
            resp.status = ntohl(*(uint32*)&response[8]);
            for(int i = 0; i < 6; i++ ) {
                resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
                values.raw[i] = resp.FTData[i] / 416666.0f;
            }

            return resp;

        }

        void print(RESPONSE resp){
            // printf( "0x%08x ", resp.status );
            for (int i =0;i < 6;i++) {
                printf("%12d", resp.FTData[i]);
            }
            printf("\n");
        }
        void print(){
            print(this->resp);
        };

        void bias(){
            
            *(uint16*)&request[0] = htons(0x1234); /* standard header. */
            *(uint16*)&request[2] = htons(BIAS_CMD); /* per table 9.1 in Net F/T user manual. */
            *(uint32*)&request[4] = htonl(NUM_SAMPLES);

            send( socketHandle, request, 8, 0 );
        };

};
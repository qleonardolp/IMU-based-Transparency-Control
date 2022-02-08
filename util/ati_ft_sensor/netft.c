/* Simple demo showing how to communicate with Net F/T using C language. */

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

int main ( int argc, char ** argv ) {
	int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
	int i;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */
	char * AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */

	if ( 2 > argc )
	{
		fprintf( stderr, "Usage: %s IPADDRESS\n", argv[0] );
		return -1;
	}
	
	/* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1) {
		exit(1);
	}
	
	*(uint16*)&request[0] = htons(0x1234); /* standard header. */
	*(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
	
	/* Sending the request. */
	he = gethostbyname(argv[1]);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}
	send( socketHandle, request, 8, 0 );

	/* Receiving the response. */
	recv( socketHandle, response, 36, 0 );
	resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
	resp.ft_sequence = ntohl(*(uint32*)&response[4]);
	resp.status = ntohl(*(uint32*)&response[8]);
	for( i = 0; i < 6; i++ ) {
		resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
	}

	/* Output the response data. */
	printf( "Status: 0x%08x\n", resp.status );
	for (i =0;i < 6;i++) {
		printf("%s: %d\n", AXES[i], resp.FTData[i]);
	}

	return 0;
}

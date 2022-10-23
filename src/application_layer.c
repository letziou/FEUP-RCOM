// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

int sequenceNumber, maxNTries;
unsigned int alarmEnabled;
LinkLayer l;

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole lr;
    if(strcmp(role,"tx") == 0) lr = 0;
    else if(strcmp(role,"rx") == 0) lr = 1;

    if(lr != 0 && lr != 1){
        printf("\nWRONG ROLE\nPlease type tx for transmitter or rx for receiver\n");
        return ;
    }

    
    for(int i=0; i< strlen(serialPort) ;i++)
        l.serialPort[i] = serialPort[i];
    l.baudRate = baudRate;
    l.nRetransmissions = nTries;
    maxNTries = nTries;
    l.timeout = timeout;
    sequenceNumber = 0;
    alarmEnabled = FALSE;
    l.role = lr;

    if(llopen(l) == -1){
        printf("\nLLOPEN FAILED\n");
        return ;
    }
}

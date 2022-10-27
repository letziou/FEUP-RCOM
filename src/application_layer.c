// Application layer protocol implementation

#include "application_layer.h"
#include <link_layer.h>
#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer l;
    strcpy(l.serialPort, serialPort);
        if(strcmp(role, "tx") == 0) l.role = LlTx; 
        else if(strcmp(role, "rx") == 0) l.role = LlRx;
        else printf("nERROR IN ROLE\n");
    l.baudRate = baudRate;
    l.nRetransmissions = nTries;
    l.timeout = timeout;

    if(llopen(l) < 0){
        printf("\nERROR IN LLOPEN\n");
        return ;
    }
    
}

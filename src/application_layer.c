// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#define CONTROL0 0x00
#define CONTROL1 0x01
#define CONTROLSTART 0x02
#define CONTROLEND 0x03

#define fragmentSize 256

int sequenceNumber, maxNTries, file;
unsigned int alarmEnabled;
LinkLayer l;
extern int fd;

void sendData(const char *filename);
int setFile(const char *filename);
int getFileSize(FILE *file);
int sendControlPacket(unsigned char control, const char *filename);
int sendPacket(int seqNumber, unsigned char* buf, int size);

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

    if(lr == 0){
        sendData(filename);
    }
    else if(lr == 1){
        //receiveData(file, filename);
    }

    llclose(0);
    return ;
}

void sendData(const char *filename){
    if(sendControlPacket(CONTROLSTART, filename) < 0){
        printf("\nERROR IN FUNCTION sendControlPacket\n");
        return ;
    }

    int bytesRead = 0, seqNumber = 0;
    unsigned char *buf = (unsigned char*) malloc(fragmentSize + 1);

    while((bytesRead = read(fd, buf, fragmentSize)) > 0){
        if (sendPacket(seqNumber, buf, bytesRead) < 0) {
            printf("\nERROR IN FUNCTION sendPacket\n");
            free(buf);
            return ;
        }

        seqNumber++;
        seqNumber %= 255;
    }

    if (sendControlPacket(CONTROLEND, filename) < 0) {
        printf("\nERROR IN FUNCTION sendControlPacket\n");
        return ;
    }

    if(close(file) < 0){
        printf("\nERROR CLOSING FILE\n");
        return ;
    }

    free(buf);
    llclose(FALSE);

    return ;
}

int setFile(const char *filename){
    if((file = open(filename, O_RDONLY)) < 0){
        printf("\nERROR OPENING FILE\n");
        return -1;
    }
    return 0;
}

int getFileSize(FILE *file){
    // saving current position
    long int currentPosition = ftell(file);

    // seeking end of file
    if (fseek(file, 0, SEEK_END) == -1)
    {
        printf("\nERROR: Could not get file size.\n");
        return -1;
    }

    // saving file size
    long int size = ftell(file);

    // seeking to the previously saved position
    fseek(file, 0, currentPosition);

    // returning size
    return size;
}

int sendControlPacket(unsigned char control, const char *filename){

    if(control == CONTROLSTART){
        if(setFile(filename) < 0){
            printf("\nERROR GETTING FILE\n");
            return -1;
        }
    }

    struct stat fileInfo;

    if (fstat(file, &fileInfo) < 0) {
        printf("\nCOULD NOT GET INFO ON FILE\n");
        return -1;
    }
    off_t fileSize = fileInfo.st_size;
    unsigned int l1 = sizeof(fileSize);
    unsigned int l2 = strlen(filename) + 1;

    int startPacketSize = 5 + l1 + l2;

    unsigned char startPacket[startPacketSize];

    startPacket[0] = control;
    startPacket[1] = CONTROL0;
    startPacket[2] = l1;
    startPacket[3 + l1] = CONTROL1;
    startPacket[3 + l1 + 1] = l2;

    strcat((char *) startPacket + l1 + 5, filename);

    if(llwrite(startPacket, startPacketSize) < 0){
        printf("\nCOULD NOT WRITE\n");
        return -1;
    }
    
    return 0;
}

int sendPacket(int seqNumber, unsigned char* buf, int size){
    int totalSize = size + 4;
    unsigned char dataPacket[totalSize];

    dataPacket[0] = CONTROL1;
    dataPacket[1] = seqNumber;
    dataPacket[2] = size / 256;
    dataPacket[3] = size % 256;

    memcpy(&dataPacket[4], buf, size);

    if(llwrite(dataPacket, totalSize) < 0){
        printf("\nERROR SENDING PACKET\n");
        return -1;
    }
    return 0;
}
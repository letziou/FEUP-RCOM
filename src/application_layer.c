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

int sequenceNumber, maxNTries, file, fileSize;
unsigned char *fileData;
unsigned int alarmEnabled;
LinkLayer l;
extern int fd;

void sendData(const char *filename);
int setFile(const char *filename);
int getFile(const char *filename);
int sendControlPacket(unsigned char control, const char *filename);
int sendPacket(int seqNumber, unsigned char* buf, int size);
void ReceiveData(char *filename);
int receiveControlPacket(const char *filename);
int receivePacket(unsigned char **buffer, int seqNumber);

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

void ReceiveData(char *filename){
    if(receiveControlPacket(filename) < 0){
        printf("\nERROR IN receiveControlPackage\n");
        return ;
    }

    int bytesRead = 0, seqNumber = 0, counter = 0;
    unsigned char *buffer;

    while(counter < fileSize){

        bytesRead = receivePacket(&buffer, seqNumber);
        if(bytesRead < 0){
            printf("\nreceivePacket COULD NOT READ\n");
            continue;
        }

        counter += bytesRead;
        if(write(file, buffer, bytesRead) <= 0) {
            printf("\nCOULD NOT WRITE TO FILE\n");
        }

        seqNumber++;
        free(buffer);
    }

    if(receiveControlPacket(filename) < 0){
        printf("\nERROR IN receiveControlPackage\n");;
        return ;
    }

    if(close(file) < 0){
        printf("\nERROR CLOSING FILE\n");
        return ;
    }

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

int getFile(const char *filename){
    if((file = open(filename, O_CREAT|O_WRONLY|O_APPEND, S_IWUSR|S_IRUSR)) < 0) {
    printf("\nERROR OPENING FILE\n");
    return -1;
  }

  return 0;
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

int receiveControlPacket(const char *filename){
    unsigned char *read_packet;
    unsigned int packet_size = llread(read_packet);

    if(read_packet < 0){
        printf("\nERROR READING\n");
        return -1;
    }

    int index = 0;

    if(read_packet[index] == CONTROLEND){
        free(read_packet);
        return 0;
    }

    index++;
    unsigned int nBytes;
    unsigned char type;

    for(int i=0; i<2 ;i++){
        type = read_packet[index++];

        switch (type)
        {
        case CONTROL0:
            nBytes = (unsigned int) read_packet[index++];
            fileSize = *((off_t*) (read_packet + index));
            fileData = (unsigned char *) malloc(fileSize);
            index += nBytes;
            break;
        case CONTROL1:
            nBytes = (unsigned int) read_packet[index++];
            filename = (unsigned char*) malloc(nBytes + 1);
            memcpy(filename, (char *)&read_packet[index++], nBytes+1);
            getFile(filename);
            break;
        default:
            printf("\nT parameter in start control packet couldn't be recognised\n");
            break;
        }
    }

    free(read_packet);
    return 0;
}

int receivePacket(unsigned char **buffer, int seqNumber){
    unsigned char *info;
    int K = 0;

    if(llread(info) < 0) {
        printf("\nERROR IN llread\n");
        return -1;
    }

    if(info == NULL){
        printf("\nreceivePacket WITH NULL INFO\n");
        return -1;
    }

    unsigned char C = info[0];
    int N = info[1];

    if(C != CONTROL1) {
        printf("\nRECEIVEPACKET WITHOUT C RIGHT\n");
        return -1;
    }
    if(N != seqNumber) {
        printf("\nRECEIVEPACKET WRONG SEQNUMBER\n");
        return -1;
    }

    int L2 = info[2];
    int L1 = info[3];
    K = 256 * L2 + L1;

    *buffer = (unsigned char*) malloc(K);
    memcpy((*buffer), (info + 4), K);

    free(info);

    return K;
}
// Link layer protocol implementation

#include "link_layer.h"
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7e


// Receiver
#define RECEIVER 1
#define UA_A 0x01
#define UA_C 0x07

// Transmitter
#define TRANSMITTER 0
#define SET_A 0x03
#define SET_C 0x03

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} StateMachine;

int fd, alarmCount = 0;
extern LinkLayer l;
extern unsigned int alarmEnabled;
extern int maxNTries;

void alarmHandler(int signal)   {
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

void sendFrame(unsigned char a, unsigned char ctr, int triggerAlarm){
    int controlSize = 5;
    unsigned char control[controlSize];
    control[0] = FLAG;
    control[1] = a;
    control[2] = ctr;
    control[3] = control[1] ^ control[2];
    control[4] = FLAG;
    int res = write(fd, control, controlSize);

    if(res < 0){
        printf("\nERROR WRITING FRAME\n");
        return ;
    }

    if(triggerAlarm){
        alarm(l.timeout);
    }
}

void receiveFrame(int senderStatus, unsigned char ctr){
    StateMachine state = START;
    int res;
    unsigned char byte, byte_A;

    while(state != STOP){
        res = read(fd, &byte, 1);
        if(res < 0){
            printf("\nREADING ERROR\n");
            return ;
        }

        switch(state){
            case START:
                if(byte==FLAG)
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(senderStatus == TRANSMITTER){
                    if(byte == SET_A){
                        state = A_RCV;
                        byte_A = byte;
                    }
                    else if(byte == FLAG)
                        state = FLAG_RCV;
                    else state = START;
                }
                else if(senderStatus== RECEIVER){
                    if(byte == UA_A){
                        state = A_RCV;
                        byte_A = byte;
                    }
                    else if(byte == FLAG)
                        state = FLAG_RCV;
                    else state = START;
                }
                break;
            case A_RCV:
                if(byte == ctr)
                    state = C_RCV;
                else if(byte == FLAG)
                    state = FLAG_RCV;
                else state = START;
                break;
            case C_RCV:
                if(byte == (byte_A ^ ctr))
                    state = BCC_OK;
                else if(byte == FLAG)
                    state = FLAG_RCV;
                else state = START;
                break;
            case BCC_OK:
                if(byte == FLAG){
                    state = STOP;
                    printf("\nRECEIVED FRAME\n");
                }
                else state = START;
                break;
            default:
            break;
        }
    } 
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(connectionParameters.serialPort);
        return -1;
    }

    signal(SIGALRM, alarmHandler);

    struct termios oldtio;
    struct termios newtio;
    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }
    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = l.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    // Set input mode
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 1 char is received
    // cleaning the line and activating the settings for the port
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    if(l.role == 0){
        sendFrame(SET_A, SET_C, TRUE);                            // in this case the transmitter is sending a SET frame and then receiving a UA frame
        receiveFrame(RECEIVER, UA_C);
    }
    else if(l.role == 1){
        receiveFrame(TRANSMITTER, SET_C);                         // in this case the receiver is receiving a SET frame and then sending a UA frame
        sendFrame(UA_A, UA_C, FALSE);
    } 

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
// Link layer protocol implementation

#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define packetSize 256

#define FLAG 0x7E
#define ESCAPE 0x7D
#define FLAG_PTR 0x5E
#define ESCAPE_PTR 0x5D

#define DISC_C 0x0B

#define RR_C(R) (R%2?0b10000101:0b00000101)
#define REJ_C(R) (R%2?0b10000001:0b00000001)
#define DATA_C(S) (S%2?0b01000000:0b00000000)

// Transmitter
#define SET_A 0x03
#define SET_C 0x03

// Receiver
#define UA_A 0x01
#define UA_C 0x07

struct termios oldtio;
struct termios newtio;
LinkLayer l;
int fd, alarmCount = 0, alarmEnabled = FALSE;
unsigned char buf[512];

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA_RCV,
    ESC_RCV,
    BCC2_OK,
    END_RCV,
    REJ_RCV
} StateMachine;

typedef struct {
    StateMachine current;
    unsigned char adr;
    unsigned char ctr;
    unsigned char bcc;
    unsigned char *data;
    unsigned int dataSize;
}   Frame;

Frame frame;

void alarm_handler();
int buildFrame(unsigned char* buffer, unsigned char address, unsigned char control);
void stateMachine(unsigned char byte, Frame *frame);

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    l = connectionParameters;
    if((fd = open(l.serialPort, O_RDWR | O_NOCTTY)) < 0){
        printf("\nERROR IN SERIALPORT\n");
        return -1;
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1){
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = l.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer 
    newtio.c_cc[VMIN] = 0;  // Blocking read until 1 chars received  
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        printf("\nERROR IN TCSETATTR\n");
        return -1;
    }

    if(l.role == LlTx){
        int recUA = 0;
        frame.current = START;
        alarmCount = 0;
        while(recUA == FALSE && alarmCount < l.nRetransmissions) {
            alarm(l.timeout);
            alarmEnabled = TRUE;
            if(alarmCount > 0) printf("\nTIME UP %d", alarmCount);
            int bufSize = buildFrame(buf, SET_A, SET_C);
            write(fd, buf, bufSize);
            printf("\nLLOPEN -- SET FRAME SENT\n");
            while(recUA == FALSE && alarmEnabled == TRUE){
                int readSize = read(fd, buf, packetSize);
                if(readSize < 0){
                    printf("\nERROR READING IN LLOPEN ON ROLE TX\n");
                    return -1;
                }
                for(unsigned int i = 0; i < readSize && recUA == FALSE; ++i){
                    stateMachine(buf[i], &frame);
                    if(frame.current == END_RCV && frame.adr == SET_A && frame.ctr == UA_C)
                        recUA = TRUE;
                }
            } 
        }
        if(recUA == TRUE) printf("\nLLOPEN RECEIVED UA\n");
        return 1;
    }
    else if(l.role == LlRx){
        alarmCount = 0;
        frame.current = START;
        int recSET = FALSE;

        while(recSET == FALSE){
            int readSize = read(fd, buf, packetSize);
            if(readSize < 0){
                printf("\nERROR READING IN LLOPEN ON ROLE RX\n");
                return -1;
            }
            for(unsigned int i = 0; i < readSize && recSET == FALSE; ++i){
                stateMachine(buf[i], &frame);
                if(frame.current == END_RCV && frame.adr == SET_A && frame.ctr == SET_C)
                    recSET = TRUE;
            }
        }
        if(recSET == TRUE)  printf("\nLLOPEN RECEIVED SET");

        int frameSize = buildFrame(buf, SET_A, UA_C);
        write(fd, buf, frameSize);
        printf("\nLLOPEN -- UA FRAME SENT\n");
        return 1;
    }

    return fd;
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

void alarm_handler() {
    alarmEnabled = FALSE;
    alarmCount++;
}

int buildFrame(unsigned char* buf, unsigned char adr, unsigned char ctr){
    buf[0] = FLAG;
    buf[1] = adr;
    buf[2] = ctr;
    buf[3] = adr ^ ctr;
    buf[4] = FLAG;
    return 5;
}

void stateMachine(unsigned char byte, Frame *frame){
    switch(frame->current){
        case REJ_RCV:
        case END_RCV:
            frame->current = START;
        case START:
            if(byte == FLAG) frame->current = FLAG_RCV;
            break;
        case FLAG_RCV:
            frame->dataSize = 0;
            if(byte == SET_A || byte == UA_A){
                frame->current = A_RCV;
                frame->adr = byte;
            }
            else if(byte == FLAG){

            }
            else {
                frame->current = START;
            }
            break;
        case A_RCV:
            if(byte == UA_C || byte == SET_C || byte == DISC_C || byte == REJ_C(0) || byte == REJ_C(1) || byte == DATA_C(0) || byte == DATA_C(1) || byte == RR_C(0) || byte == RR_C(1)){
                frame->current = C_RCV;
                frame->ctr = byte;
                frame->bcc = frame->adr ^ frame->ctr;
            }
            else if(byte == FLAG) frame->current = FLAG_RCV;
            else frame->current = START;
            break;
        case C_RCV:
            if(byte == frame->bcc) frame->current = BCC_OK;
            else if(byte == FLAG) frame->current = FLAG;
            else frame->current = START;
            break;
        case BCC_OK:
            if(byte == FLAG){
                if(frame->ctr == DATA_C(0) || frame->ctr == DATA_C(1)) frame->current = FLAG_RCV;
                else frame->current = END_RCV;
            }
            else if(frame->ctr == DATA_C(0) || frame->ctr == DATA_C(1)){
                if(frame->data != NULL){
                    frame->dataSize = 0;
                    if(byte == ESCAPE){
                        frame->current = ESC_RCV;
                        frame->bcc = 0;
                    }
                    else {
                        frame->data[frame->dataSize++] = byte;
                        frame->bcc = byte;
                        frame->current = DATA_RCV;
                    }
                }
            }
            else frame->current = START;
            break;
        case DATA_RCV:
            if(byte == ESCAPE) frame->current = ESC_RCV;
            else if(byte == FLAG) frame->current = REJ_RCV;
            else if(byte == frame->bcc) frame->current = BCC2_OK;
            else{
                frame->data[frame->dataSize++] = byte;
                frame->bcc ^= byte;
            }
            break;
        case ESC_RCV:
            if(byte == FLAG) frame->current = REJ_RCV;
            else if(byte == FLAG_PTR){
                if(frame->bcc = FLAG) frame->current = BCC2_OK;
                else{
                    frame->bcc ^= FLAG;
                    frame->data[frame->dataSize++] = FLAG;
                    frame->current = DATA_RCV;
                }
            }
            else if(byte == ESCAPE_PTR){
                if(frame->bcc == ESCAPE) frame->current = BCC2_OK;
                else{
                    frame->bcc ^= ESCAPE;
                    frame->data[frame->dataSize++] = ESCAPE;
                    frame->current = DATA_RCV;
                }
            }
            else frame->current = START;
            break;
        case BCC2_OK:
            if(byte == FLAG) frame->current = END_RCV;
            else if(byte == 0){
                frame->data[frame->dataSize++] = frame->bcc;
                frame->bcc = 0;
            }
            else if(byte == ESCAPE){
                frame->data[frame->dataSize++] = frame->bcc;
                frame->bcc = 0;
                frame->current = ESC_RCV;
            }
            else{
                frame->data[frame->dataSize++] = frame->bcc;
                frame->data[frame->dataSize++] = byte;
                frame->bcc = byte;
                frame->bcc = DATA_RCV;
            }
            break;
    }
}
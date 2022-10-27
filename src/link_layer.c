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
int fd, alarmCount = 0, alarmEnabled = FALSE, DISC_Received = FALSE;
unsigned char buf[512];
unsigned char dataFlag = 0;

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
int buildDataFrame(unsigned char* frame, const unsigned char* data, unsigned int dataSize, unsigned char adr, unsigned char ctr);
int byteStuffing(const unsigned char* src, int size, unsigned char* dest, unsigned char *bcc);

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    l = connectionParameters;
    if((fd = open(l.serialPort, O_RDWR | O_NOCTTY)) < 0){
        printf("\nERROR -- SERIALPORT\n");
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
        printf("\nERROR -- IN TCSETATTR\n");
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
            printf("\nLLOPEN SET FRAME SENT\n");
            while(recUA == FALSE && alarmEnabled == TRUE){
                int readSize = read(fd, buf, packetSize);
                if(readSize < 0){
                    printf("\nERROR -- READING IN LLOPEN ON ROLE TX\n");
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
                printf("\nERROR -- READING IN LLOPEN ON ROLE RX\n");
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
        printf("\nLLOPEN UA FRAME SENT\n");
        return 1;
    }

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char buffer[bufSize*2+100];
    int frameSize = buildDataFrame(buffer, buf, bufSize, SET_A, DATA_C(dataFlag));

    for(int i=0; i<frameSize ;){
        int writeSize = write(fd, buffer+i, frameSize-i);
        if(writeSize == -1){
            printf("\nERROR -- WRITING IN LLWRITE\n");
            return -1;
        }
        i += writeSize;
    }

    int receivePacket = FALSE, resend = FALSE, nRetransmissions = 0;
    frame.data = NULL;

    alarmEnabled = TRUE;
    alarm(l.timeout);

    while(receivePacket == FALSE){
        if(alarmEnabled == FALSE){
            resend = TRUE;
            alarmEnabled = TRUE;
            alarm(l.timeout);
        }
        if(resend == TRUE){
            if(nRetransmissions == l.nRetransmissions){
                printf("\nERROR -- EXCEED RETRANSMISSION LIMIT\n");
                return -1;
            }
            for(int i=0; i<frameSize ;){
                int writeSize = write(fd, buffer+i, frameSize-i);
                if(writeSize == -1){
                    printf("\nERROR -- WRITING IN LLWRITE\n");
                    return -1;
                }
                i += writeSize;
            }
            resend = FALSE;
            nRetransmissions++;
        }
        int readSize = read(fd, buf, packetSize);
        if(readSize < 0){
            printf("\nERROR -- READING NOT POSSIBLE\n");
            return -1;
        }

        for(int i=0; i<readSize && receivePacket == FALSE ;++i){
            stateMachine(buf[i], &frame);
            if(frame.current == END_RCV){
                if(frame.adr == SET_A && frame.ctr == RR_C(dataFlag)){
                    receivePacket = TRUE;
                    break;
                }
                if(frame.adr == SET_A && frame.ctr == REJ_C(dataFlag)){
                    resend = TRUE;
                    break;
                }
            }
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    int receivePacket = FALSE;
    frame.data = packet;

    while(receivePacket == FALSE){
        int readSize = read (fd, buf, packetSize);
        if(readSize < 0){
            printf("\nERROR -- READING ERROR IN LLREAD\n");
            return -1;
        }

        for(int i=0; i<readSize && receivePacket==FALSE ;++i){
            stateMachine(buf[i], &frame);

            if(frame.current == REJ_RCV && frame.adr == SET_A){
                int frameSize = buildFrame(buf, SET_A, (frame.ctr == DATA_C(0) ? REJ_C(0) : REJ_C(1)));
                write(fd, buf, frameSize);
                printf("\nLLREAD SENT REJ\n");
            }   

            if(frame.current == END_RCV && frame.adr == SET_A && frame.ctr == SET_C){
                int frameSize = buildFrame(buf, SET_A, UA_C);
                write(fd, buf, frameSize);
                printf("\nLLREAD SENT UA\n");
            }

            if(frame.current == END_RCV && frame.adr == SET_A){
                if(frame.ctr == DATA_C(0)){
                    int frameSize = buildFrame(buf, SET_A, RR_C(0));
                    write(fd, buf, frameSize);;
                    printf("\nLLREAD SENT RR %d\n", dataFlag);
                    return frame.dataSize;
                }
                else if(frame.ctr == DATA_C(1)){
                    int frameSize = buildFrame(buf, SET_A, RR_C(1));
                    write(fd, buf, frameSize);;
                    printf("\nLLREAD SENT RR %d\n", dataFlag);
                    return frame.dataSize;
                }
                else{
                    int frameSize = buildFrame(buf, SET_A, RR_C(0));
                    write(fd, buf, frameSize);;
                    printf("\nLLREAD SENT RR %d\n", dataFlag);
                }
            }

            if(frame.ctr == DISC_C){
                DISC_Received = TRUE;
                printf("\nLLREAD RECEIVED DISC\n");
                return -1;
            }
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    signal(SIGALRM, alarm_handler);
    alarmCount = 0;

    if(l.role == LlTx){
        int DISC_Received = FALSE;
        while(DISC_Received == FALSE && alarmCount < l.nRetransmissions){
            alarm(l.timeout);
            alarmEnabled = TRUE;
            if(alarmCount > 0) printf("\nERROR -- TIMED OUT\n");
            int size = buildFrame(buf, SET_A, DISC_C);
            write(fd, buf, size);
            printf("\nLLCLOSE SENT DISC\n");

            while(alarmEnabled == TRUE && DISC_Received == FALSE){
                int bytesRead = read(fd, buf, packetSize);
                if(bytesRead < 0){
                    printf("\nERROR -- READING IN LLCLOSE LLTX SIDE\n");
                    return -1;
                }
                for(int i=0; i<bytesRead && DISC_Received==FALSE ; ++i){
                    stateMachine(buf[i], &frame);
                    if(frame.current == END_RCV && frame.adr == SET_A && frame.ctr == DISC_C)
                        DISC_Received = TRUE;
                }
            }
        }
        if(DISC_Received == TRUE) printf("\nLLCLOSE RECEIVED DISC\n");
        int frameSize = buildFrame(buf, SET_A, UA_C);
        write(fd, buf, frameSize);
        printf("\nLLCLOSE SENT UA\nGOODBYE\n");
        sleep(1);
    }
    else {
        while(DISC_Received == FALSE){
            int bytesRead = read(fd, buf, packetSize);
            if(bytesRead < 0){
                printf("\nERROR -- READING IN LLCLOSE LLTX SIDE\n");
                return -1;
            }
            for(int i=0; i<bytesRead && DISC_Received == FALSE ;++i){
                stateMachine(buf[i], &frame);
                if(frame.current == END_RCV && frame.adr == SET_A && frame.ctr == DISC_C)
                    DISC_Received = TRUE;
            }
        }

        if(DISC_Received == TRUE) printf("\nLLCLOSE RECEIVED DISC\n");
        int frameSize = buildFrame(buf, SET_A, DISC_C);
        write(fd, buf, frameSize);
        printf("\nLLCLOSE SENT DISC\n");

        int recUA = FALSE;

        while(recUA == FALSE){
            int bytesRead = read(fd, buf, packetSize);
            if(bytesRead < 0){
                printf("\nERROR -- READING IN LLCLOSE LLRX SIDE\n");
                return -1;
            }
            for(int i=0;i<bytesRead && recUA == FALSE ; ++i){
                stateMachine(buf[i], &frame);
                if(frame.current == END_RCV && frame.adr == SET_A && frame.ctr == UA_C)
                    recUA = TRUE;
            }
        }

        if(recUA == TRUE) printf("\nLLCLOSE RECEIVED UA\n");
    }

    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
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
                if(frame->bcc == FLAG) frame->current = BCC2_OK;
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

int buildDataFrame(unsigned char* frame, const unsigned char* data, unsigned int dataSize, unsigned char adr, unsigned char ctr){
    frame[0] = FLAG;
    frame[1] = adr;
    frame[2] = ctr;
    frame[3] = adr ^ ctr;

    int offset;
    unsigned char bcc = 0;
    for(int i=0; i<dataSize ; ++i)
        offset += byteStuffing(data+i, 1, frame+offset+4, &bcc);
    
    offset += byteStuffing(&bcc, 1, frame+offset+4, NULL);
    frame[4+offset] = FLAG;
    return 5 + offset;
}

int byteStuffing(const unsigned char* src, int size, unsigned char* dest, unsigned char *bcc){
    int destSize= 0;

    for(int i=0; i<size; i++){
        if(bcc != NULL){
            *bcc ^= src[i];
        }
        
        if(src[i] == ESCAPE) {
			dest[destSize++] = ESCAPE;
			dest[destSize++] = ESCAPE_PTR;
			break;
		}

		else if(src[i] ==  FLAG) {
			dest[destSize++] = ESCAPE;
			dest[destSize++] = FLAG_PTR;
			break;
		}else{
            dest[destSize++] = src[i];
        }
    }

    return destSize;
}
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
#define _POSIX_SOURCE 1
#define packetSize 256

#define FLAG (0x7e)
#define ESCAPE (0x7d)
#define FLAG_PTR (0x5e)
#define ESCAPE_PTR (0x5d)

#define DISC_C (0x0b)

#define RR_C(R) (R%2?0b10000101:0b00000101)
#define REJ_C(R) (R%2?0b10000001:0b00000001)
#define DATA_C(S) (S%2?0b01000000:0b00000000)

// Transmitter
#define SET_A (0x03)
#define SET_C (0x03)

// Receiver
#define UA_A (0x01)
#define UA_C (0x07)

struct termios oldtio;
struct termios newtio;
LinkLayer l;
int fd, alarmCount = 0, alarmEnabled = FALSE, Disc_Received = FALSE;
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
} state;

typedef struct {
    state current;
    unsigned char adr;
    unsigned char ctrl;
    unsigned char bcc;
    unsigned char *data;
    unsigned int data_size;
} Frame;

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
    fd = open(l.serialPort, O_RDWR | O_NOCTTY);
    int result = FALSE;

    if (fd < 0){
        printf("\n ERROR -- l.serialPort");
        return -1;
    }

    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1){
        printf("\n ERROR -- In tcsetattr\n");
        return -1;
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
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        printf("\n ERROR -- In tcsetattr\n");
        return -1;
    }

    if(l.role == LlTx){
        int receivedUA = 0;
        frame.current=START;
        alarmCount = 0;
        while(alarmCount<l.nRetransmissions && receivedUA == FALSE){
            alarm(l.timeout);
            alarmEnabled = TRUE;
            if(alarmCount > 0){
                printf("\nERROR -- Timed out\n");
            }
            int size = buildFrame(buf,SET_A,SET_C);
            printf("\nLlopen SET Sent\n");
            write(fd,buf,size);

            while(alarmEnabled == TRUE && receivedUA == FALSE){
                int bytes_read = read(fd, buf, packetSize);
                if(bytes_read < 0)
                    return -1;
                for(unsigned int i = 0;i < bytes_read && receivedUA == FALSE; ++i){
                    stateMachine(buf[i], &frame);
                    if(frame.current == END_RCV && frame.adr == SET_A && frame.ctrl == UA_C)
                        receivedUA=1;
                }
            }
        }
        if(receivedUA){
            printf("\nLlopen UA Received\n");
        } 
        return 1;
    }
    else if (l.role == LlRx){
        alarmCount = 0;
        
        frame.current = START;
        int receivedSET = FALSE;
            while(receivedSET == FALSE){
                int bytes_read = read(fd, buf, packetSize);
                if(bytes_read < 0)
                    return -1;
                for(unsigned int i = 0;i < bytes_read && receivedSET == FALSE; ++i){
                    stateMachine(buf[i], &frame);
                    if(frame.current == END_RCV && frame.adr == SET_A && frame.ctrl == SET_C)
                        receivedSET = TRUE;
                }
            }
            if(receivedSET == TRUE){
                printf("\nLlopen: SET Received\n");
            } 
            int frameSize = buildFrame(buf, SET_A, UA_C);
            write(fd,buf,frameSize);
            printf("\nLlopen UA Sent\n");
            return 1;
        }
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){

    unsigned char Impbuffer[bufSize*2+100];
    int frameSize = buildDataFrame(Impbuffer, buf, bufSize, SET_A, DATA_C(dataFlag));
    
    for(unsigned int sent = 0;sent < frameSize;){
        int ret = write(fd,Impbuffer+sent,frameSize-sent);
        if(ret == -1){
            return -1;
        }
        sent += ret;
    }

    int receivedPacket = FALSE, resend = FALSE, retransmissions = 0;
    frame.data=NULL;
    
    alarmEnabled = TRUE;
    alarm(l.timeout);
    while(receivedPacket == FALSE){
        if(alarmEnabled == FALSE){
            resend = TRUE;
            alarmEnabled = TRUE;
            alarm(l.timeout);
        }
        if(resend == TRUE){
            if(retransmissions == l.nRetransmissions){
                printf("\nERROR -- Exceeded retransmission limit\n");
                return -1;
            }
            for(unsigned int sent = 0; sent < frameSize ;){
                int ret=write(fd, Impbuffer + sent , frameSize - sent);
                if(ret == -1){
                    return -1;
                }   
                sent += ret;
            }
            resend = FALSE;
            retransmissions++;
        }
        int bytes_read = read(fd, buf, packetSize);
        if(bytes_read < 0){
            return -1;
        }  
        for(unsigned int i = 0;i < bytes_read && receivedPacket == FALSE; ++i){ 
            stateMachine(buf[i], &frame);
            if(frame.current == END_RCV){
                if(frame.adr == SET_A && frame.ctrl == RR_C(dataFlag)){
                    receivedPacket = TRUE;
                    break;
                }
                if(frame.adr == SET_A && frame.ctrl == REJ_C(dataFlag)){
                    resend=1;
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
int llread(unsigned char *packet){
    int receivedPacket = 0;
    frame.data = packet;
    while(receivedPacket == FALSE){
        int bytes_read = read(fd, buf, packetSize);
        if(bytes_read<0)
            return -1;
        for(unsigned int i = 0;i < bytes_read && receivedPacket == FALSE; ++i){
            stateMachine(buf[i], &frame);

            if(frame.current == REJ_RCV && frame.adr == SET_A){
                int frameSize = buildFrame(buf,SET_A,(frame.ctrl == DATA_C(0) ? REJ_C(0) : REJ_C(1)));
                write(fd, buf, frameSize);
                printf("\nLlread REJ Sent\n");
            }
            if(frame.current == END_RCV && frame.adr == SET_A && frame.ctrl == SET_C){
                int frameSize = buildFrame(buf, SET_A, UA_C);
                write(fd, buf, frameSize);
                printf("\nLlread UA Sent\n");
            }
            if(frame.current == END_RCV && frame.adr == SET_A){
                if(frame.ctrl == DATA_C(0)){
                    int frameSize = buildFrame(buf, SET_A, RR_C(0));
                    write(fd, buf, frameSize);
                    printf("\nLlread RR Sent\n");
                    return frame.data_size;
                }
                else if(frame.ctrl == DATA_C(1)){
                    int frameSize = buildFrame(buf, SET_A, RR_C(1));
                    write(fd, buf, frameSize);
                    printf("\nLlread RR Sent\n");
                    return frame.data_size;
                }
                else{
                    int frameSize = buildFrame(buf, SET_A, RR_C(0));
                    write(fd, buf, frameSize);
                    printf("\nLlread RRSent\n");
                }
            }
            if(frame.ctrl == DISC_C) {
                Disc_Received = TRUE;
                printf("\n ERROR -- Llread: Received DISC.\n");
                return -1;
            }
        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics){
    signal(SIGALRM, alarm_handler);
    alarmCount=0;

    if(l.role == LlTx) {
        int DISC_Receiver = 0;
        
        while(alarmCount<l.nRetransmissions && DISC_Receiver == FALSE){
            alarm(l.timeout);
            alarmEnabled = TRUE;
            if(alarmCount > 0){
                printf("\nERROR -- Timed out\n");
            }
            int size = buildFrame(buf,SET_A,DISC_C);
            printf("\nLlclose: Sent DISC\n");
            write(fd,buf,size);
            while(alarmEnabled == TRUE && DISC_Receiver == FALSE){
                int bytes_read = read(fd, buf, packetSize);
                if(bytes_read < 0)
                    return -1;
                for(unsigned int i = 0;i < bytes_read && DISC_Receiver == FALSE;++i){
                    stateMachine(buf[i],&frame);
                    if(frame.current == END_RCV && frame.adr == SET_A && frame.ctrl == DISC_C)
                        DISC_Receiver=1;
                }
            }
        }
        if(DISC_Receiver == TRUE){
            printf("\nLlclose: Received DISC\n");
        } 
        int frameSize = buildFrame(buf, SET_A, UA_C);
        write(fd, buf, frameSize);
        printf("\nLlclose: Sent UA\n");
        sleep(1);

    } else {
        while(Disc_Received == FALSE){
            int bytes_read = read(fd, buf, packetSize);
            if(bytes_read < 0)
                return -1;
            for(unsigned int i = 0;i < bytes_read && Disc_Received == FALSE;++i){
                stateMachine(buf[i], &frame);
                if(frame.current == END_RCV && frame.adr == SET_A && frame.ctrl == DISC_C)
                    Disc_Received = TRUE;
            }
        }
        if( Disc_Received == TRUE){
            printf("\nLlclose: Received DISC\n");
        } 
        int frameSize = buildFrame(buf, SET_A, DISC_C);
        write(fd, buf, frameSize); 
        printf("\nLlclose: Sent DISC\n");

        int receivedUA = FALSE;
        while( receivedUA == FALSE){
            int bytes_read = read(fd ,buf, packetSize);
            if(bytes_read<0)
                return -1;
            for(unsigned int i = 0;i < bytes_read && receivedUA == FALSE; ++i){
                stateMachine(buf[i],&frame);
                if(frame.current == END_RCV && frame.adr == SET_A && frame.ctrl == UA_C)
                    receivedUA = TRUE;
            }
        }
        if(receivedUA == TRUE){
            printf("\nLlclose: Received UA\n");
        } 
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        printf("\n ERROR -- In tcsetattr\n");
        return -1;
    }

    close(fd);
    return 1;
}

void alarm_handler() {
    alarmEnabled = FALSE;
    alarmCount++;
}

int buildFrame(unsigned char* buffer, unsigned char address, unsigned char control){
    
    buffer[0] = FLAG;
    buffer[1] = address;
    buffer[2] = control;
    buffer[3] = address ^ control;
    buffer[4] = FLAG;
    return 5;
}

int buildDataFrame(unsigned char* frame, const unsigned char* data, unsigned int dataSize, unsigned char adr, unsigned char ctr){
    frame[0] = FLAG;
    frame[1] = adr;
    frame[2] = ctr;
    frame[3] = adr ^ ctr;
    int offset = 0;
    unsigned char bcc = 0;
    for(unsigned int i = 0;i < dataSize;++i){
        offset += byteStuffing(data + i, 1, frame + offset + 4, &bcc);
    }
    offset += byteStuffing(&bcc, 1, frame + offset + 4 ,NULL);
    frame[4 + offset]=FLAG;
    return 5 + offset;
}

int byteStuffing(const unsigned char* src, int size, unsigned char* dest, unsigned char *bcc){
    int destSize = 0;

    for(int i = 0; i < size; i++){
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

void stateMachine(unsigned char byte, Frame *frame){
    switch (frame -> current){
        case REJ_RCV:
        case END_RCV:
            frame -> current=START;
        case START:
            if(byte == FLAG){
                frame ->current = FLAG_RCV;
            }break;
        case FLAG_RCV:
            frame->data_size = 0;
            if(byte == SET_A || byte == UA_A){
                frame->current=A_RCV;
                frame->adr=byte;
            }else if(byte == FLAG){
                //nothing happens
            }else{
                frame = START;
            }
            break;
        case A_RCV:
            if(byte == UA_C || byte == SET_C || byte == DISC_C || byte == REJ_C(0) || byte == REJ_C(1) || byte == DATA_C(0) || byte == DATA_C(1)|| byte == RR_C(0) || byte == RR_C(1)){
                frame->current = C_RCV;
                frame->ctrl = byte;
                frame->bcc = frame->adr ^ frame->ctrl;
            }
            else if (byte == FLAG){
                frame->current = FLAG_RCV;
            }else {
                frame->current = START;
            }
            break;    
        case C_RCV:
            if(byte == frame->bcc){
                frame->current = BCC_OK;
            }
            else if (byte == FLAG){
                frame->current = FLAG_RCV;
            }else{
                frame->current = START;
            }
            break;
        case BCC_OK:
            if(byte==FLAG){
                if(frame->ctrl == DATA_C(0) || frame->ctrl == DATA_C(0) ){
                    frame->current = FLAG_RCV;
                }else{
                    frame->current = END_RCV;
                }
            }else if( frame->ctrl == DATA_C(0) || frame->ctrl == DATA_C(1)){
                if(frame->data != NULL){
                    frame->data_size = 0;
                    if(byte == ESCAPE){
                        frame->current = ESC_RCV;
                        frame->bcc = 0;
                    }else{
                        frame->data[frame->data_size++] = byte;
                        frame->bcc = byte;
                        frame->current = DATA_RCV;
                    }
                }
            }else{
                frame->current = START;
            }
            break;
        case DATA_RCV:
            if(byte == ESCAPE){
                frame->current = ESC_RCV;
            }else if(byte == FLAG){
                frame->current = REJ_RCV;
            }else if(byte == frame->bcc){
                frame->current = BCC2_OK;
            }else{
                frame->data[frame->data_size++] = byte;
                frame->bcc ^= byte;
            }
            break;
        case ESC_RCV:
            if(byte == FLAG){
                frame ->current = REJ_RCV; 
            }else if(byte ==FLAG_PTR){
                if(frame->bcc == FLAG){
                    frame->current = BCC2_OK;
                }else{
                    frame->bcc ^= FLAG;
                    frame->data[frame->data_size++] = FLAG;
                    frame->current = DATA_RCV;
                }
            }else if(byte == ESCAPE_PTR){
                if(frame->bcc == ESCAPE){
                    frame->current = BCC2_OK;
                }else{
                    frame->bcc ^= ESCAPE;
                    frame->data[frame->data_size++] = ESCAPE;
                    frame->current = DATA_RCV;
                }
            }else{
                frame->current = START;
            }
            break;
        case BCC2_OK:
            if(byte == FLAG){
                frame->current = END_RCV;
            }else if( byte == 0){
                frame->data[frame->data_size++] = frame->bcc;
                frame->bcc = 0;
            }else if(byte == ESCAPE){
                frame->data[frame->data_size++] = frame->bcc;    
                frame->bcc = 0;
                frame->current = ESC_RCV;
            }
            else{
                frame->data[frame->data_size++] = frame->bcc;
                frame->data[frame->data_size++] = byte;
                frame->bcc = byte;
                frame->current = DATA_RCV;
            }
            break;
    }
}
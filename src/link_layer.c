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
#define ESCAPE 0x7D
#define FLAG_PTR 0x5E
#define ESCAPE_PTR 0x5D
#define RR_CONTROL0 0x05
#define RR_CONTROL1 0x85

#define REJ_CONTROL0 0x01
#define REJ_CONTROL1 0x81

#define RR_POSITIVE_ACK 0x05
#define REJ_NEGATIVE_ACK 0x01

#define Info 1
#define Setup 0

//I frame
#define CONTROL0 0x00
#define CONTROL1 0x40


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
extern int maxNTries, sequenceNumber;

void alarmHandler(int signal);
void sendFrame(unsigned char a, unsigned char ctr, int triggerAlarm);
void receiveFrame(int senderStatus, unsigned char ctr);
unsigned char *byteStuffing(unsigned char *frame, unsigned int *length);
void receiveRREJ(int fd, unsigned char rr_control, unsigned char rej_control, unsigned char *frame, unsigned int frameSize);
unsigned char *byteDestuffing(unsigned char *data, unsigned int *size);
unsigned char calculateBCC2(const unsigned char *buffer, unsigned int size);

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
int llwrite(const unsigned char *buf, int bufSize){
    unsigned int totalLength = 6 + bufSize;
    unsigned char IFrame[totalLength], BCC2, BBC;
    int maxRetransmissions = l.nRetransmissions;

    IFrame[0] = FLAG;
    IFrame[1] = SET_A;
    if (sequenceNumber == 0) {
        IFrame[2] = CONTROL0;
    } 
    else if (sequenceNumber == 1) {
        IFrame[2] = CONTROL1;
    }

    IFrame[3] = IFrame[1] ^ IFrame[2];

    int i;
    IFrame[4] = buf[0];
    BCC2 = IFrame[4];
    for (i = 5; i < bufSize + 4; i++) {
        IFrame[i] = buf[i - 4];
        BCC2 = BCC2 ^ IFrame[i];
    }

    IFrame[totalLength - 2] = BCC2;
    IFrame[totalLength - 1] = FLAG;

    unsigned char *stuffedFrame = byteStuffing(IFrame, &totalLength);

    BBC = stuffedFrame[totalLength -2];
    int res = write(fd, stuffedFrame, totalLength);
    printf("\nSENT FRAME\n");

    stuffedFrame[totalLength - 2] = BBC;

    alarm(l.timeout);

    if(sequenceNumber == 0){
        receiveRREJ(fd, RR_CONTROL1, REJ_CONTROL0, stuffedFrame, totalLength);
        sequenceNumber = 1;
    }
    else if(sequenceNumber == 1){
        receiveRREJ(fd, RR_CONTROL0, REJ_CONTROL1, stuffedFrame, totalLength);
        sequenceNumber = 0;
    }

    free(stuffedFrame);

    l.nRetransmissions = maxRetransmissions;
    return res;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    StateMachine state = START;
    unsigned char byte, controlByte;

    unsigned int size = 0;
    unsigned char *dbcc = (unsigned char *)malloc(size);
    packet = (unsigned char *)malloc(0);
    unsigned char *destuffed;

    while (state != STOP) {
        int res = read(fd, &byte, 1);

        if(res < 0){
            printf("\nERROR IN READING\n");
        }
        else if(res > 0){
            switch (state)  {
            case START:
                if(byte == FLAG)
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == SET_A){
                    state = A_RCV;
                }
                else if(state == FLAG)
                    state = FLAG_RCV;
                else state = START;
                break;
            case A_RCV:
                if(byte == CONTROL0 && sequenceNumber == 0){
                    controlByte = CONTROL0;
                    state = C_RCV;
                }
                else if(byte == CONTROL1 && sequenceNumber == 1){
                    controlByte = CONTROL1;
                    state = C_RCV;
                } 
                else if(byte == FLAG)
                    state = FLAG_RCV;
                else state = START;
                break;
            case C_RCV:
                if(byte == (SET_A ^ controlByte)){
                    state = BCC_OK;
                }
                else if(byte == FLAG){
                    state = FLAG_RCV;
                }
                else state = START;
                break;
            case BCC_OK:
                if(byte == FLAG){

                    destuffed = byteDestuffing(dbcc, &size);

                    if(!calculateBCC2(destuffed, size)){
                        if(sequenceNumber == 0) sendFrame(UA_A, REJ_CONTROL0, FALSE);
                        else if(sequenceNumber == 1) sendFrame(UA_A, REJ_CONTROL1, FALSE);

                        size = 0;
                        dbcc = (unsigned char*) realloc(dbcc, size);
                        state = START;  
                        break;           
                    } else {
                        state = STOP;
                    }
                }   else {
                        dbcc = (unsigned char*) realloc(dbcc, ++size);
                        dbcc[size-1] = byte;
                }
				break;
            default:
                break;
            }
        }
    }

	packet = (unsigned char*) realloc(packet, --size);

	for(int i=0; i<size ;i++)
		packet[i] = destuffed[i];

	if (sequenceNumber == 0) {
		sendFrame(UA_A, RR_CONTROL1, FALSE);
		sequenceNumber = 1;
  	} 
  	else if (sequenceNumber == 1) {
		sendFrame(UA_A, RR_CONTROL0, FALSE);
		sequenceNumber = 0;
  }

  free(dbcc);
  free(destuffed);
  return size;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if(l.role == TRANSMITTER){
        sendFrame(SET_A, 0x0B, TRUE);
        receiveFrame(RECEIVER, 0x0B);
        sendFrame(SET_A, UA_C, FALSE);
    }
    else if(l.role == RECEIVER){
        receiveFrame(TRANSMITTER, 0x0B);
        sendFrame(UA_A, 0x0B, TRUE);
        receiveFrame(TRANSMITTER, UA_C);
    }

    return 1;
}

void alarmHandler(int signal){
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

unsigned char *byteStuffing(unsigned char *frame, unsigned int *length){
    unsigned char *stuffedFrame = (unsigned char *) malloc(*length);
    unsigned int finalLenght = *length;

    int j=0;
    stuffedFrame[j++] = FLAG;

    for(int i=1; i< *length - 1; i++){
        if(frame[i] == FLAG){
            stuffedFrame = (unsigned char *) realloc(stuffedFrame, ++finalLenght);
            stuffedFrame[j] = ESCAPE;
            stuffedFrame[++j] = FLAG_PTR;
            j++;
            continue;
        }else if(frame[i] == ESCAPE){
            stuffedFrame = (unsigned char *) realloc(stuffedFrame, ++finalLenght);
            stuffedFrame[j] = ESCAPE;
            stuffedFrame[++j] = ESCAPE_PTR;
            j++;
            continue;
        }else{
            stuffedFrame[j++] = frame[i];
        }     
    }

    stuffedFrame[j] = FLAG;

    *length=finalLenght;
    return stuffedFrame;
}

void receiveRREJ(int fd, unsigned char rr_control, unsigned char rej_control, unsigned char *frame, unsigned int frameSize){

    StateMachine receive = START;
    int res;
    unsigned char byte, controlByte;
    int retransmit = TRUE;

    while (receive != STOP) {
        if (retransmit) {
            if (l.nRetransmissions == 0) {
                printf("No more retransmissions, leaving.\n");
                return ;
            }
        res = write(fd, frame, frameSize);
            l.nRetransmissions--;
            alarm(l.timeout);
            retransmit = FALSE;
        }

        res = read(fd, &byte, 1);
        if (res < 0) {
        perror("\nRECEIVING ERROR\n");
        } else if (res == 0) {
            res = write(fd, frame, frameSize);
            res = read(fd, &byte, 1);
        }

        switch (receive) {
        case START:
            if (byte == FLAG)
            receive = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == UA_A) {
            receive = A_RCV;
            } else if (byte == FLAG) {
            receive = FLAG_RCV;
            } else
            receive = START;
            break;

        case A_RCV:
            if (byte == rr_control) {
            controlByte = rr_control;
            receive = C_RCV;
            alarm(0);
            } else if (byte == rej_control) {
            controlByte = rej_control;
            retransmit = TRUE;
            l.nRetransmissions++;
            } else if (byte == FLAG) {
            receive = FLAG_RCV;
            } else
            receive = START;
            break;

        case C_RCV:
            if (byte == (UA_A ^ controlByte)) {
            receive = BCC_OK;
            } else if (byte == FLAG) {
            receive = FLAG_RCV;
            } else {
            receive = START;
            }
            break;
        case BCC_OK:
            if (byte == FLAG) {
            receive = STOP;
            alarm(0);
            retransmit = FALSE;
            } else
            receive = START;
            break;

        default:
            break;
        }
    }
}

unsigned char *byteDestuffing(unsigned char *data, unsigned int *size){
    unsigned int finalSize = 0;
    unsigned char *newData = malloc(finalSize);

    for(int i = 0; i < *size; i++){
        if(data[i] == ESCAPE){
            if(data[i+1] == FLAG_PTR){
                newData = (unsigned char *) realloc(newData, ++finalSize);
                newData[finalSize - 1] = FLAG;
                i++;
            }else if(data[i+1] == ESCAPE_PTR){
                newData = (unsigned char *) realloc(newData, ++finalSize);
                newData[finalSize - 1] = ESCAPE;
                i++;
            }
        }else{
            newData = (unsigned char *)realloc(newData, ++finalSize);
            newData[finalSize - 1] = data[i];
        }
    }

    *size = finalSize;
    return newData;
}

unsigned char calculateBCC2(const unsigned char *buffer, unsigned int size){
    unsigned char BCC2;

    for(int i = 0; i < size; i++){
        BCC2 ^= buffer[i];
    }
    return BCC2;
}
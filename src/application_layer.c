// Application layer protocol implementation

#include "application_layer.h"
#include <link_layer.h>
#include <stdio.h>
#include <string.h>

unsigned char buf[256];

#define DATA_CTR 0x01
#define START_CTR 0x02
#define END_CTR 0x03

int getValues(unsigned char *buffer, unsigned char* type, unsigned char* length, unsigned char** value);

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

    if(l.role == LlTx){
        FILE* file = fopen(filename, "r");

        if(file == 0){
            printf("\nERROR OPENING FILE\n");
            return ;
        }

        fseek(file, 0, SEEK_END);               // seek to end of file
        long int fileSize = ftell(file);        // get current file pointer
        fseek(file, 0, SEEK_SET);               // seek back to beginning of file

        buf[0] = START_CTR;
        buf[1] = 0;
        buf[2] = sizeof(long);

        *(long*)(buf+3) = fileSize;
        llwrite(buf, 10);

        unsigned char fail = FALSE;
        unsigned long bytesPassed = 0;

        for(unsigned char i=0; bytesPassed<fileSize ;++i){
            int quantity = fileSize - bytesPassed < MAX_PAYLOAD_SIZE ? fileSize - bytesPassed : MAX_PAYLOAD_SIZE;
            unsigned long fileBytes = fread(buf+4, 1, quantity, file);

            if(fileBytes != quantity){
                printf("\nERROR -- FAILURE READING FILE");
                fail = TRUE;
                break;
            }

            buf[0] = DATA_CTR;
            buf[1] = i;
            buf[2] = fileBytes >> 8;
            buf[3] = fileBytes % 256;

            if(llwrite(buf, fileBytes+4) < 0){
                printf("\nERROR -- FAILURE WRITING USING LLWRITE\n");
                fail = TRUE;
                break;
            }
            else printf("\nSENT PACKET WITH %d BYTES\n", quantity);
            bytesPassed += quantity;
        }

        if(fail == FALSE){
            buf[0] = END_CTR;
            if(llwrite(buf, 1) < 0) printf("\nERROR -- FAILURE SENDING END PACKET\n");
        }
        fclose(file);
    }
    else if(l.role == LlRx){
        long int fileSize = 0, fileSizeReceived = 0;
        int bytesRead = llread(buf);
        unsigned char t, l, *v;

        if(buf[0] == START_CTR){
            int offset = 1;
            unsigned char EARLY = FALSE, lastNumber = 0;

            while(offset < bytesRead){
                offset += getValues(buf+offset, &t, &l, &v);
                if(t == 0){
                    fileSize = *((unsigned long*)v);
                    printf("\nFILESIZE %li\n", fileSize);
                }
            }

            FILE* file = fopen(filename, "w");
            if(file == FALSE){
                printf("\nERROR -- FAILURE TO OPEN FILE IN LLRX\n");
                return ;
            }

            while(fileSizeReceived < fileSize){
                int nBytes = llread(buf);
                if(nBytes < 1){
                    if(nBytes == -1) printf("\nERROR -- LLREAD\n"); 
                    else printf("\nERROR -- RECEIVED PACKET TOO SMALL\n"); 
                }

                if(buf[0] == END_CTR){
                    printf("\nERROR -- ENDED BEFORE EOF\n");
                    EARLY = TRUE;
                    break;
                }

                if(buf[0] == DATA_CTR){
                    if(nBytes < 5) printf("\nERROR -- RECEIVED PACKET TOO SMALL\n");
                    if(buf[1] != lastNumber) printf("\nERROR -- RECEIVED PACKET WITH WRONG SEQUENCE NUMBER\n");
                    else{
                        unsigned long size = buf[3] + buf[2] * 256;
                        if(size != nBytes-4) printf("\nERROR -- RECEIVED PACKET SIZE DOES NOT EQUAL HEADER\n");
                        fwrite(buf+4, 1, size, file);
                        fileSizeReceived += size;
                        lastNumber++;
                    }
                }
            }
            fclose(file);

            if(EARLY == TRUE){
                int nBytes = llread(buf);
                if(nBytes < 1){
                    if(nBytes == -1) printf("\nERROR -- LLREAD IN EARLY==TRUE\n");
                    else printf("\nERROR -- RECEIVED PACKET TOO SMALL IN EARLY==TRUE\n");
                }

                if(buf[0] != END_CTR) printf("\nERROR -- RECEIVED WRONG PACKET\n");
                else printf("\nRECEIVED END PACKET\n");
            }
        }
        else{
            printf("\nERROR -- DID NOT START WITH START PACKET\n");
        }
    }

    llclose(0);
    sleep(1);
}

int getValues(unsigned char *buffer, unsigned char* type, unsigned char* length, unsigned char** value){
    *type = buffer[0];
    *length = buffer[1];
    *value = buffer + 2;

    return 2 + *length;
}
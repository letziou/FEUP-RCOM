// Application layer protocol implementation

#include "application_layer.h"
#include <link_layer.h>
#include <stdio.h>
#include <string.h>

#define CONTROL_START (0x02)
#define CONTROL_END (0x03)
#define CONTROL_DATA (0x01)
#define TYPE_FILESIZE (0)


unsigned char buffer[256];

int getValues(unsigned char *src, unsigned char* type, unsigned char* length, unsigned char** value);
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer l;
	l.baudRate = baudRate;
	l.nRetransmissions = nTries;
	l.timeout = timeout;
	strcpy(l.serialPort,serialPort);


    if(strcmp(role,"tx") == 0) {
		l.role=LlTx;
		printf("\nI am a transmitter\n");
    }else if(strcmp(role,"rx") == 0) {
		l.role=LlRx;
		printf("\nI am a receiver\n");
    }else{
		printf("\n ERROR -- Unknown role, use 'tx' or 'rx'\n");
    }


	if(llopen(l) < 0){
		printf("\n ERROR -- Could not open connection\n");
		llclose(0);
		return -1;
	}

	if(l.role == LlTx) {
		FILE* file = fopen(filename,"r");

		if(file == FALSE){
			printf("\n ERROR -- Could not open file\n");
			return -1;
		}
  		

		fseek(file,0L,SEEK_END);
		long int fileSize = ftell(file);
		fseek(file,0,SEEK_SET);

		buffer[0] = CONTROL_START;
		buffer[1] = TYPE_FILESIZE;
		buffer[2] = sizeof(long);

		*(long*)(buffer+3) = fileSize;
		llwrite(buffer,10);

		unsigned char fFlag = 0;
		unsigned long bytesTransmitted = 0;

		for(unsigned char i = 0; bytesTransmitted < fileSize;++i){
			int quant = fileSize - bytesTransmitted < (MAX_PAYLOAD_SIZE)? fileSize - bytesTransmitted : (MAX_PAYLOAD_SIZE);
			unsigned long fileBytes = fread(buffer + 4, 1, quant, file);

			if(fileBytes != quant){
                printf("\n ERROR -- File read failure\n");
                fFlag=TRUE;
                break;
            }

			buffer[0] = CONTROL_DATA;
			buffer[1] = i;
			buffer[2] = fileBytes >> 8;
			buffer[3] = fileBytes % 256;

			if(llwrite(buffer, fileBytes + 4) < 0){
				printf("\n ERROR -- Failure on writing\n");
				fFlag = TRUE;
				break;
			}else{
				printf("Sent packet with this many bytes: %d\n", quant);
			}
			bytesTransmitted += quant; 
		}
		if(fFlag == FALSE){
			buffer[0] = CONTROL_END;

			if(llwrite(buffer, 1) < 0){
				printf("\n ERROR -- Failure on end of packet\n");
			}else{
				printf("\nSucess on send\n");
			}
		}fclose(file);

	}else if(l.role == LlRx){
		long int fileSize = 0 , fileSizeReceived = 0;
		int readBytes = llread(buffer);
		unsigned char t,l,*v;

		if(buffer[0] == CONTROL_START){
			int offset = 1;
			unsigned char fEarly = FALSE, lastNumber = 0;

			while(offset < readBytes){
				offset += getValues(buffer+offset, &t, &l, &v);
				if(t == TYPE_FILESIZE){
					fileSize = *((unsigned long*)v);
					printf("Filesize:%li\n",fileSize);
					}
			}
			

			FILE* file = fopen(filename, "w");
			if(file == FALSE) {		
				printf("\n ERROR -- Could not open file\n");
				return;
			} 

            while(fileSizeReceived < fileSize){
                int nBytes = llread(buffer);
                if(nBytes < 1){
                    if(nBytes == -1) printf("\n ERROR -- Error on llread\n");
                    else printf("\n ERROR -- Received a packet that is too small\n");
                }
                if(buffer[0] == CONTROL_END){
                    printf("\n ERROR -- Disconnected before EOF\n");
                    fEarly = TRUE;
                    break;
                }
                if(buffer[0] == CONTROL_DATA){
                    if(nBytes < 5) printf("\n ERROR -- Received a packet that is too small to be correct with nBytes: \n" + nBytes);

                    if(buffer[1] != lastNumber) printf("\n ERROR -- Received a packet that is too small to be correct\n");

                    else{
                        unsigned long size = buffer[3] + buffer[2] * 256;
                        if(size != nBytes-4) printf("\n ERROR -- Received packet size doesnt match header's info\n");

						fwrite(buffer + 4, 1, size, file);
						fileSizeReceived += size;
						lastNumber++;
					
						printf("Received packet with number: %d\n", buffer[1]);
                    }
                }
            }
			fclose(file);

			if(fEarly == FALSE){
                int nBytes = llread(buffer);
                if(nBytes < 1){
                    if(nBytes == -1){
						printf("\n ERROR -- On llread\n");
					}else printf("\n ERROR -- Received a packet that is too small to be correct\n");
                }
                if(buffer[0] != CONTROL_END){
                    printf("\n ERROR -- Received wrong type of packet\n");
                }else{
                    printf("\nReceived end packet\n");
                }
            }	
        }
        else printf("\nERROR -- Transmission didn't start with a start packet.\n");
		
		}
	llclose(0);
	sleep(1);
}

int getValues(unsigned char *src, unsigned char* type, unsigned char* length, unsigned char** value){
	*type = src[0];
	*length = src[1];
	*value = src + 2;

    return 2 + *length;
}
/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

extern "C"{
#include "StmFw_Upgrade.h"
};

SoftwareSerial mySerial(0, 1); // RX, TX
unsigned char * buff = (unsigned char *)malloc(sizeof(unsigned char)*100);

void serial_write(const unsigned char * buff, int len){
  Serial1.write((const char *)buff, len);
}

unsigned char serial_read(void){
  return Serial1.read();
}

void serial_reads(unsigned char* buff, int len){
  Serial1.readBytes((char *)buff, len);
}

void debug_write(const char * buff){
  Serial.write(buff);
}

void debug_read(char * data, int len){
  Serial.readBytes(data, len);
}


uart_handlers_t uartPtrs = {&serial_read, &serial_reads, &serial_write, &debug_write, &delay, &debug_read};


void setup() {
  load_uart_handlers(&uartPtrs);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  Serial1.begin(115200, SERIAL_8E1);
  Serial1.setTimeout(5000); //5 sec timeout

  strcpy((char *)buff,"Hello Josh");
  
}

void loop() { // run over and over
  initChip();
  int ver = cmdGet(); //this will update erase memory type
  unsigned long addr = 0x08000000;
  cmdEraseMemory(NULL);
  writeMemory(addr);
  //memVerify();
  while(1){
    Serial.println("STM FW Upgrade complete ");
    delay(2000);
  }
}


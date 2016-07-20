/* StmFw_Upgrade.c
Author: joshuathompson@sonicsensory.com
Date Created: 07/22/2016
*/


#include <inttypes.h>
#include <stdint.h>
#include "StmFw_Upgrade.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
        
error_status_t errorStatus = {0, 0, 0, 0};
int data_size = 4;

/* TODO: add string names to name attribute */
/* This struct isn't used at the moment but it can be used to verify the chip we are communicating with is correct */
//chip_ids_t chipIDs = {0,0,0,0,0,0,0,0,0};
/*
chipIDs.lowDensity.id = 0x412;
chipIDs.medDensity.id = 0x410;
chipIDs.highDensity.id = 0x414;
chipIDs.medDensityValLine.id = 0x420;
chipIDs.highDensityValLine.id = 0x428;
chipIDs.xlDensity.id = 0x430;
chipIDs.medDensityUlPwrLine.id = 0x416;
chipIDs.stm32F2xx.id = 0x411;
chipIDs.stm32F4xx.id = 0x413;
*/


/* Handle error */
void handle_error(error_status_type_t error_type){
  char error[100];
  switch(error_type){
    case NACK_ERROR:
      strcpy(error, "nack error\n");
      errorStatus.nack = 1;
      break;
    case TIMEOUT_ERROR:
      strcpy(error, "timeout error\n");
      errorStatus.timeout = 1;
      break;
    case DATA_LEN_ERROR:
      strcpy(error, "data len error\n");
      errorStatus.data_len = 1;
      break;
    case UNKNOWN_ERROR:
      strcpy(error, "unknown error\n");
      errorStatus.unknown = 1;
      break;
    default:
      strcpy(error, "unknown error\n");
      errorStatus.unknown = 1;
      break;
  }
  mdebug(0, error);
}

/* dummy time sleep */
void load_uart_handlers(uart_handlers_t * handlers){
  uartHandlers.uart_read_ptr = handlers->uart_read_ptr;
  uartHandlers.uart_reads_ptr = handlers->uart_reads_ptr;
  uartHandlers.uart_write_ptr = handlers->uart_write_ptr;
  uartHandlers.debug_write_ptr = handlers->debug_write_ptr;
  uartHandlers.delay_ptr = handlers->delay_ptr;
  uartHandlers.debug_reads_ptr = handlers->debug_reads_ptr;
}

void time_sleep(float time_sec){
    /* TODO: use real time function to sleep */
    uartHandlers.delay_ptr(time_sec*1000);
}

/* dummy uart set DTR function */
void uart_set_dtr(int state){
    /* TODO: use real uart set dtr function when ready */
}

/* dummy uart set RTS function */
void uart_set_rts(int state){
    /* TODO: use real uart set rts function when ready */
}

/* dummy uart read function */
unsigned char uart_read(void){
    unsigned char c;
    c = uartHandlers.uart_read_ptr();
    /* TODO: use real uart read function when ready */
    return c;
}

void uart_reads(unsigned char* buff, int len){
    /* TODO: use real uart read multiple char function when ready */
    uartHandlers.uart_reads_ptr(buff, len);
}

/* dummy uart write function */
void uart_write(const unsigned char * buff, int len){
    /* TODO: use real uart write function when ready */
    uartHandlers.uart_write_ptr((const char *)buff, len);
}

/* dummy uart timeout read */
int uart_timeout(void){
    /* TODO: how are we going to catch a uart timeout event? */
    return 0;
}

/* dummy get uart timeout param */
int get_uart_timeout(void){
  /* TODO: define function */
  return 0;
}

/* dummy set uart timeout */
void set_uart_timeout(int timeout){
  /* TODO: define function */
}

void debug_read(unsigned char * data, int len){
  uartHandlers.debug_reads_ptr((char *)data, len);
}

void mdebug(int level, const char * message){
    /* TODO: print error?
    if(QUIET >= level):
        print >> sys.stderr , message
    */
    uartHandlers.debug_write_ptr(message);
}


void open_uart(void){            
    /* TODO: open uart port with following params
     * parity: even
     * bytesize: 8
     * stopbits: 1
     * xonxoff: 0
     * rtscts: 0
     * timeout = 5
     * baudrate = 115200
     */
}

/* Wait for nack or ack response */
int wait_for_ask(void){
    /* wait for ask */
    time_sleep(0.2); //need this in here :(
    unsigned char ask = uart_read();
    /* TODO: what to do if timeout? */
    if(uart_timeout()){
        /* Timeout handler here */
    handle_error(TIMEOUT_ERROR);
        return 0;
    }
    else{
        if(ask == 0x79){
            /* ACK */
            return 1;
        }
        else{
            if(ask == 0x1F){
                /* NACK */
                /* TODO: how are we going to handle nack? */
                handle_error(NACK_ERROR);
                return 0;
            }
            else{
                /* Unknown response */
                /* TODO: how are we going to handle unknown response */
            	handle_error(UNKNOWN_ERROR);
                return 0;
            }
        }
    }
    return 0; /* should not get here */
}

void reset(void){
    uart_set_dtr(0);
    time_sleep(0.1);
    uart_set_dtr(1);
    time_sleep(0.5);
}

int initChip(void){
    /* Set boot */
    uart_set_rts(0);
    reset();
  unsigned char cmd[] = {0x7F};
    uart_write(cmd, 1);       /* tell bootloader to startup */
    time_sleep(0.1); //need this for some reason
    int ok = wait_for_ask();
    if(ok){
      mdebug(0, "init success\n");
    }
    else{
      mdebug(0, "init failed\n");
    }

    return ok;
}

void releaseChip(void){
    uart_set_rts(1);
    reset();
}

/* Generic cmd routine - use this for high level cmds */
int cmdGeneric(unsigned char c){
  unsigned char cmd[2];
  cmd[0] = c;
  cmd[1] = c ^ 0xFF;
  uart_write(cmd, 2); /* Control cmd byte */
  time_sleep(0.01);
  return wait_for_ask();
}

/* Get the number of cmd available and version num */
char cmdGet(void){
	mdebug(0, "sending get cmd\n");
    if(cmdGeneric(0x00)){
    	time_sleep(0.1); //need this for some reason
    	mdebug(0, "getting len\n");
        unsigned char len = uart_read();
        time_sleep(0.1); //need this for some reason
        mdebug(0, "getting ver\n");
        unsigned char ver = uart_read();

        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * (len+1));
        time_sleep(0.1); //need this for some reason
        uart_reads(buff, len);
        buff[len] = '\0';
        
        mdebug(0, "getting cmds\n");
        for(int i=0; i<len; i++){
            if(0x44 == buff[i]){
                extendedErase = 1;
            }
        }
        time_sleep(0.1);
        mdebug(0, "get last ack\n");
        int ok = wait_for_ask();

        if(ok){mdebug(0, "cmd get ok\n");}
        else {mdebug(0, "cmd get failed\n");}
        

        return ver;
    }
    else{
        /* TODO: what to do if receive nack? */
        handle_error(NACK_ERROR);
        return 0;
    }
}

/* Get Version of the ST Device */
unsigned char cmdGetVersion(void){
    if(cmdGeneric(0x01)){
        mdebug(10, "*** GetVersion command\n");
        unsigned char version = uart_read();
        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * 2);
        uart_reads(buff, 2);
        wait_for_ask();
        mdebug(10, "    Bootloader version\n");
        return version;
    }
    else{
        /* TODO: Handle nack ? */
    handle_error(NACK_ERROR);
        return 0;
    }
}

/* Get ID of ST Device */
int cmdGetID(void){
    if(cmdGeneric(0x02)){
        mdebug(10, "*** GetID command\n");
        char len = uart_read();
        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * (len+1));
        uart_reads(buff, len+1);
        wait_for_ask();
        int devID = (buff[0]<<8) + buff[1]; /* Not doing anything with the other bytes for now -- see AN3155 for info */
        return devID;
    }
    else{
        /* TODO: nack handle */
    handle_error(NACK_ERROR);
        return 0;
    }
}

void encode_addr(unsigned long addr, unsigned char * addr_buffer){
    unsigned char byte3 = (addr >> 0) & 0xFF;
    unsigned char byte2 = (addr >> 8) & 0xFF;
    unsigned char byte1 = (addr >> 16) & 0xFF;
    unsigned char byte0 = (addr >> 24) & 0xFF;
    unsigned char crc = byte0 ^ byte1 ^ byte2 ^ byte3;
    addr_buffer[0] = byte0; /*msb*/
    addr_buffer[1] = byte1;
    addr_buffer[2] = byte2;
    addr_buffer[3] = byte3;
    addr_buffer[4] = crc;
}


void cmdReadMemory(unsigned long addr, int lng, unsigned char * data){
    if(lng > data_size){
    handle_error(DATA_LEN_ERROR);
        return;
  } /* TODO: handler assert error */

    if(cmdGeneric(0x11)){
        mdebug(10, "*** ReadMemory command\n");
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer, 5);
        wait_for_ask();
        int n = (lng - 1) & 0xFF;
        unsigned char crc = n ^ 0xFF;
        unsigned char n_crc_buff[2];
        n_crc_buff[0] = n;
        n_crc_buff[1] = crc;
        uart_write(n_crc_buff, 2);
        wait_for_ask();
        /* read lng number of bytes */
        uart_reads(data, lng);
    }
    else{
        /* TODO: handle nack */
    handle_error(NACK_ERROR);
    }
}

void cmdGo(unsigned long addr){
    if(cmdGeneric(0x21)){
        mdebug(10, "*** Go command\n");
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer, 5);
        wait_for_ask();
    }
    else{
        /* TODO: handle nack */
    handle_error(NACK_ERROR);
    }
}

void cmdWriteMemory(unsigned long addr, unsigned char *data, int len){
  
    if(len > data_size){ 
        handle_error(DATA_LEN_ERROR);
        return;
    }
    if(cmdGeneric(0x31)){
        //mdebug(10, "*** Write memory command");
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer, 5);
        wait_for_ask();
        int lng = len-1; //(strlen((char *)data)-1) & 0xFF;
        //mdebug(10, "    %s bytes to write"); /* TODO: debug ? */
        unsigned char length_data[1];
        length_data[0] = lng;
        uart_write(length_data, 1); /* len really */
        unsigned char crc[] = {lng};
        unsigned char c[1];
        for(int i = 0; i <= lng; i++){
            c[0] = data[i];
            crc[0] = crc[0] ^ c[0];
            //uart_write(c, 1);
        }
        uart_write(data, len);
        uart_write(crc, 1);
        
        //mdebug(0, "last ack");
        wait_for_ask();
        //mdebug(10, "    Write memory done");
    }
    else{
        /* TODO: handle nack */
    mdebug(0, "write cmd not accepted\n");
    handle_error(NACK_ERROR);
    }
}

void cmdExtendedEraseMemory(void){
    if(cmdGeneric(0x44)){
        mdebug(10, "*** Extended Erase memory command\n");
        /* Global mass erase */
        unsigned char cmd[] = {0xFF};
        uart_write(cmd, 1);
        uart_write(cmd, 1);
        /* Checksum */
        cmd[0] = 0x00;
        uart_write(cmd, 1);
        int tmp = get_uart_timeout();
        //set_uart_timeout(30);
        time_sleep(10); //can take 30 sec
        /* Extended erase (0x44), can take ten seconds or more */
        wait_for_ask();
        set_uart_timeout(tmp); /* set back to default */
        mdebug(10, "    Extended Erase memory done");
    }
    else{
        /* TODO: handle nack */
        handle_error(NACK_ERROR);
    }
}


void cmdEraseMemory(unsigned char * sectors, int secLen){
    if(extendedErase){
        mdebug(0, "Cmd extended erase memory");
        cmdExtendedEraseMemory();
        return;
  }

    if(cmdGeneric(0x43)){
        mdebug(10, "*** Erase memory command");
        if(sectors == NULL){
            /* Global erase */
            unsigned char cmd[] = {0xFF};
            uart_write(cmd, 1);
            cmd[0] = 0x00;
            uart_write(cmd, 1);
        }
        else{
            /* Sectors erase */
            unsigned char cmd[1];
            cmd[0] = secLen -1; //(strlen((char *)sectors)-1) & 0xFF;
            uart_write(cmd, 1);
            unsigned char crc[] = {0xFF};
            unsigned char c[1];
            for(int i = 0; i<secLen; i++){
                c[0] = sectors[i];
                crc[0] = crc[0] ^ c[0];
                uart_write(c, 1);
            }
            uart_write(crc, 1);
            wait_for_ask();
            mdebug(10, "    Erase memory done");
    }
  }
  else{
        /* TODO: handle nack */
    mdebug(10, "*** Erase memory command failed");
    handle_error(NACK_ERROR);
  }
}



void cmdWriteProtect(unsigned char * sectors, int secLen){
    if(cmdGeneric(0x63)){
        mdebug(10, "*** Write protect command");
    unsigned char cmd[1];
    cmd[0] = secLen-1; //(strlen((char *)sectors)-1) & 0xFF;
        uart_write(cmd, 1);
        unsigned char crc[] = {0xFF};
    unsigned char c[1];
    int len = secLen;//strlen((char *)sectors);
        for(int i = 0; i<len; i++){
      c[0] = sectors[i];
            crc[0] = crc[0] ^ c[0];
            uart_write(c, 1);
    }
      
        uart_write(crc, 1);
        wait_for_ask();
        mdebug(10, "    Write protect done");
  }
    else{
    /* TODO: handle nack */
    handle_error(NACK_ERROR);
    }
}

void cmdWriteUnprotect(void){
    if(cmdGeneric(0x73)){
        mdebug(10, "*** Write Unprotect command");
        wait_for_ask();
        wait_for_ask();
        mdebug(10, "    Write Unprotect done");
  }
    else{
    /* TODO: handle nack */
    handle_error(NACK_ERROR);
    }
}

void cmdReadoutProtect(void){
    if(cmdGeneric(0x82)){
        mdebug(10, "*** Readout protect command");
        wait_for_ask();
        wait_for_ask();
        mdebug(10, "    Read protect done");
  }
    else{
    /* TODO: handle nack */
    handle_error(NACK_ERROR);
    }
}

void cmdReadoutUnprotect(void){
    if(cmdGeneric(0x92)){
        mdebug(10, "*** Readout Unprotect command");
        wait_for_ask();
        wait_for_ask();
        mdebug(10, "    Read Unprotect done");
  }
    else{
    /* TODO: handle nack */
    handle_error(NACK_ERROR);
    }
}

/* Complex commands section */

int readMemory(void){
    /* TODO: read all data back from stm32 and then compare to image stored in flash */
  /* return 1 if same 0 if not */
  return 0;
}

int writeMemory(unsigned long addr){
  /* TODO: write all data of image from flash to stm32 */
  /* return 1 if successful and 0 if not */
  unsigned char * data =  (unsigned char *)malloc(sizeof(unsigned char)*data_size);
  
  int complete = 0;
  while(!complete){
      mdebug(0, "@"); //informs python server to send data bytes
      debug_read(data,data_size);
      //mdebug(0, "Got data");
      if(data[0] == 's' && data[1] == 't' && data[2] == 'o' && data[3] == 'p' && data[4] == '!' && data[5] == '!'){
        //complete!
        return 0;
      }
      //mdebug(5, "Write bytes");
      cmdWriteMemory(addr, data, data_size);
      addr = addr + data_size;
  }
  mdebug(0, "Write Complete");

  return 0;
}




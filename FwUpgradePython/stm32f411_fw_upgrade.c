#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Verbose level */
#define QUIET                   20
/* these come from AN2606 */
#define CHIP_ID_MAX_STR         100
        
int extendedErase = 0;

typedef struct error_status_t{
	int nack;
	int timeout;
	int data_len;
	int unknown;
}error_status_t;

typedef enum error_status_type_t {NACK_ERROR=0, TIMEOUT_ERROR=1, DATA_LEN_ERROR=2, UNKNOWN_ERROR=3} error_status_type_t;


error_status_t errorStatus = {0, 0, 0, 0};

typedef struct chip_id_t{
    unsigned int id;
    unsigned char name[CHIP_ID_MAX_STR];
} chip_id_t;

typedef struct chip_ids_t {
    chip_id_t lowDensity;
    chip_id_t medDensity;
    chip_id_t highDensity;
    chip_id_t medDensityValLine;
    chip_id_t highDensityValLine;
    chip_id_t xlDensity;
    chip_id_t medDensityUlPwrLine;
    chip_id_t stm32F2xx;
    chip_id_t stm32F4xx;
} chip_ids_t;

/* TODO: add string names to name attribute */
/* This struct isn't used at the moment but it can be used to verify the chip we are communicating with is correct */
chip_ids_t chipIDs = {0,0,0,0,0,0,0,0,0};
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
	switch(error_type){
		case NACK_ERROR:
			errorStatus.nack = 1;
			break;
		case TIMEOUT_ERROR:
			errorStatus.timeout = 1;
			break;
		case DATA_LEN_ERROR:
			errorStatus.data_len = 1;
			break;
		case UNKNOWN_ERROR:
			errorStatus.unknown = 1;
			break;
		default:
			errorStatus.unknown = 1;
			break;
	}
}

/* dummy time sleep */
void time_sleep(float time_sec){
    /* TODO: use real time function to sleep */
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
char uart_read(void){
    char c;
    /* TODO: use real uart read function when ready */
    return c;
}

void uart_reads(unsigned char* buffer, int len){
    /* TODO: use real uart read multiple char function when ready */
    
}

/* dummy uart write function */
void uart_write(const unsigned char * buffer, int len){
    /* TODO: use real uart write function when ready */
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

void mdebug(int level, const char * message){
    /* TODO: print error?
    if(QUIET >= level):
        print >> sys.stderr , message
    */
}


void open(void){            
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
    char ask = uart_read();
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
    return wait_for_ask();
}

void releaseChip(void){
    uart_set_rts(1);
    reset();
}

/* Generic cmd routine - use this for high level cmds */
int cmdGeneric(unsigned char c){
	unsigned char cmd[1];
	cmd[0] = c;
    uart_write(cmd, 1);
	cmd[0] = c ^ 0xFF;
    uart_write(cmd, 1); /* Control cmd byte */
    return wait_for_ask();
}

/* Get the number of cmd available and version num */
char cmdGet(void){
    if(cmdGeneric(0x00)){
        mdebug(10, "*** Get command");
        unsigned char len = uart_read();
        unsigned char version = uart_read();
        mdebug(10, "    Bootloader version: "); /* TODO: print version num ? */
        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * len);
        uart_reads(buff, len);
        for(int i=0; i<len; i++){
            if(0x44 == buff[i]){
                extendedErase = 1;
            }
        }
        mdebug(10, "    Available commands"); /* TODO: show available cmds */
        wait_for_ask();
        return version;
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
        mdebug(10, "*** GetVersion command");
        unsigned char version = uart_read();
        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * 2);
        uart_reads(buff, 2);
        wait_for_ask();
        mdebug(10, "    Bootloader version");
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
        mdebug(10, "*** GetID command");
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
    if(!(lng <= 256)){
		handle_error(DATA_LEN_ERROR);
        return;
	} /* TODO: handler assert error */

    if(cmdGeneric(0x11)){
        mdebug(10, "*** ReadMemory command");
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
        mdebug(10, "*** Go command");
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

void cmdWriteMemory(unsigned long addr, unsigned char *data){
    if(!(strlen(data) <= 256)){ 
		handle_error(DATA_LEN_ERROR);
        return;
	}
	
    if(cmdGeneric(0x31)){
        mdebug(10, "*** Write memory command");
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer, 5);
        wait_for_ask();
        int lng = (strlen(data)-1) & 0xFF;
        mdebug(10, "    %s bytes to write"); /* TODO: debug ? */
        unsigned char length_data[1];
        length_data[0] = lng;
        uart_write(length_data, 1); /* len really */
        unsigned char crc[] = {0xFF};
        unsigned char c[1];
        for(int i = 0; i <= lng; i++){
            c[0] = data[i];
            crc[0] = crc[0] ^ c[0];
            uart_write(c, 1);
        }
        uart_write(crc, 1);

        wait_for_ask();
        mdebug(10, "    Write memory done");
    }
    else{
        /* TODO: handle nack */
		handle_error(NACK_ERROR);
    }
}

void cmdExtendedEraseMemory(void){
    if(cmdGeneric(0x44)){
        mdebug(10, "*** Extended Erase memory command");
        /* Global mass erase */
        unsigned char cmd[] = {0xFF};
        uart_write(cmd, 1);
        uart_write(cmd, 1);
        /* Checksum */
        cmd[0] = 0x00;
        uart_write(cmd, 1);
        int tmp = get_uart_timeout();
        set_uart_timeout(30);
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


void cmdEraseMemory(unsigned char * sectors){
    if(extendedErase){
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
			cmd[0] = (strlen(sectors)-1) & 0xFF;
            uart_write(cmd, 1);
            unsigned char crc[] = {0xFF};
			unsigned char c[1];
            for(int i = 0; i<strlen(sectors); i++){
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
		handle_error(NACK_ERROR);
	}
}



void cmdWriteProtect(unsigned char * sectors){
    if(cmdGeneric(0x63)){
        mdebug(10, "*** Write protect command");
		unsigned char cmd[1];
		cmd[0] = (strlen(sectors)-1) & 0xFF;
        uart_write(cmd, 1);
        unsigned char crc[] = {0xFF};
		unsigned char c[1];
		int len = strlen(sectors);
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

int writeMemory(void){
	/* TODO: write all data of image from flash to stm32 */
	/* return 1 if successful and 0 if not */
	return 0;
}
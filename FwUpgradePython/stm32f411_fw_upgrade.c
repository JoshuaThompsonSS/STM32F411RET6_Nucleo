/* Verbose level */
QUIET = 20

/* these come from AN2606 */
#define CHIP_ID_MAX_STR         100
        
int extendedErase = 0;
        
typedef struct chip_id_t{
    unsigned int id;
    unsigned char[CHIP_ID_MAX_STR];
} chip_id_t;

struct chip_ids_s {
    chip_id_t lowDensity;
    chip_id_t medDensity;
    chip_id_t highDensity;
    chip_id_t medDensityValLine;
    chip_id_t highDensityValLine;
    chip_id_t xlDensity;
    chip_id_t medDensityUlPwrLine;
    chip_id_t stm32F2xx;
    chip_id_t stm32F4xx;
} chip_ids_s;

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

char uart_reads(unsigned char* buffer, int len){
    /* TODO: use real uart read multiple char function when ready */
}

/* dummy uart write function */
void uart_write(const unsigned char * buffer){
    /* TODO: use real uart write function when ready */
}

/* dummy uart timeout read */
int uart_timeout(void){
    /* TODO: how are we going to catch a uart timeout event? */
    return 0;
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
void wait_for_ask(void){
    /* wait for ask */
    char ask = uart_read();
    /* TODO: what to do if timeout? */
    if(uart_timeout()){
        /* Timeout handler here */
    }
    else{
        if(ask == 0x79){
            /* ACK */
            return 1;
        }
        else{
            if ask == 0x1F{
                /* NACK */
                /* TODO: how are we going to handle nack? */
                return 0;
            }
            else{
                /* Unknown response */
                /* TODO: how are we going to handle unknown response */
            }
        }
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

    uart_write("\x7F")       /* tell bootloader to startup */
    return wait_for_ask();
}

void releaseChip(void){
    uart_set_rts(1);
    reset();
}

/* Generic cmd routine - use this for high level cmds */
int cmdGeneric(unsigned char cmd){
    uart_write(cmd);
    uart_write(cmd ^ 0xFF); /* Control cmd byte */
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
        unsigned char * dat = uart_reads(buff, len);
        for(int i=0; i<len; i++){
            if(0x44 == dat[i]){
                extendedErase = 1;
            }
        }
        mdebug(10, "    Available commands"); /* TODO: show available cmds */
        wait_for_ask();
        return version;
    }
    else{
        /* TODO: what to do if receive nack? */
        return 0;
    }
}

/* Get Version of the ST Device */
unsigned char cmdGetVersion(void){
    if(cmdGeneric(0x01)){
        mdebug(10, "*** GetVersion command")
        unsigned char version = uart_read();
        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * 2);
        uart_reads(buff, 2);
        wait_for_ask();
        mdebug(10, "    Bootloader version");
        return version;
    }
    else{
        /* TODO: Handle nack ? */
        return 0;
    }
}

/* Get ID of ST Device */
int cmdGetID(void){
    if(cmdGeneric(0x02)){
        mdebug(10, "*** GetID command")
        char len = uart_read();
        unsigned char * buff = (unsigned char *) malloc(sizeof(unsigned char) * (len+1));
        unsigned char * id = uart_reads(buff, len+1);
        wait_for_ask();
        int devID = (id[0]<<8) + id[1];
        return devID;
    }
    else{
        /* TODO: nack handle */
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


void cmdReadMemory(unsigned long addr, int lng, unsigned char * data):
    if(!(lng <= 256)){return 0;} /* TODO: handler assert error */

    if(cmdGeneric(0x11)){
        mdebug(10, "*** ReadMemory command");
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer);
        wait_for_ask();
        int n = (lng - 1) & 0xFF;
        unsigned char crc = n ^ 0xFF;
        unsigned char n_crc_buff[2];
        n_crc_buff[0] = n;
        n_crc_buff[1] = crc;
        uart_write(n_crc_buff);
        wait_for_ask();
        /* read lng number of bytes */
        uart_reads(data, lng);
    }
    else{
        /* TODO: handle nack */
    }
}

void cmdGo(unsigned long addr):
    if self.cmdGeneric(0x21){
        mdebug(10, "*** Go command")
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer);
        wait_for_ask();
    }
    else{
        /* TODO: handle nack */
    }

void cmdWriteMemory(unsigned long addr, unsigned char *data):
    if(!(strlen(data) <= 256)){ return 0;} /* TODO: handle assert */
    if(cmdGeneric(0x31)){
        mdebug(10, "*** Write memory command")
        unsigned char * addr_buffer = (unsigned char *) malloc(sizeof(unsigned char)*5);
        encode_addr(addr, addr_buffer); /* Get addr and crc buffer */
        uart_write(addr_buffer);
        wait_for_ask();
        int lng = (strlen(data)-1) & 0xFF
        mdebug(10, "    %s bytes to write"); /* TODO: debug ? */
        unsigned char length_data[1];
        length_data[0] = lng;
        uart_write(length_data) /* len really */
        unsigned char crc = 0xFF;
        unsigned char c[1];
        for(int i = 0; i <= lng; i++){
            c = data[i];
            crc = crc ^ c;
            uart_write(c);
        }
        c = crc;
        uart_write(c);

        wait_for_ask();
        mdebug(10, "    Write memory done");
    }
    else{
        /* TODO: handle nack */
    }

def cmdEraseMemory(self, sectors = None):
    if self.extended_erase:
        return cmd.cmdExtendedEraseMemory()

    if self.cmdGeneric(0x43):
        mdebug(10, "*** Erase memory command")
        if sectors is None:
            # Global erase
            self.sp.write(chr(0xFF))
            self.sp.write(chr(0x00))
        else:
            # Sectors erase
            self.sp.write(chr((len(sectors)-1) & 0xFF))
            crc = 0xFF
            for c in sectors:
                crc = crc ^ c
                self.sp.write(chr(c))
            self.sp.write(chr(crc))
        self._wait_for_ask("0x43 erasing failed")
        mdebug(10, "    Erase memory done")
    else:
        raise CmdException("Erase memory (0x43) failed")

void cmdExtendedEraseMemory(void):
    if(cmdGeneric(0x44)){
        mdebug(10, "*** Extended Erase memory command");
        /* Global mass erase */
        uart_write(0xFF);
        uart_write(0xFF);
        # Checksum
        self.sp.write(chr(0x00))
        tmp = self.sp.timeout
        self.sp.timeout = 30
        print "Extended erase (0x44), this can take ten seconds or more"
        self._wait_for_ask("0x44 erasing failed")
        self.sp.timeout = tmp
        mdebug(10, "    Extended Erase memory done");
    }
    else{
        /* TODO: handle nack */
    }
def cmdWriteProtect(self, sectors):
    if self.cmdGeneric(0x63):
        mdebug(10, "*** Write protect command")
        self.sp.write(chr((len(sectors)-1) & 0xFF))
        crc = 0xFF
        for c in sectors:
            crc = crc ^ c
            self.sp.write(chr(c))
        self.sp.write(chr(crc))
        self._wait_for_ask("0x63 write protect failed")
        mdebug(10, "    Write protect done")
    else:
        raise CmdException("Write Protect memory (0x63) failed")

def cmdWriteUnprotect(self):
    if self.cmdGeneric(0x73):
        mdebug(10, "*** Write Unprotect command")
        self._wait_for_ask("0x73 write unprotect failed")
        self._wait_for_ask("0x73 write unprotect 2 failed")
        mdebug(10, "    Write Unprotect done")
    else:
        raise CmdException("Write Unprotect (0x73) failed")

def cmdReadoutProtect(self):
    if self.cmdGeneric(0x82):
        mdebug(10, "*** Readout protect command")
        self._wait_for_ask("0x82 readout protect failed")
        self._wait_for_ask("0x82 readout protect 2 failed")
        mdebug(10, "    Read protect done")
    else:
        raise CmdException("Readout protect (0x82) failed")

def cmdReadoutUnprotect(self):
    if self.cmdGeneric(0x92):
        mdebug(10, "*** Readout Unprotect command")
        self._wait_for_ask("0x92 readout unprotect failed")
        self._wait_for_ask("0x92 readout unprotect 2 failed")
        mdebug(10, "    Read Unprotect done")
    else:
        raise CmdException("Readout unprotect (0x92) failed")


# Complex commands section

def readMemory(self, addr, lng):
    data = []
    if usepbar:
        widgets = ['Reading: ', Percentage(),', ', ETA(), ' ', Bar()]
        pbar = ProgressBar(widgets=widgets,maxval=lng, term_width=79).start()
    
    while lng > 256:
        if usepbar:
            pbar.update(pbar.maxval-lng)
        else:
            mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
        data = data + self.cmdReadMemory(addr, 256)
        addr = addr + 256
        lng = lng - 256
    if usepbar:
        pbar.update(pbar.maxval-lng)
        pbar.finish()
    else:
        mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
    data = data + self.cmdReadMemory(addr, lng)
    return data

def writeMemory(self, addr, data):
    lng = len(data)
    if usepbar:
        widgets = ['Writing: ', Percentage(),' ', ETA(), ' ', Bar()]
        pbar = ProgressBar(widgets=widgets, maxval=lng, term_width=79).start()
    
    offs = 0
    while lng > 256:
        if usepbar:
            pbar.update(pbar.maxval-lng)
        else:
            mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
        self.cmdWriteMemory(addr, data[offs:offs+256])
        offs = offs + 256
        addr = addr + 256
        lng = lng - 256
    if usepbar:
        pbar.update(pbar.maxval-lng)
        pbar.finish()
    else:
        mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
    self.cmdWriteMemory(addr, data[offs:offs+lng] + ([0xFF] * (256-lng)) )
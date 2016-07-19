/* stmfw_upgrade.h developed by jbthompson.eng@gmail.com
Date Created: 07/22/2016
*/

#ifndef STMFW_UPGRADE_h
#define STMFW_UPGRADE_h

#include "Arduino.h"

#endif // STMFW_UPGRADE_h


/* Verbose level */
#define QUIET                   20
/* these come from AN2606 */
#define CHIP_ID_MAX_STR         100

int extendedErase;


/* Typedefs */
typedef struct uart_handlers_t {
  unsigned char (*uart_read_ptr)(void);
  void (*uart_reads_ptr)(unsigned char* buff, int len);
  void (*uart_write_ptr)(const char* buff, int len);
  void (*debug_write_ptr)(const char * buff);
  void (*delay_ptr)(long unsigned int ms);
  void (*debug_reads_ptr)(char* data, int len);
} uart_handlers_t;

uart_handlers_t uartHandlers;
typedef struct error_status_t{
  int nack;
  int timeout;
  int data_len;
  int unknown;
}error_status_t;

typedef enum error_status_type_t {NACK_ERROR=0, TIMEOUT_ERROR=1, DATA_LEN_ERROR=2, UNKNOWN_ERROR=3} error_status_type_t;

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

/* Function Declarations */
void load_uart_handlers(uart_handlers_t * handlers);
void handle_error(error_status_type_t error_type);
void time_sleep(float time_sec);
void uart_set_dtr(int state);
void uart_set_rts(int state);
unsigned char uart_read(void);
void uart_reads(unsigned char* buff, int len);
void uart_write(const unsigned char * buff, int len);
int uart_timeout(void);
int get_uart_timeout(void);
void set_uart_timeout(int timeout);
void debug_read(unsigned char * data, int len);
void mdebug(int level, const char * message);
void open_uart(void);
int wait_for_ask(void);
void reset(void);
int initChip(void);
void releaseChip(void);
int cmdGeneric(unsigned char c);
char cmdGet(void);
unsigned char cmdGetVersion(void);
int cmdGetID(void);
void encode_addr(unsigned long addr, unsigned char * addr_buffer);
void cmdReadMemory(unsigned long addr, int lng, unsigned char * data);
void cmdGo(unsigned long addr);
void cmdWriteMemory(unsigned long addr, unsigned char *data, int len);
void cmdExtendedEraseMemory(void);
void cmdEraseMemory(unsigned char * sectors, int secLen);
void cmdWriteProtect(unsigned char * sectors, int secLen);
void cmdReadoutProtect(void);
void cmdReadoutUnprotect(void);
int readMemory(void);
int writeMemory(unsigned long addr);



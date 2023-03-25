#ifndef _HILA_H_
#define _HILA_H_

#include <stdint.h>

#define PRESCALER 41

#define RS485_BAUDRATE  57600
#define DIAG_BAUDRATE   28800

#define RS485_BUF_SIZE  32
#define TEST_BUF_SIZE   32
#define DIAG_BUF_SIZE   32

#define HILA_REG_SETPOINT_MIN 12.0
#define HILA_TEMPERATURE      25
#define HILA_TEMPERATURE_WAR  50
#define HILA_TEMPERATURE_ERR  55       

#define HILA_MK2_MODEL_NUM  "YLM-50-ASML\r"
#define HILA_MK2_FW_VERSION ""

// hila status register bit definitions
#define HILA_STA_CMD_BUFFER_OVF (1 << 0)    // 1
#define HILA_STA_OVERHEAT       (1 << 1)    // 2
#define HILA_STA_EMISSION       (1 << 2)    // 4
#define HILA_STA_BACK_REFLECT   (1 << 3)    // 8
#define HILA_STA_NOPOWER        (1 << 11)   // 2048
#define HILA_STA_MODULATION     (1 << 12)   // 4096
#define HILA_STA_PSU_FAILURE    (1 << 19)   // 524288
#define HILA_STA_LOW_TEMP       (1 << 24)   // 16777216
#define HILA_STA_SUPPLY_ALARM   (1 << 25)   // 33554432
#define HILA_STA_GND_LEAKAGE    (1 << 26)   // 67108864
#define HILA_STA_CRITICAL_ERR   (1 << 29)   // 536870912
#define HILA_STA_INTERLOCK      (1 << 30)   // 1073741824


// hila output threshold definitions
#define HILA_OUTPUT_LOW     0.5
#define HILA_OUTPUT_OFF     0.01
#define HILA_DIAG_DELAY_TICKS   10

// HILA power equation coefficients
#define HILA_COEFF_A    0.5
#define HILA_COEFF_B    0

#define HILA_DISCHARGE_TIME 25 // hila discharge time must be below 50 ms


#define HILA_ID  6


// MHCB2 - HILA communication functions
// extern void hila_rs485_initialize(uint32_t baud);
extern void hila_rs485_putc(char ch);
extern uint16_t hila_rs485_getc(void);
extern void hila_rs485_puts(char* text);

// HILA IO functions
// extern void hila_io_initialize(void);
// extern void hila_set_emission_led(uint8_t state);
// extern void hila_set_ready_led(uint8_t state);
// extern void hila_set_error_led(uint8_t state);
// extern void hila_set_rs485_direction(uint8_t state);
// extern uint8_t hila_get_vp24v_status(void); // might be either an analog reading or digital reading. Let's decide later.

extern void hila_initialize(void);
// extern void hila_rs485_cmd_handle(void);
extern void hila_rs485_cmd_execute(void);
// void hila_diag_cmd_handle(void);
extern void hila_diag_cmd_execute(void);
extern void hila_io_handle(void);

// hila test communication functions -->
extern void hila_test_cmd_execute(void);

// extern void _delay_setup(void);
// extern void _delay_ms(uint32_t duration);

extern int _write(int file, char *ptr, int len);

#endif
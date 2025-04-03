#include "hila.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "ftoa.h"
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
/*
CH232 converter orange --> PC11 and yellow --> PC10
*/


// HILA RS485 port stuff:
char rs485_cmd_buffer[RS485_BUF_SIZE];
char rs485_resp_buffer[RS485_BUF_SIZE];
uint8_t rs485_cmd_idx            = 0;
bool rs485_cmd_isOverflow        = false;
bool hila_cmd_isReceived         = false;
char rs485_command[TEST_BUF_SIZE];
char rs485_command_parameter[TEST_BUF_SIZE];

// HILA RS485 command with arguments:
char hila_cmd_argument[32];
uint8_t hila_cmd_arg_idx             = 0;

// HILA DIAG port stuff
char diag_cmd_buffer[DIAG_BUF_SIZE];
char diag_resp_buffer[DIAG_BUF_SIZE];
uint8_t diag_cmd_idx             = 0;
bool diag_cmd_isOverflow         = false;
bool diag_cmd_isReceived         = false;

// HILA TEST port stuff
char test_cmd_buffer[TEST_BUF_SIZE];
// char test_resp_buffer[DIAG_BUF_SIZE];
uint8_t test_cmd_idx             = 0;
bool test_cmd_isOverflow         = false;
bool test_cmd_isReceived         = false;
char test_command[TEST_BUF_SIZE];
char test_command_parameter_str[TEST_BUF_SIZE];
int test_command_parameter;

// HILA internal registers and variables
float hila_setpoint_register        = 0.0;
float reg_setpoint_min              = HILA_REG_SETPOINT_MIN;
uint32_t hila_status_register       = HILA_STA_NOPOWER;
float hila_output_power_register    = 0.0;
uint8_t hila_disch_time             = HILA_DISCHARGE_TIME;
uint8_t hila_laser_temp             = HILA_TEMPERATURE;
uint8_t hila_lifetime_eol           = 0;
bool hila_error_flag                = false;
bool hila_diag_isEnabled            = false;
int hila_diag_delay                 = -1;
int hila_sdc_counter                = 0;
int hila_critical_err_counter       = 0;


// MCU related variables
int timeout_counter = 0;
bool isTimedOut = false;


static void hila_sysclk_setup(void)
{
    // set sysclock to 84MHz using internal reference oscillator
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
}

static void hila_calc_output_power(void)
{
    float output_power;
    output_power = HILA_COEFF_A * hila_setpoint_register + HILA_COEFF_B;

    if ( output_power <= 0.0) {
        output_power = 0.0;
    }

    if (hila_lifetime_eol == 0) // no end of life error/warning
    {
        if (hila_status_register & HILA_STA_EMISSION)
        {
            hila_output_power_register = output_power;
        }
        else
        {
            hila_output_power_register = 0.0;
        }
    }
    else if (hila_lifetime_eol == 1) // end of life warning
    {
        if (hila_status_register & HILA_STA_EMISSION)
        {
            hila_output_power_register = output_power - (output_power * 0.20);
        }
        else
        {
            hila_output_power_register = 0.0;
        }
    }

    else if (hila_lifetime_eol == 2) // end of life error
    {
        if (hila_status_register & HILA_STA_EMISSION)
        {
            hila_output_power_register = output_power - (output_power * 0.35);
        }
        else
        {
            hila_output_power_register = 0.0;
        }
    }
    
}

static void sdc(char* response)
{
    char str[8];
    float setpoint = atof(rs485_command_parameter); // convert string argument into a floating point number
    if (setpoint == 0 ) {
        sprintf(response, "SDC: 0.0\r"); 
        hila_setpoint_register = 0.0;
    } else if (setpoint < reg_setpoint_min || setpoint > 100.0) {
        sprintf(response, "ERR: Argument out of range\r");
        
    } else {
        if (setpoint < 9.995)
        {
            hila_setpoint_register = ftoa_2dec(setpoint, str);
        }
        else
        {
            hila_setpoint_register = ftoa_1dec(setpoint, str);
        }
        sprintf(response, "SDC: %s\r", str); 
    }
    hila_sdc_counter++; 
    hila_calc_output_power();
}

static void rcs(char* response)
{
    char str[8];
    (void)ftoa_2dec(hila_setpoint_register, str); // float to ascii/string
    sprintf(response, "RCS: %s\r", str); // sprintf has some problem with %f that's why it is needed to convert float into string/ascii
}

static void emoff(char* response)
{
    hila_status_register &= ~HILA_STA_EMISSION;
    sprintf(response, "EMOFF\r");
}

static void emon(char* response)
{
    if (!((hila_status_register & HILA_STA_NOPOWER) && (hila_error_flag == true)))
    {
        hila_status_register |= HILA_STA_EMISSION;
    }

    sprintf(response, "EMON\r");
}

static void emod(char* response)
{
    if (!(hila_status_register & HILA_STA_EMISSION))
    {
        sprintf(response, "EMOD\r");
        hila_status_register |= HILA_STA_MODULATION;
    }
    else
    {
        sprintf(response, "ERR: Emission is ON!\r");
    }
}

static void dmod(char* response)
{
    if (!(hila_status_register & HILA_STA_EMISSION))
    {
        sprintf(response, "DMOD\r");
        hila_status_register &= ~HILA_STA_MODULATION;
    }
    else
    {
        sprintf(response, "ERR: Emission is ON!\r");
    }
}

static void rct(char* response)
{
    sprintf(response, "RCT: %d\r", hila_laser_temp);
}

static void ricdt(char* response)
{
    sprintf(response, "RICDT: %d\r", hila_disch_time);

    if (hila_disch_time > 50)
    {
        hila_status_register |= HILA_STA_CRITICAL_ERR; // status register critical error, discharge time is longer than it should be.
        hila_critical_err_counter++;
    }
    else
    {
        hila_status_register &= ~HILA_STA_CRITICAL_ERR; // discharge time is OK. normal operation
    }
}

static void rerr(char* response)
{
    sprintf(response, "RERR\r");
    hila_status_register &= ~(HILA_STA_OVERHEAT | HILA_STA_BACK_REFLECT | HILA_STA_LOW_TEMP | HILA_STA_SUPPLY_ALARM | HILA_STA_GND_LEAKAGE);
}

static void rmn(char* response)
{
    sprintf(response, HILA_MK2_MODEL_NUM);
}

static void rfv(char* response)
{
    sprintf(response, HILA_MK2_RFV);
}


static void rop(char* response)
{
    char str[8];
    if (hila_output_power_register < HILA_OUTPUT_OFF)
    {
        sprintf(response, "ROP: Off\r");
    }
    else if (hila_output_power_register < HILA_OUTPUT_LOW)
    {
        sprintf(response, "ROP: Low\r");
    }
    else
    {
        if (hila_output_power_register < 9.995f) 
        {
            (void)ftoa_2dec(hila_output_power_register, str);
        } 
        else 
        {
            (void)ftoa_1dec(hila_output_power_register, str);
        }
        sprintf(response, "ROP: %s\r", str);
    }
}

static void rpp(char* response)
{
    char str[8];
    if (hila_output_power_register < HILA_OUTPUT_OFF)
    {
        sprintf(response, "RPP: Off\r");
    }
    else if (hila_output_power_register < HILA_OUTPUT_LOW)
    {
        sprintf(response, "RPP: Low\r");
    }
    else
    {
        if (hila_output_power_register < 9.995f) 
        {
            (void)ftoa_2dec(hila_output_power_register, str);
        } 
        else 
        {
            (void)ftoa_1dec(hila_output_power_register, str);
        }
        sprintf(response, "RPP: %s\r", str);
    }
}

static void rnc(char* response)
{
    char str[8];
    (void)ftoa_1dec(reg_setpoint_min, str);
    sprintf(response, "RNC: %s\r", str); 
}

static void sta(char* response)
{
    sprintf(response, "STA: %d\r", (int)hila_status_register); 
}

static void dle(char* response)
{
    if (!(hila_status_register & HILA_STA_EMISSION))
    {
        sprintf(response, "DLE\r");
        // hila_status_register |= HILA_STA_MODULATION;
    }
    else
    {
        sprintf(response, "ERR: Emission ON!\r");
    }
}

static void dgm(char* response)
{
    if (!(hila_status_register & HILA_STA_EMISSION))
    {
        sprintf(response, "DGM\r");
        // hila_status_register |= HILA_STA_MODULATION;
    }
    else
    {
        sprintf(response, "ERR: Emission ON!\r");
    }
}

static void dec(char* response)
{
    if (!(hila_status_register & HILA_STA_EMISSION))
    {
        sprintf(response, "DEC\r");
        // hila_status_register |= HILA_STA_MODULATION;
    }
    else
    {
        sprintf(response, "ERR: Emission ON!\r");
    }
}

static void edc(char* response)
{
    if (!(hila_status_register & HILA_STA_EMISSION))
    {
        sprintf(response, "EDC\r");
        // hila_status_register |= HILA_STA_MODULATION;
    }
    else
    {
        sprintf(response, "ERR: Emission ON!\r");
    }
}

static void rmec(char* response)
{
    sprintf(response, "RMEC: 0\r");
    // if (!(hila_status_register & HILA_STA_EMISSION))
    // {
    //     sprintf(response, "RMEC: 0\r");
    //     // hila_status_register |= HILA_STA_MODULATION;
    // }
    // else
    // {
    //     sprintf(response, "ERR: Emission ON!\r");
    // }
}

static void ret(char* response)
{
    sprintf(response, "RET: %d\r", hila_sdc_counter); 
}

static void rec(char* response)
{
    sprintf(response, "REC: %d\r", hila_critical_err_counter); 
}

static void rsn(char* response)
{
    sprintf(response, "RSN: %s", HILA_MK2_SERIAL_NUM); 
}


static void bcmd(char* response)
{
    sprintf(response, "BCMD\r");
}


// diag functions
static void run(void)
{
    hila_diag_isEnabled = true;
}

static void stop(void)
{
    hila_diag_isEnabled = false;
}
static void hila_io_initialize(void)

{
    // enable clocks for gpios:
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);




    // configure gpios:
    // emission on/off control output:
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    // ready led control output:
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    // error led control output:
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    // rs485 drive enable output:
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    // hila power supply status input:
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    // stm32 nucleo on board led:
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    // set alternate function for PA9: USART1_TX and PA10: USART1_RX
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

    // set alternate mode as AF7
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);

}

static void hila_set_emission_led(uint8_t state)
{
    switch (state)
    {
    case 1:
        gpio_set(GPIOC, GPIO7);
        break;
    case 0:
        gpio_clear(GPIOC, GPIO7);
        break;
    default:
        break;
    }
}

static void hila_set_ready_led(uint8_t state)
{
    switch (state)
    {
    case 1:
        gpio_set(GPIOC, GPIO6);
        break;
    case 0:
        gpio_clear(GPIOC, GPIO6);
        break;
    default:
        break;
    }
}

static void hila_set_error_led(uint8_t state)
{
    switch (state)
    {
    case 1:
        gpio_set(GPIOC, GPIO8);
        break;
    case 0:
        gpio_clear(GPIOC, GPIO8);
        break;
    default:
        break;
    }
}

static void hila_set_rs485_direction(uint8_t state)
{
    switch (state)
    {
    case 1: // transmission enabled
        gpio_set(GPIOA, GPIO6);
        break;
    case 0: // reception enabled
        gpio_clear(GPIOA, GPIO6);
        break;
    default:
        break;
    }
}

static uint8_t hila_get_vp24v_status(void)
{
    return gpio_get(GPIOC, GPIO0);
}

// static void hila_set_on_board_led(uint8_t state)
// {
//     switch (state)
//     {
//     case 1: // on-board led is on
//         gpio_set(GPIOA, GPIO5);
//         break;
//     case 0: // led is off
//         gpio_clear(GPIOA, GPIO5);
//         break;
//     default:
//         break;
//     }
// }


static void hila_timeout_setup(void)
{
    
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_prescaler(TIM2, 41); // 1MHz timer freq
    timer_set_period(TIM2, 10000); // 1000 usec
    timer_disable_irq(TIM2, TIM_DIER_UIE);
}

static void hila_timeout_nvic(void)
{
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 4);
}


static void hila_diag_delay_timer_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM5);
    timer_set_prescaler(TIM5, 41999); // 1kHz timer freq
    timer_set_period(TIM5, 200); // sets the period to 200 ms. 100 ms ON and 100 ms OFF
    timer_enable_irq(TIM5, TIM_DIER_UIE);
    timer_enable_counter(TIM5);
}

static void hila_diag_delay_nvic(void)
{
    nvic_enable_irq(NVIC_TIM5_IRQ);
    nvic_set_priority(NVIC_TIM5_IRQ, 5);
}

static void hila_rs485_delay_setup()
{
    rcc_periph_clock_enable(RCC_TIM3);
    timer_set_prescaler(TIM3, 41); // 1MHz timer freq
}

void hila_rs485_delay(uint16_t us)
{
	timer_set_period(TIM3, us);
    timer_enable_counter(TIM3);
    while(!timer_get_flag(TIM3, TIM_SR_UIF));
    timer_clear_flag(TIM3, TIM_SR_UIF);
}

/**************SYSYTICK STARTS******************/
volatile uint32_t system_millis;

/* Called when systick fires */
void sys_tick_handler(void)
{
	system_millis++;
}

/* sleep for delay milliseconds */
static void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(84000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

/****************SYSTICK ENDS*************************************/ 

static void hila_rs485_initialize(uint32_t baud)
{
    // enable clock for usart1 peripheral
    rcc_periph_clock_enable(RCC_USART1);
    // set baudrate
    usart_set_baudrate(USART1, baud);
    // data bit:8, stop bit:1, flow control:none, parity:none
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_parity(USART1, USART_PARITY_NONE);
    // enable tx and rx 
    usart_set_mode(USART1, USART_MODE_TX_RX);
    // enable rx interrupt:
    usart_enable_rx_interrupt(USART1);

    // set direction to reception by setting DE low
    hila_set_rs485_direction(0);

    // enable usart module
    usart_enable(USART1);
}

static void hila_rs485_nvic_setup(void)
{
    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_set_priority(NVIC_USART1_IRQ, 1);
}

void hila_rs485_putc(char ch)
{
    usart_send_blocking(USART1, ch);
}

uint16_t hila_rs485_getc(void)
{
    return usart_recv_blocking(USART1);
}

void hila_rs485_puts(char *text)
{   
    uint8_t length;
    uint8_t text_idx;
    uint32_t usart_stat;

    length = strlen(text);
    msleep(1);
    hila_set_rs485_direction(1); // switch to transmitter mode
    for(int i = 0; i < 2000; i++);
    // // if it is the first byte that wait for TXE = 1 and switch the direction to transmitter
    // while(!usart_get_flag(USART1, USART_SR_TXE));
    // usart_send(USART1, text[0]);
    // send data until the byte before the last byte
    for(text_idx = 0; text_idx < length; text_idx++)
    {
        while(!usart_get_flag(USART1, USART_SR_TXE));
        usart_send(USART1, text[text_idx]);
    }
    // // if it is the last byte to send, send data enable TC interrupt and switch direction to receiver mode in transmission complete (TC) interrupt function.
    //while(!usart_get_flag(USART1, USART_SR_TXE));
    // usart_send(USART1, text[length-1]); 
    // USART1_CR1 |= USART_CR1_TCIE;

    do 
    {
        usart_stat = (USART1_SR &  USART_SR_TC);
    }
    while(usart_stat != USART_SR_TC);
    for(int i = 0; i < 2000; i++);
    
    hila_set_rs485_direction(0); // switch to transmitter mode
    USART1_SR &= ~(USART_SR_TC);
    // USART1_SR &= ~(1 << USART_SR_TC);
    
}

static void hila_diag_initialize(uint32_t baud)
{
    // enable clock for usart gpios
    rcc_periph_clock_enable(RCC_GPIOA);
    // set alternate function for PA2: USART1_TX and PA3: USART1_RX
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    // set alternate mode as AF7
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

    // enable clock for usart1 peripheral
    rcc_periph_clock_enable(RCC_USART2);
    // set baudrate
    usart_set_baudrate(USART2, baud);
    // data bit:8, stop bit:1, flow control:none, parity:none
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_parity(USART2, USART_PARITY_NONE);
    // enable tx and rx 
    usart_set_mode(USART2, USART_MODE_TX_RX);
    // enable rx interrupt
    usart_enable_rx_interrupt(USART2);
    // enable usart module
    usart_enable(USART2);
}

static void hila_diag_nvic_setup(void)
{
    nvic_enable_irq(NVIC_USART2_IRQ);
    nvic_set_priority(NVIC_USART2_IRQ, 2);
}

static void hila_diag_putc(char ch)
{
    usart_send_blocking(USART2, ch);
}


/*This function is redirect hila diag serial port to printf function*/
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART2, '\r');
			}
			usart_send_blocking(USART2, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

// static uint16_t hila_diag_getc(void)
// {
//     return usart_recv_blocking(USART2);
// }

static void hila_diag_puts(char* text)
{
    while(*text)
    {
        hila_diag_putc(*text);
        text++;
    }
}

// hila test port is for injecting and forcing errors and warnings related to HILA-MK2
// for commands refer to hila_test_cmd_execute() function
static void hila_test_initialize(void)
{
    // enable clock for usart gpios
    rcc_periph_clock_enable(RCC_GPIOC);
    // rcc_periph_clock_enable(RCC_GPIOB);
    // // set alternate function for PC10: USART3_TX and PC11: USART3_RX
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
    // gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    // set alternate mode as AF7
    gpio_set_af(GPIOC, GPIO_AF7, GPIO11);
    gpio_set_af(GPIOC, GPIO_AF7, GPIO10);
    // enable clock for usart3 peripheral
    rcc_periph_clock_enable(RCC_USART3);
    // set baudrate
    usart_set_baudrate(USART3, 115200);
    // data bit:8, stop bit:1, flow control:none, parity:none
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_parity(USART3, USART_PARITY_NONE);
    // enable tx and rx 
    usart_set_mode(USART3, USART_MODE_TX_RX);
    // enable rx interrupt:
    usart_enable_rx_interrupt(USART3);
    // enable usart module
    usart_enable(USART3);
}

static void hila_test_nvic_setup(void)
{
    nvic_enable_irq(NVIC_USART3_IRQ);
    nvic_set_priority(NVIC_USART3_IRQ, 3);
}

static void hila_test_putc(char ch)
{
    usart_send_blocking(USART3, ch);
}

// static uint16_t hila_diag_getc(void)
// {
//     return usart_recv_blocking(USART2);
// }

static void hila_test_puts(char* text)
{
    while(*text)
    {
        hila_test_putc(*text);
        text++;
    }
}

void hila_initialize(void)
{
    hila_sysclk_setup();
    hila_io_initialize();   
    hila_rs485_initialize(RS485_BAUDRATE);
    hila_diag_initialize(DIAG_BAUDRATE);
    hila_test_initialize();
    printf("HILA Emulator Diagnostics Port\r\n"); // to be removed
    hila_test_puts("HILA Emulator Test Port ---> ");
    hila_test_puts("This port is used to inject commands to force HILA errors\r\n");
    hila_diag_delay_timer_setup();
    hila_diag_delay_nvic();
    hila_rs485_nvic_setup();
    hila_diag_nvic_setup();
    hila_test_nvic_setup();
    hila_timeout_setup();    
    systick_setup();

    //MVUR
    timer_disable_counter(TIM2);
    timer_disable_irq(TIM2, TIM_DIER_UIE);

    hila_rs485_delay_setup();
    //MVUR

    hila_timeout_nvic();    
}

static void hila_rs485_cmd_buf_flush(void)
{
    memset((char*)rs485_cmd_buffer, '\0', RS485_BUF_SIZE);
}

static void hila_rs485_resp_buf_flush(void)
{
    for(uint8_t i = 0; i < RS485_BUF_SIZE; i++) {
        rs485_resp_buffer[i] = 0;
    }
    //memset(rs485_resp_buffer, '\0', RS485_BUF_SIZE);
}

static void hila_diag_resp_buf_flush(void)
{
    memset(diag_resp_buffer, '\0', DIAG_BUF_SIZE);
}

void hila_rs485_cmd_execute(void)
{
    char str[8];
    float p_out;
    float p_received_from_mhcb2;

    sscanf((char*)rs485_cmd_buffer, "%s %s", rs485_command, rs485_command_parameter);

    if (!strcmp((const char*)rs485_command, "SDC"))
    {
        // if (rs485_cmd_buffer[3] == 32)
        // {
        //     for(hila_cmd_arg_idx = 0; hila_cmd_arg_idx < strlen((const char*)rs485_cmd_buffer); hila_cmd_arg_idx++)
        //     {
        //         hila_cmd_argument[hila_cmd_arg_idx] = rs485_cmd_buffer[hila_cmd_arg_idx + 4];
        //     }
        // }
        sdc(rs485_resp_buffer); // call set diode current function
        if ((hila_status_register & HILA_STA_EMISSION) == HILA_STA_EMISSION)
        {
            p_received_from_mhcb2 = atof(rs485_command_parameter);
        
            if(p_received_from_mhcb2 > 0.0 ) {
                p_out = (0.54444 * p_received_from_mhcb2) + (-4.4444);
            } else {
                p_out = 0.0;
            }
            
            
            (void)ftoa_2dec(p_out, str);
            //printf("HILA-%d: %s W\r\n", HILA_ID, str);
            hila_diag_puts(str);
            hila_diag_putc('\r');
        }
    }
    else if (!strcmp((const char*)rs485_command, "RCS"))
    {
        rcs(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RMN"))
    {
        rmn(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RFV"))
    {
        rfv(rs485_resp_buffer);
    }    
    else if(!strcmp((const char*)rs485_command, "EMON"))
    {
        emon(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "EMOFF"))
    {
        emoff(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "ROP"))
    {
       rop(rs485_resp_buffer); 
    }
    else if(!strcmp((const char*)rs485_command, "RCT"))
    {
       rct(rs485_resp_buffer);
       
    }
    else if(!strcmp((const char*)rs485_command, "RERR"))
    {
        rerr(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "STA"))
    {
        sta(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "EMOD"))
    {
        emod(rs485_resp_buffer);  
    }
    else if(!strcmp((const char*)rs485_command, "DMOD"))
    {
        dmod(rs485_resp_buffer);
    }
    else if (!strcmp((const char*)rs485_command, "RCT"))
    {
        rct(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RERR"))
    {
        rerr(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RICDT"))
    {
        ricdt(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "DGM"))
    {
        dgm(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "DLE"))
    {
        dle(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RNC"))
    {
        rnc(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RPP"))
    {
        rpp(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RSN"))
    {
        rsn(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "REC"))
    {
        rec(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RET"))
    {
        ret(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "EDC"))
    {
        edc(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "DEC"))
    {
        dec(rs485_resp_buffer);
    }
    else if(!strcmp((const char*)rs485_command, "RMEC"))
    {
        rmec(rs485_resp_buffer);
    }

    else
    {
        bcmd(rs485_resp_buffer);
    }
    
    hila_rs485_puts(rs485_resp_buffer);
    
    rs485_cmd_idx = 0;
    rs485_cmd_buffer[0] = 0;
    rs485_cmd_isOverflow = false;

    hila_rs485_cmd_buf_flush(); // clear buffer
    hila_diag_resp_buf_flush();
    hila_rs485_resp_buf_flush();
}

void hila_diag_cmd_execute(void)
{
    if (!strcmp((const char*)diag_cmd_buffer, "RUN"))
    {
        run();
    }
    else if (!strcmp((const char*)diag_cmd_buffer, "STOP"))
    {
        stop();
    }

    hila_diag_puts(diag_resp_buffer);
    hila_diag_resp_buf_flush();
    diag_cmd_idx = 0;
    diag_cmd_buffer[0] = 0;
    diag_cmd_isOverflow = false; 
}


void hila_test_cmd_execute(void)
{
    sscanf((char*)test_cmd_buffer, "%s %s", test_command, test_command_parameter_str);
    test_command_parameter = atoi(test_command_parameter_str);

    if (!strcmp((const char*)test_command, "TEST"))
    {
        hila_diag_puts("Port is functional\r\n");
    }
    else if (!strcmp((const char*)test_command, "SICDT"))
    {
        hila_disch_time = test_command_parameter;
    }
    else if (!strcmp((const char*)test_command, "STA")) // Test error LED behavior for each error case!!!
    {
        if ((test_command_parameter != 2) && (test_command_parameter != 11) && (test_command_parameter != 12) && (test_command_parameter <= 31))
        {
            switch (test_command_parameter)
            {
                case 0:
                    hila_status_register |= HILA_STA_CMD_BUFFER_OVF;
                    hila_error_flag = true;
                    break;
                case 1:
                    hila_status_register |= HILA_STA_OVERHEAT;
                    hila_status_register &= ~HILA_STA_EMISSION;
                    hila_error_flag = true;
                    break;
                case 3:
                    hila_status_register |= HILA_STA_BACK_REFLECT;
                    hila_status_register &= ~HILA_STA_EMISSION;
                    hila_error_flag = true;
                    break;
                case 19:
                    hila_status_register |= HILA_STA_PSU_FAILURE;
                    hila_status_register &= ~HILA_STA_EMISSION;
                    hila_error_flag = true;
                    break;
                case 24:
                    hila_status_register |= HILA_STA_LOW_TEMP;
                    hila_status_register &= ~HILA_STA_EMISSION;
                    hila_error_flag = true;
                    break;
                case 25:
                    hila_status_register |= HILA_STA_PSU_FAILURE;
                    hila_status_register &= ~HILA_STA_EMISSION;
                    hila_error_flag = true;
                    break;
                case 26:
                    hila_status_register |= HILA_STA_GND_LEAKAGE;
                    hila_status_register &= ~HILA_STA_EMISSION;
                    hila_error_flag = true;
                    break;
                default:
                    break;
            }
        }
        else
        {
            hila_test_puts("Invalid parameter. Try again!\r\n");
        }
    }
    else if (!strcmp((const char*)test_command, "CLR")) // --> normally RERR from MHCB2 clears errors
    {
        switch (test_command_parameter)
        {
            case 0:
                hila_status_register &= ~HILA_STA_CMD_BUFFER_OVF;
                hila_error_flag = false;
                break;
            case 1:
                hila_status_register &= ~HILA_STA_OVERHEAT;
                hila_error_flag = false;
                break;
            case 3:
                hila_status_register &= ~HILA_STA_BACK_REFLECT;
                hila_error_flag = false;
                break;
            case 19:
                hila_status_register &= ~HILA_STA_PSU_FAILURE;
                hila_error_flag = false;
                break;
            case 24:
                hila_status_register &= ~HILA_STA_LOW_TEMP;
                hila_error_flag = false;
                break;
            case 25:
                hila_status_register &= ~HILA_STA_PSU_FAILURE;
                hila_error_flag = false;
                break;
            case 26:
                hila_status_register &= ~HILA_STA_GND_LEAKAGE;
                hila_error_flag = false;
                break;
            default:
                break;
        } 

    }
    else if (!strcmp((const char*)test_command, "SCT"))
    {
        hila_laser_temp = test_command_parameter;
    }
    else if (!strcmp((const char*)test_command, "EOL"))
    {
        if (test_command_parameter == 0) // end of life: none
        {
            hila_lifetime_eol = 0;
        }
        else if (test_command_parameter == 1) // end of life: warning
        {
            hila_lifetime_eol = 1;
        }
        else if (test_command_parameter == 2) // end of life: error
        {
            hila_lifetime_eol = 2;
            hila_status_register |= HILA_STA_CRITICAL_ERR;
            hila_critical_err_counter++;
        }
    }

    hila_test_puts(test_cmd_buffer);

    test_cmd_idx = 0;
    test_cmd_buffer[0] = 0;
    test_cmd_isOverflow = false; 
}


static void hila_handle_led(void)
{
    if ((hila_status_register & HILA_STA_EMISSION) == HILA_STA_EMISSION)
    {
        hila_set_ready_led(0); // hila is ready
        hila_set_emission_led(0);   // emission is on
    }
    else if ((hila_status_register & HILA_STA_NOPOWER) == HILA_STA_NOPOWER)
    {
        hila_set_ready_led(1); // hila is not ready
        hila_set_emission_led(1);   // emission is off
        if (hila_error_flag == true)
        {
            hila_set_error_led(1); // error led turns to red
        }
    }
    else
    {
        hila_set_ready_led(0); // hila is ready
        hila_set_error_led(0);  // no error
        hila_set_emission_led(1);   // emission is off
    }
}

static void hila_handle_psu_status(void)
{
    uint8_t hila_vp24v_stat;
    hila_vp24v_stat = hila_get_vp24v_status();
    if(hila_vp24v_stat == 1)
    {
        hila_status_register &= ~HILA_STA_NOPOWER;
    }
    else
    {
        hila_status_register |= HILA_STA_NOPOWER;
        hila_status_register &= ~HILA_STA_EMISSION;
    }
}

static void hila_handle_diag(void)
{
    char str[8];
    char buffer[32];

    if (hila_diag_delay > 0)
    {
        hila_diag_delay--;
    }
    else
    {
        if (hila_diag_isEnabled == true)
        {
            // setpoint conversion and sending it to serial port
            (void)ftoa_2dec(hila_setpoint_register, str);
            sprintf(buffer, "Laser setpoint: %s %%\r\n", str);
            hila_diag_puts(buffer);
            memset(buffer, '\0', 32);
            // output power conversion and sending it to serial port:
            (void)ftoa_2dec(hila_output_power_register, str);
            sprintf(buffer, "Laser output power: %s W\r\n", str);
            hila_diag_puts(buffer);
            memset(buffer, '\0', 32);
        }
        hila_diag_delay = HILA_DIAG_DELAY_TICKS - 1;
    }
    
}

void hila_io_handle(void)
{
    hila_handle_diag();
    hila_handle_led();
    hila_handle_psu_status();
    hila_calc_output_power();
    // if (isTimedOut)
    // {
    //     printf("timeout count --> %d\r\n", timeout_counter);
    //     isTimedOut = false;
    // }
    
}


void tim5_isr(void)
{
    timer_clear_flag(TIM5, TIM_SR_UIF);
    gpio_toggle(GPIOA, GPIO5);

    hila_io_handle(); 
}

void tim2_isr(void)
{
    timer_disable_counter(TIM2);
    timer_clear_flag(TIM2, TIM_SR_UIF);
    timer_disable_irq(TIM2, TIM_DIER_UIE);
    rs485_cmd_idx = 0;
    hila_rs485_cmd_buf_flush();
    isTimedOut = true;
    timeout_counter++;
    hila_rs485_resp_buf_flush();
}

void usart1_isr(void)
{   
    if (usart_get_flag(USART1, USART_SR_RXNE) == 1)
    {        
        rs485_cmd_buffer[rs485_cmd_idx] = (char)usart_recv(USART1);

        if (rs485_cmd_buffer[rs485_cmd_idx] == '\r')
        {
            
            timer_disable_counter(TIM2);
            timer_disable_irq(TIM2, TIM_DIER_UIE);
            hila_cmd_isReceived = true;
            rs485_cmd_buffer[rs485_cmd_idx] = '\0';
            rs485_cmd_idx = 0;

            if (rs485_cmd_isOverflow == true)
            {
                rs485_cmd_buffer[0] = '\0';
            }

        }
        else
        {
            timer_set_counter(TIM2, 0);
            timer_enable_irq(TIM2, TIM_DIER_UIE);
            timer_enable_counter(TIM2);
            rs485_cmd_idx++;
            if (rs485_cmd_idx >= RS485_BUF_SIZE)
            {
                rs485_cmd_idx = RS485_BUF_SIZE - 1;
                rs485_cmd_isOverflow = true;
            }
        }
    }

    // if(usart_get_flag(USART1, USART_SR_TC) == 1)
    // {
    //     USART1_CR1 &= ~(USART_CR1_TCIE);
    //     hila_set_rs485_direction(0);
    // }   
}

// void usart2_isr(void)
// {   
//     if (usart_get_flag(USART2, USART_SR_RXNE) == 1)
//     {
//         diag_cmd_buffer[diag_cmd_idx] = toupper((char)usart_recv(USART2));
//         if (diag_cmd_buffer[diag_cmd_idx] == '\r')
//         {
//             diag_cmd_isReceived = true;
//             diag_cmd_buffer[diag_cmd_idx] = '\0';

//             if (diag_cmd_isOverflow == true)
//             {
//                 diag_cmd_buffer[0] = '\0';
//             }
//         }
//         else
//         {
//             diag_cmd_idx++;
//             if (diag_cmd_idx >= DIAG_BUF_SIZE)
//             {
//                 diag_cmd_idx = DIAG_BUF_SIZE - 1;
//                 diag_cmd_isOverflow = true;
//             }
//         }
//     }   
// }

void usart3_isr(void)
{   
    if (usart_get_flag(USART3, USART_SR_RXNE) == 1)
    {
        test_cmd_buffer[test_cmd_idx] = toupper((char)usart_recv(USART3));
        if (test_cmd_buffer[test_cmd_idx] == '\r')
        {
            test_cmd_isReceived = true;
            test_cmd_buffer[test_cmd_idx] = '\0';

            if (test_cmd_isOverflow == true)
            {
                test_cmd_buffer[0] = '\0';
            }
        }
        else
        {
            test_cmd_idx++;
            if (test_cmd_idx >= TEST_BUF_SIZE)
            {
                test_cmd_idx = TEST_BUF_SIZE - 1;
                test_cmd_isOverflow = true;
            }
        }
    }   
}





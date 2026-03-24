#include "spit.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

/* ===================|CONFIG|============================*/ 
#define SPIT_BUFFER_SIZE 128
#define SPIT_TIMEOUT     10   // ms


/* ===================|INTERNAL STATE|============================*/ 
static UART_HandleTypeDef *spit_uart = NULL;


/* ===================|INIT|============================*/ 
void spitDefault(UART_HandleTypeDef *huart){
    spit_uart = huart;
}


/* ===================|INTERNAL WRITE|============================*/ 
static void spitWrite(const uint8_t *data, uint16_t len){
    if (spit_uart == NULL || data == NULL || len == 0)
        return;

    HAL_UART_Transmit(spit_uart, (uint8_t *)data, len, SPIT_TIMEOUT);
}


/* ===================|BASIC WRITE|============================*/ 
void spit(const char *str){
    if (str == NULL)
        return;

    spitWrite((uint8_t *)str, strlen(str));
}

void spitln(const char *str){
    spit(str);
    spit("\r\n");
}


/* ===================|PRINTF STYLE|============================*/ 
void spitf(const char *fmt, ...){
    char buf[SPIT_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    spit(buf);
}
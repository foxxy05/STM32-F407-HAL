#include "spit.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

/* ================= CONFIG ================= */
#define SPIT_BUFFER_SIZE 128
#define SPIT_TIMEOUT     10


/* ================= INTERNAL STATE ================= */
static UART_HandleTypeDef *spit_uart = NULL;


/* ================= INIT ================= */
void spitDefault(UART_HandleTypeDef *huart){
    spit_uart = huart;
}


/* ================= INTERNAL WRITE ================= */
static void spitWrite(const uint8_t *data, uint16_t len){
    if (!spit_uart || !data || len == 0)
        return;

    HAL_UART_Transmit(spit_uart, (uint8_t *)data, len, SPIT_TIMEOUT);
}


/* ================= BASIC HELPERS ================= */
void spit_str(const char *str){
    if (!str) return;
    spitWrite((uint8_t *)str, strlen(str));
}

void spitln_str(const char *str){
    spit_str(str);
    spit_str("\r\n");
}


/* ================= INTEGER ================= */
void spit_int(int32_t val){
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", val);
    spit_str(buf);
}

void spitln_int(int32_t val){
    spit_int(val);
    spit_str("\r\n");
}


void spit_uint(uint32_t val)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", val);
    spit_str(buf);
}

void spitln_uint(uint32_t val)
{
    spit_uint(val);
    spit_str("\r\n");
}


/* ================= FLOAT ================= */

void spit_float(double val){
    char buf[SPIT_BUFFER_SIZE];

    /* default precision = 2 */
    snprintf(buf, sizeof(buf), "%.2f", val);
    spit_str(buf);
}

void spitln_float(double val){
    spit_float(val);
    spit_str("\r\n");
}


/* ================= PRINTF ================= */
void spitf(const char *fmt, ...){
    char buf[SPIT_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    spit_str(buf);
}
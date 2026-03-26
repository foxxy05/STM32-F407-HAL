#include "spit.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ================= INTERNAL STATE ================= */
static UART_HandleTypeDef *spitUart = NULL;

/* ================= SPITCHECK FUNTION ================= */
void spitCheck(void){
    if (!spitUart)
    {
        /* can't print anything, uart not set */
        return;
    }

    spitln("SPIT OK");
    spitlnf("Buffer : %d bytes", SPIT_BUFFER_SIZE);
    spitlnf("Timeout: %d ms",    SPIT_TIMEOUT);
}

/* ================= INIT ================= */
void spitDefault(UART_HandleTypeDef *huart){
    spitUart = huart;
}

/* ================= INTERNAL WRITE ================= */
static void spitWrite(const uint8_t *data, uint16_t len){
    if (!spitUart || !data || len == 0)
        return;
    HAL_UART_Transmit(spitUart, (uint8_t *)data, len, SPIT_TIMEOUT);
}

/* ================= STRING ================= */
void spitStr(const char *str){
    if (!str) return;
    spitWrite((uint8_t *)str, strlen(str));
}

void spitLnStr(const char *str){
    if (!str) return;
    spitStr(str);
    spitStr("\r\n");
}

/* ================= CHAR ================= */
void spitChar(char c){
    spitWrite((uint8_t *)&c, 1);
}

void spitLnChar(char c){
    spitChar(c);
    spitStr("\r\n");
}

/* ================= SIGNED INTEGER ================= */
void spitInt(int32_t val){
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", val);
    spitStr(buf);
}

void spitLnInt(int32_t val){
    spitInt(val);
    spitStr("\r\n");
}

/* ================= UNSIGNED INTEGER ================= */
void spitUInt(uint32_t val){
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", val);
    spitStr(buf);
}

void spitLnUInt(uint32_t val){
    spitUInt(val);
    spitStr("\r\n");
}

/* ================= FLOAT DEFAULT================= */
void spitFloatDefault(double val){
    char buf[SPIT_BUFFER_SIZE];
    snprintf(buf, sizeof(buf), "%.2f", val);
    spitStr(buf);
}

void spitLnFloatDefault(double val){
    spitFloatDefault(val);
    spitStr("\r\n");
}

/* ================= FLOAT WITH PRECISION ================= */
void spitFloat(double val, uint8_t precision){
    if (precision > 6) precision = 6;

    char fmt[10];
    char buf[SPIT_BUFFER_SIZE];

    snprintf(fmt, sizeof(fmt), "%%.%df", precision);
    snprintf(buf, sizeof(buf), fmt, val);
    spitStr(buf);
}

void spitLnFloat(double val, uint8_t precision){
    spitFloat(val, precision);
    spitStr("\r\n");
}
/* ================= PRINTF STYLE================= */
void spitf(const char *fmt, ...){
    char buf[SPIT_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    spitStr(buf);
}

void spitlnf(const char *fmt, ...){
    char buf[SPIT_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    spitStr(buf);
    spitStr("\r\n");
}

/* ================= BASE CONVERSION CORE ================= */
static void spitBase(uint32_t val, uint8_t base){
    char buf[33];
    int i = 0;

    if (val == 0)
    {
        spitStr("0");
        return;
    }

    while (val > 0){
        uint8_t digit = val % base;
        buf[i++] = (digit < 10) ? (digit + '0') : (digit - 10 + 'A');
        val /= base;
    }

    /* reverse in place */
    int left = 0, right = i - 1;
    while (left < right)
    {
        char tmp    = buf[left];
        buf[left]   = buf[right];
        buf[right]  = tmp;
        left++; right--;
    }

    buf[i] = '\0';
    spitStr(buf);
}

/* ================= HEX ================= */
void spitHex(uint32_t val){
    spitBase(val, 16);
}

void spitLnHex(uint32_t val){
    spitHex(val);
    spitStr("\r\n");
}

/* ================= OCTAL ================= */
void spitOct(uint32_t val){
    spitBase(val, 8);
}

void spitLnOct(uint32_t val){
    spitOct(val);
    spitStr("\r\n");
}

/* ================= BINARY ================= */
void spitBin(uint32_t val){
    spitBase(val, 2);
}

void spitLnBin(uint32_t val){
    spitBin(val);
    spitStr("\r\n");
}
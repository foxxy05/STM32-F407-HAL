#include "spit.hpp"
#include <cstring>
#include <cstdio>
#include <cstdarg>

#define SPIT_BUFFER_SIZE 128
#define SPIT_TIMEOUT     10

/* global object */
SpitClass Spitter;


/* ================= INIT ================= */
void SpitClass::begin(UART_HandleTypeDef *huart)
{
    uart = huart;
}


/* ================= INTERNAL WRITE ================= */
static void write(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t len)
{
    if (!uart || !data || len == 0)
        return;

    HAL_UART_Transmit(uart, (uint8_t*)data, len, SPIT_TIMEOUT);
}


/* ================= BASIC ================= */
void SpitClass::spit(const char *str)
{
    if (!str) return;
    write(uart, (uint8_t*)str, strlen(str));
}

void SpitClass::spatln(const char *str)
{
    spit(str);
    spit("\r\n");
}


/* ================= OVERLOADS ================= */
void SpitClass::spit(int32_t val)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", val);
    spit(buf);
}

void SpitClass::spit(uint32_t val)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", val);
    spit(buf);
}

void SpitClass::spit(float val, uint8_t precision)
{
    if (precision > 8) precision = 8;

    char fmt[10];
    char buf[SPIT_BUFFER_SIZE];

    snprintf(fmt, sizeof(fmt), "%%.%df", precision);
    snprintf(buf, sizeof(buf), fmt, val);

    spit(buf);
}


/* ================= PRINTF ================= */
void SpitClass::spitf(const char *fmt, ...)
{
    char buf[SPIT_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    spit(buf);
}
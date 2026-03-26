#ifndef SPIT_H
#define SPIT_H

#include "main.h"
#include <stdint.h>

/* ================= CONFIG ================= */
#ifndef SPIT_BUFFER_SIZE
#define SPIT_BUFFER_SIZE 128
#endif

#ifndef SPIT_TIMEOUT
#define SPIT_TIMEOUT 10
#endif

/* ================= INIT ================= */
void spitDefault(UART_HandleTypeDef *huart);
void spitCheck(void);


/* ================= INTERNAL HANDLERS ================= */
/* Used by _Generic (not meant for direct use ideally) */

void spitStr(const char *str);
void spitInt(int32_t val);
void spitUInt(uint32_t val);
void spitFloatDefault(double val);
void spitChar(char c);
void spitLnChar(char c);

void spitLnStr(const char *str);
void spitLnInt(int32_t val);
void spitLnUInt(uint32_t val);
void spitLnFloatDefault(double val);


/* ================= GENERIC MACROS ================= */

#define spit(x) _Generic((x),   \
    char*: spitStr,             \
    const char*: spitStr,       \
    char: spitChar,             \
    int: spitInt,               \
    unsigned int: spitUInt,     \
    float: spitFloatDefault,    \
    double: spitFloatDefault    \  
)(x)

#define spitln(x) _Generic((x), \
    char*: spitLnStr,           \
    const char*: spitLnStr,     \
    char: spitChar,             \
    int: spitLnInt,             \
    unsigned int: spitLnUInt,   \
    float: spitLnFloatDefault,  \
    double: spitLnFloatDefault  \
)(x)


/* ================= PRINTF ================= */
void spitf(const char *fmt, ...);
void spitlnf(const char *fmt, ...);


/* ================= EXTENDED ================= */

void spitFloat(double val, uint8_t precision);
void spitLnFloat(double val, uint8_t precision);

void spitHex(uint32_t val);
void spitLnHex(uint32_t val);

void spitOct(uint32_t val);
void spitLnOct(uint32_t val);

void spitBin(uint32_t val);
void spitLnBin(uint32_t val);

#endif
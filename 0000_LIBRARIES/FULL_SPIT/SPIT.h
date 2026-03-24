#ifndef SPIT_H
#define SPIT_H

#include "main.h"
#include "stdint.h"

/* ================= INIT ================= */
void spitDefault(UART_HandleTypeDef *huart);


/* ================= INTERNAL HANDLERS ================= */
/* user should NOT call these directly */

void spit_str(const char *str);
void spit_int(int32_t val);
void spit_uint(uint32_t val);
void spit_float(double val);

void spitln_str(const char *str);
void spitln_int(int32_t val);
void spitln_uint(uint32_t val);
void spitln_float(double val);


/* ================= GENERIC MACROS ================= */

/* basic spit */
#define spit(x) _Generic((x), \
    char*: spit_str, \
    const char*: spit_str, \
    int: spit_int, \
    unsigned int: spit_uint, \
    float: spit_float, \
    double: spit_float \
)(x)

/* spitln */
#define spitln(x) _Generic((x), \
    char*: spitln_str, \
    const char*: spitln_str, \
    int: spitln_int, \
    unsigned int: spitln_uint, \
    float: spitln_float, \
    double: spitln_float \
)(x)


/* ================= PRINTF ================= */
void spitf(const char *fmt, ...);

#endif
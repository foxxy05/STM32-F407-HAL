#ifndef SPIT_HPP
#define SPIT_HPP

#include "main.h"
#include <cstdint>

class SpitClass {
public:
    void begin(UART_HandleTypeDef *huart);

    void spit(const char *str);
    void spatln(const char *str);

    void spit(int32_t val);
    void spit(uint32_t val);
    void spit(float val, uint8_t precision = 2);

    void spitf(const char *fmt, ...);

private:
    UART_HandleTypeDef *uart = nullptr;
};

/* THIS is the trick */
extern SpitClass Spitter;

#endif
/*
 * EncoderLib.h
 *
 *  Created on: Dec 15, 2025
 *      Author: Chinmay
 */

#ifndef INC_ENCODERLIB_H_
#define INC_ENCODERLIB_H_
#include "stdio.h"
#include "stdint.h"
#include "main.h"

typedef struct{
	int16_t velocity;
	int64_t position;
	uint32_t last_counter_value;
    uint8_t initialized;


}encoder_instance;

void update_encoder(encoder_instance *encoder_value ,TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *encoder_value);

#endif /* INC_ENCODERLIB_H_ */

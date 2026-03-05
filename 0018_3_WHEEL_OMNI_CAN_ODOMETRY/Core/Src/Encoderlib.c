/*
 * Encoderlib.c
 *
 *  Created on: Dec 15, 2025
 *      Author: Chinmay
 */


#include "Encoderlib.h"

void update_encoder(encoder_instance *encoder_value ,TIM_HandleTypeDef *htim){
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	if (!encoder_value->initialized) {
	    encoder_value->velocity = 0;
	    encoder_value->initialized = 1;
	    encoder_value->last_counter_value = temp_counter;
	    return;
	}
	else{
		if(temp_counter == encoder_value-> last_counter_value){
			encoder_value->velocity = 0;
		}
		else if(temp_counter > encoder_value-> last_counter_value){

			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
				encoder_value->velocity = -encoder_value ->last_counter_value - (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter);
			}
			else{
				encoder_value->velocity = temp_counter - encoder_value ->last_counter_value;
		   }
		}
		else{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value->velocity = temp_counter - encoder_value ->last_counter_value;

			}
			else{
				encoder_value ->velocity =  temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder_value ->last_counter_value);
			}
		}
	}
	encoder_value -> position += encoder_value ->velocity;
	encoder_value ->last_counter_value = temp_counter;
}

void reset_encoder(encoder_instance *encoder_value){
	encoder_value -> position = 0;
	encoder_value ->last_counter_value=0;
	encoder_value ->velocity = 0;

}

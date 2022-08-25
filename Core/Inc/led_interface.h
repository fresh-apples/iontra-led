/*
 * led_interface.h
 *
 *  Created on: Aug 21, 2022
 *      Author: gedeonzema
 */


#ifdef __cplusplus
extern "C" {
#endif



#ifndef INC_LED_INTERFACE_H_
#define INC_LED_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

#define RED 0xff00ffff
#define GREEN 0xFFFF00FF
#define BLUE 0xFFFFFF00
#define OFF 0xFFFFFFFF


//#ifdef STM32FBoard
//#include "stm32f4xx_hal.h"
//#endif

#define MAX_LEDS 5


uint8_t LED_Data[MAX_LEDS][4];
uint8_t LED_dimmed[MAX_LEDS][4];





void led_init(void);
//void set_LED(uint8_t pos, uint8_t red, uint8_t green, uint8_t blue);
void set_LED(uint8_t pos,  uint32_t col);
void set_Brightness(uint8_t brightness);
void led_show(void);
void gpio_init(void);
void pwm_init(void);




#ifdef __cplusplus
}
#endif
#endif /* INC_LED_INTERFACE_H_ */

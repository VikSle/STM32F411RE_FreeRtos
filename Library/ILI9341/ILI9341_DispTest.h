/*
 * ILI9341_DispTest.h
 *
 *  Created on: Apr 8, 2025
 *      Author: viksle
 */

#ifndef ILI9341_DISPTEST_H_
#define ILI9341_DISPTEST_H_

#include "stm32f4xx_hal.h"


extern TIM_HandleTypeDef htim1;

void Perfomance_Test(void);
void Counting_Multiple_Segments_Test(void);
void Counting_Single_Segments_Test(void);
void Alignment_Test(void);
void Lines_Example_Test(void);
void Hollow_Circles_Test(void);
void Filled_Circles_Test(void);
void Hollow_Rectangles_Test(void);
void Filled_Rectangles_Test(void);
void Individual_Pixel_Test(void);
void Individual2_Pixel_Test(void);
void Colour_Test(void);
void Image_Snow_Tiger_Test(void);
void TouchScreen_Test(void);

extern uint32_t GeneratePsuedoRandomNumber(void);

#endif /* ILI9341_DISPTEST_H_ */

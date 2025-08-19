/*
 * ONBOARDLED.cpp
 *
 *  Created on: Aug 10, 2025
 *      Author: Mugi
 */

#include "ONBOARDLED.h"
#include "stm32f1xx_hal.h"

ONBOARDLED::ONBOARDLED() {
	// TODO Auto-generated constructor stub

}

ONBOARDLED::~ONBOARDLED() {
	// TODO Auto-generated destructor stub
}
void ONBOARDLED::On(){
	/*Turn on the LED (active low, so set PC13 to RESSET) */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}
void ONBOARDLED::Off(){
	/*Turn off the LED (active low, so set PC13 to SET) */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

}
void ONBOARDLED::Toggle(){

}

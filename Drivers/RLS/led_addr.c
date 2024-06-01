/** @file led_addr.c
 *
 * @brief Drivers for addressible LEDs such as the WS2812, SK6805, or similar.
 *
 * @author Colton Crandell

 */

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "led_addr.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/


/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
ledAddrData_t ledAddrData[NUM_LEDS];
uint8_t ledDmaBuf[LED_DMA_BUF_LEN];
volatile uint8_t ledDmaCompleteFlag;

/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/


/********************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/

/**
  * @brief  One-Line Description of the Function
  * @note   Document any notes if needed
  * @param  <paramName> List and describe input parameters
  * @retval <retvalName> List and describe return value
  */
HAL_StatusTypeDef LedAddr_Init(void) {
	HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Init(&LED_TIM);

	// Clear DMA Buffer
	for (uint16_t bufIndex = 0; bufIndex < LED_DMA_BUF_LEN; bufIndex++) {
		ledDmaBuf[bufIndex] = 0;
	}

	// Set DMA transfer ready flag
	ledDmaCompleteFlag = 1;

	return halStatus;
}

void LedAddr_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
	ledAddrData[index].color_t.r = r;
	ledAddrData[index].color_t.g = g;
	ledAddrData[index].color_t.b = b;
}

HAL_StatusTypeDef LedAddr_Update(void) {
	// Check to see if previous DMA transfer has been completed
	if (!ledDmaCompleteFlag) {
		return HAL_BUSY;
	}

	// Loop through RGB LED data and fill DMA buffer
	uint16_t bufIndex = 0;

	for (uint8_t ledIndex = 0; ledIndex < NUM_LEDS; ledIndex++) {
		for (uint8_t bitIndex = 0; bitIndex < LED_BITS; bitIndex++) {
			if ((ledAddrData[ledIndex].data >> bitIndex) & 0x01) {
				ledDmaBuf[bufIndex] = LED_HI_VAL;
			} else {
				ledDmaBuf[bufIndex] = LED_LO_VAL;
			}
			bufIndex++;
		}
	}

	// The remaining part of the buffer is always 0 because of the needed 80us of reset "pulses"

	HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Start_DMA(&LED_TIM, LED_TIM_CHANNEL, (uint32_t*) ledDmaBuf, LED_DMA_BUF_LEN);

	if (halStatus == HAL_OK) {
		ledDmaCompleteFlag = 0;
	}

	return halStatus;
}

void LedAddr_Callback(void) {
	HAL_TIM_PWM_Stop_DMA(&LED_TIM, LED_TIM_CHANNEL);
	ledDmaCompleteFlag = 1;
}

/*** end of file ***/

/** @file led_addr.h
 *
 * @brief Drivers for addressible LEDs such as the WS2812, SK6805, or similar.
 *
 * @author Colton Crandell

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_ADDR_H_
#define LED_ADDR_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "main.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Define
  * @note   Document any notes if needed
  */

#define NUM_LEDS				18

#define LED_TIM					htim2
#define LED_TIM_CHANNEL			TIM_CHANNEL_1

#define LED_HI_VAL				38			// 0.594s
#define LED_LO_VAL				19			// 0.297s
#define LED_RST_PERIODS			64			// 80us

#define LED_BITS				24			// Bit order: R7...R0 G7...G0 B7...B0

#define LED_DMA_BUF_LEN			((NUM_LEDS * LED_BITS) + LED_RST_PERIODS)

/********************************************************************************
 * TYPES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Type
  * @note   Document any notes if needed
  */
typedef union {
	struct {
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} color_t;

	uint32_t data;
} ledAddrData_t;

/********************************************************************************
 * VARIABLES
 *******************************************************************************/
extern TIM_HandleTypeDef htim2;

extern ledAddrData_t ledAddrData[NUM_LEDS];
extern uint8_t ledDmaBuf[LED_DMA_BUF_LEN];
extern volatile uint8_t ledDmaCompleteFlag;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
HAL_StatusTypeDef LedAddr_Init(void);
void LedAddr_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef LedAddr_Update(void);
void LedAddr_Callback(void);

#endif // LED_ADDR_H_

/*** end of file ***/

/** @file rls.h
 *
 * @brief Application drivers for the remote launch system
 *
 * @author Colton Crandell
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RLS_H_
#define RLS_H_

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "cmsis_os.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Define
  * @note   Document any notes if needed
  */

#define LAUNCH_CHANNEL_COUNT		4		/*!< Number of supported launch channels						*/
#define LAUNCH_DURATION				2000	/*!< Hold time to ensure igniter(s) fire properly				*/
#define LAUNCH_CODE					"4321"	/*!< Change this code to whatever you want.						*/

#define SYS_VOLTAGE					3300
#define BAT_VOLT_RESISTOR_TOP		71500
#define BAT_VOLT_RESISTOR_BOT		24000

#define ENABLE_LAUNCH_OUTPUT		false	/*!< Disable fire pin output for debug and development			*/

#define ENABLE_ALARM_OUTPUT			false	/*!< Disable alarm output for debug and development				*/
#define ALARM_ON_DURATION			1000	/*!< Alarm on time during launch sequence						*/
#define ALARM_OFF_DURATION			1000	/*!< Alarm off time during launch sequence						*/

#define LED_BLINK_DELAY				1000	/*!< LED blink timer in seconds									*/


/********************************************************************************
 * RTOS FUNCTIONS FOR RLS
 *******************************************************************************/
#define RLS_LAUNCH_PROCESS_NAME        	"RLS_LAUNCH_PROCESS"
#define RLS_LAUNCH_PROCESS_ATTR_BITS   	(0)
#define RLS_LAUNCH_PROCESS_CB_MEM      	(0)
#define RLS_LAUNCH_PROCESS_CB_SIZE     	(0)
#define RLS_LAUNCH_PROCESS_STACK_MEM   	(0)
#define RLS_LAUNCH_PROCESS_PRIORITY    	osPriorityNone
#define RLS_LAUNCH_PROCESS_STACK_SIZE  	(128 * 4)

#define RLS_MAIN_PROCESS_NAME        	"RLS_MAIN_PROCESS"
#define RLS_MAIN_PROCESS_ATTR_BITS   	(0)
#define RLS_MAIN_PROCESS_CB_MEM      	(0)
#define RLS_MAIN_PROCESS_CB_SIZE     	(0)
#define RLS_MAIN_PROCESS_STACK_MEM   	(0)
#define RLS_MAIN_PROCESS_PRIORITY    	osPriorityNone
#define RLS_MAIN_PROCESS_STACK_SIZE  	(128 * 20)

#define RLS_BATTERY_PROCESS_NAME        "RLS_BATTERY_PROCESS"
#define RLS_BATTERY_PROCESS_ATTR_BITS   (0)
#define RLS_BATTERY_PROCESS_CB_MEM      (0)
#define RLS_BATTERY_PROCESS_CB_SIZE     (0)
#define RLS_BATTERY_PROCESS_STACK_MEM   (0)
#define RLS_BATTERY_PROCESS_PRIORITY    osPriorityAboveNormal
#define RLS_BATTERY_PROCESS_STACK_SIZE  (128 * 10)

#define RLS_LED_PROCESS_NAME        	"RLS_LED_PROCESS"
#define RLS_LED_PROCESS_ATTR_BITS   	(0)
#define RLS_LED_PROCESS_CB_MEM      	(0)
#define RLS_LED_PROCESS_CB_SIZE     	(0)
#define RLS_LED_PROCESS_STACK_MEM   	(0)
#define RLS_LED_PROCESS_PRIORITY    	osPriorityNone
#define RLS_LED_PROCESS_STACK_SIZE  	(128 * 20)

#define RLS_LED_BLINK_PROCESS_NAME        	"RLS_LED_BLINK_PROCESS"
#define RLS_LED_BLINK_PROCESS_ATTR_BITS   	(0)
#define RLS_LED_BLINK_PROCESS_CB_MEM      	(0)
#define RLS_LED_BLINK_PROCESS_CB_SIZE     	(0)
#define RLS_LED_BLINK_PROCESS_STACK_MEM   	(0)
#define RLS_LED_BLINK_PROCESS_PRIORITY    	osPriorityNone
#define RLS_LED_BLINK_PROCESS_STACK_SIZE  	(128 * 10)

#define RLS_ALARM_PROCESS_NAME        	"RLS_ALARM_PROCESS"
#define RLS_ALARM_PROCESS_ATTR_BITS   	(0)
#define RLS_ALARM_PROCESS_CB_MEM      	(0)
#define RLS_ALARM_PROCESS_CB_SIZE     	(0)
#define RLS_ALARM_PROCESS_STACK_MEM   	(0)
#define RLS_ALARM_PROCESS_PRIORITY    	osPriorityNone
#define RLS_ALARM_PROCESS_STACK_SIZE  	(128 * 10)

#define RLS_BEACON_PROCESS_NAME        	"RLS_BEACON_PROCESS"
#define RLS_BEACON_PROCESS_ATTR_BITS   	(0)
#define RLS_BEACON_PROCESS_CB_MEM      	(0)
#define RLS_BEACON_PROCESS_CB_SIZE     	(0)
#define RLS_BEACON_PROCESS_STACK_MEM   	(0)
#define RLS_BEACON_PROCESS_PRIORITY    	osPriorityNone
#define RLS_BEACON_PROCESS_STACK_SIZE  	(128 * 10)


/********************************************************************************
 * TYPES
 *******************************************************************************/

typedef enum {
	RLS_BLE_DISCONNECTED = 0,
	RLS_BLE_CONNECTED,
} rlsBleStatus_t;

// In order to enable a channel to fire, it first must be in the disarmed state.
// When the user tries to arm a channel, the RLS immediately checks for continuity.
// If continuity is found, the state is advanced to ARMED.
// If continuity is NOT found, the channel state is advanced to NOT READY.
// To clear the blocked state, the user must disarm the channel, connect the igniter,
// and re-arm.
typedef enum {
	RLS_CHANNEL_DISARMED = 0,
	RLS_CHANNEL_NOT_READY,				// Blinking Red
	RLS_CHANNEL_ARMED,					// Green
	RLS_CHANNEL_LAUNCH,
	RLS_CHANNEL_LAUNCHING,
	RLS_CHANNEL_LAUNCH_ERROR,			// Yellow
	RLS_CHANNEL_LAUNCH_GOOD,			// Blue
} channelState_t;

typedef struct {
	uint16_t batteryVoltage;	// Battery voltage in mV
	uint8_t batterySOC;
} rlsBatteryInfo_t;

typedef struct {
	bool lanuchActivated;
	channelState_t channelState[LAUNCH_CHANNEL_COUNT];
	bool launchCommandReceived[LAUNCH_CHANNEL_COUNT];
	rlsBleStatus_t rlsBleStatus;
	rlsBatteryInfo_t batteryInfo;
	bool ledBlinkState;
} rlsHandle_t;

/********************************************************************************
 * EXPORTED VARIABLES
 *******************************************************************************/
extern rlsHandle_t rlsHandle;

extern ADC_HandleTypeDef hadc1;

// External RTOS task IDs
extern osThreadId_t RxStatusProcessId;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
void RLS_Init(void);


#endif // RLS_H_

/*** end of file ***/

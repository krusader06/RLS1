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

/********************************************************************************
 * DEFINES
 *******************************************************************************/
/**
  * @brief  One-Line Description of the Define
  * @note   Document any notes if needed
  */

#define LAUNCH_CHANNEL_COUNT		4		/*!< Number of supported launch channels						*/

#define LAUNCH_DURATION				2000	/*!< Hold time to ensure igniter(s) fire properly				*/

#define SYS_VOLTAGE					3300
#define BAT_VOLT_RESISTOR_TOP		71500
#define BAT_VOLT_RESISTOR_BOT		24000


/********************************************************************************
 * RTOS FUNCTIONS FOR RLS
 *******************************************************************************/
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
	RLS_CHANNEL_NOT_READY,
	RLS_CHANNEL_ARMED,
	RLS_CHANNEL_LAUNCH,
	RLS_CHANNEL_LAUNCH_ERROR,
	RLS_CHANNEL_LAUNCH_GOOD,
} channelState_t;

typedef struct {
	uint16_t batteryVoltage;	// Battery voltage in mV
	uint8_t batterySOC;
} rlsBatteryInfo_t;

typedef struct {
	bool lanuchActivated;
	channelState_t channelState[LAUNCH_CHANNEL_COUNT];
	bool launchCommandReceived[LAUNCH_CHANNEL_COUNT];
	uint32_t launchCode;
	rlsBleStatus_t rlsBleStatus;
	rlsBatteryInfo_t batteryInfo;
} rlsHandle_t;

/********************************************************************************
 * EXPORTED VARIABLES
 *******************************************************************************/
extern rlsHandle_t rlsHandle;

extern ADC_HandleTypeDef hadc1;

/********************************************************************************
 * PROTOTYPES
 *******************************************************************************/
void RLS_Init(void);


#endif // RLS_H_

/*** end of file ***/

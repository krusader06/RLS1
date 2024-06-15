/** @file rls.c
 *
 * @brief Application drivers for the remote launch system
 *
 * @author Colton Crandell
 *
 */

/*******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "rls.h"

#include "main.h"
#include "cmsis_os.h"
#include "led_addr.h"

#include "app_ble.h"

/********************************************************************************
 * DEFINES
 *******************************************************************************/

/********************************************************************************
 * GLOBAL VARIABLES
 *******************************************************************************/
rlsHandle_t rlsHandle = {0};

uint16_t BAT_SOC_MAP[10][2] = {
		{10, 11510},
		{20, 11660},
		{30, 11810},
		{40, 11960},
		{50, 12100},
		{60, 12240},
		{70, 12370},
		{80, 12500},
		{90, 12620},
		{100, 12730},
};

/********************************************************************************
 * MODULAR VARIABLES
 *******************************************************************************/
osThreadId_t RlsProcessId;
osThreadId_t RlsBatteryProcessId;
osThreadId_t RlsLedProcessId;
osThreadId_t RlsAlarmProcessId;
osThreadId_t RlsBeaconProcessId;

const osThreadAttr_t RlsLaunchProcess_attr = {
    .name = RLS_LAUNCH_PROCESS_NAME,
    .attr_bits = RLS_LAUNCH_PROCESS_ATTR_BITS,
    .cb_mem = RLS_LAUNCH_PROCESS_CB_MEM,
    .cb_size = RLS_LAUNCH_PROCESS_CB_SIZE,
    .stack_mem = RLS_LAUNCH_PROCESS_STACK_MEM,
    .priority = RLS_LAUNCH_PROCESS_PRIORITY,
    .stack_size = RLS_LAUNCH_PROCESS_STACK_SIZE
};

const osThreadAttr_t RlsProcess_attr = {
    .name = RLS_MAIN_PROCESS_NAME,
    .attr_bits = RLS_MAIN_PROCESS_ATTR_BITS,
    .cb_mem = RLS_MAIN_PROCESS_CB_MEM,
    .cb_size = RLS_MAIN_PROCESS_CB_SIZE,
    .stack_mem = RLS_MAIN_PROCESS_STACK_MEM,
    .priority = RLS_MAIN_PROCESS_PRIORITY,
    .stack_size = RLS_MAIN_PROCESS_STACK_SIZE
};

const osThreadAttr_t RlsBatteryProcess_attr = {
    .name = RLS_BATTERY_PROCESS_NAME,
    .attr_bits = RLS_BATTERY_PROCESS_ATTR_BITS,
    .cb_mem = RLS_BATTERY_PROCESS_CB_MEM,
    .cb_size = RLS_BATTERY_PROCESS_CB_SIZE,
    .stack_mem = RLS_BATTERY_PROCESS_STACK_MEM,
    .priority = RLS_BATTERY_PROCESS_PRIORITY,
    .stack_size = RLS_BATTERY_PROCESS_STACK_SIZE
};

const osThreadAttr_t RlsLedProcess_attr = {
    .name = RLS_LED_PROCESS_NAME,
    .attr_bits = RLS_LED_PROCESS_ATTR_BITS,
    .cb_mem = RLS_LED_PROCESS_CB_MEM,
    .cb_size = RLS_LED_PROCESS_CB_SIZE,
    .stack_mem = RLS_LED_PROCESS_STACK_MEM,
    .priority = RLS_LED_PROCESS_PRIORITY,
    .stack_size = RLS_LED_PROCESS_STACK_SIZE
};

const osThreadAttr_t RlsLedBlinkProcess_attr = {
    .name = RLS_LED_BLINK_PROCESS_NAME,
    .attr_bits = RLS_LED_BLINK_PROCESS_ATTR_BITS,
    .cb_mem = RLS_LED_BLINK_PROCESS_CB_MEM,
    .cb_size = RLS_LED_BLINK_PROCESS_CB_SIZE,
    .stack_mem = RLS_LED_BLINK_PROCESS_STACK_MEM,
    .priority = RLS_LED_BLINK_PROCESS_PRIORITY,
    .stack_size = RLS_LED_BLINK_PROCESS_STACK_SIZE
};

const osThreadAttr_t RlsAlarmProcess_attr = {
    .name = RLS_ALARM_PROCESS_NAME,
    .attr_bits = RLS_ALARM_PROCESS_ATTR_BITS,
    .cb_mem = RLS_ALARM_PROCESS_CB_MEM,
    .cb_size = RLS_ALARM_PROCESS_CB_SIZE,
    .stack_mem = RLS_ALARM_PROCESS_STACK_MEM,
    .priority = RLS_ALARM_PROCESS_PRIORITY,
    .stack_size = RLS_ALARM_PROCESS_STACK_SIZE
};

const osThreadAttr_t RlsBeaconProcess_attr = {
    .name = RLS_BEACON_PROCESS_NAME,
    .attr_bits = RLS_BEACON_PROCESS_ATTR_BITS,
    .cb_mem = RLS_BEACON_PROCESS_CB_MEM,
    .cb_size = RLS_BEACON_PROCESS_CB_SIZE,
    .stack_mem = RLS_BEACON_PROCESS_STACK_MEM,
    .priority = RLS_BEACON_PROCESS_PRIORITY,
    .stack_size = RLS_BEACON_PROCESS_STACK_SIZE
};


/********************************************************************************
 * PRIVATE PROTOTYPES
 *******************************************************************************/

static GPIO_PinState getChannelArmSwitchState(uint8_t channelID);
static GPIO_PinState getChannelReadyState(uint8_t channelID);
static void writeChannelFirePin(uint8_t channelID, GPIO_PinState pinState);
static void serviceLaunchChannelState(uint8_t channelID);
static void detectChannelStateChanges(void);

static void measureBattery(void);
static void calculateBatterySOC(void);
static void setBatteryIndicator(uint8_t percent);

static void RlsLaunchProcess(void *argument);
static void RlsProcess(void *argument);
static void RlsBatteryProcess(void *argument);
static void RlsLedProcess(void *argument);
static void RlsAlarmProcess(void *argument);
static void RlsBeaconProcess(void *argument);

/********************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************/

static GPIO_PinState getChannelArmSwitchState(uint8_t channelID) {
	switch (channelID) {
		case 0:
			return HAL_GPIO_ReadPin(CH1_ARM_GPIO_Port, CH1_ARM_Pin);
			break;
		case 1:
			return HAL_GPIO_ReadPin(CH2_ARM_GPIO_Port, CH2_ARM_Pin);
			break;
		case 2:
			return HAL_GPIO_ReadPin(CH3_ARM_GPIO_Port, CH3_ARM_Pin);
			break;
		case 3:
			return HAL_GPIO_ReadPin(CH4_ARM_GPIO_Port, CH4_ARM_Pin);
			break;
	}

	return GPIO_PIN_RESET;
}

static GPIO_PinState getChannelReadyState(uint8_t channelID) {
	switch (channelID) {
		case 0:
			return !HAL_GPIO_ReadPin(CH1_READY_GPIO_Port, CH1_READY_Pin);
			break;
		case 1:
			return !HAL_GPIO_ReadPin(CH2_READY_GPIO_Port, CH2_READY_Pin);
			break;
		case 2:
			return !HAL_GPIO_ReadPin(CH3_READY_GPIO_Port, CH3_READY_Pin);
			break;
		case 3:
			return !HAL_GPIO_ReadPin(CH4_READY_GPIO_Port, CH4_READY_Pin);
			break;
	}

	return GPIO_PIN_RESET;
}

static void writeChannelFirePin(uint8_t channelID, GPIO_PinState pinState) {
	if (!ENABLE_LAUNCH_OUTPUT) {
		return;
	}

	switch (channelID) {
		case 0:
			HAL_GPIO_WritePin(CH1_FIRE_GPIO_Port, CH1_FIRE_Pin, pinState);
			break;
		case 1:
			HAL_GPIO_WritePin(CH2_FIRE_GPIO_Port, CH2_FIRE_Pin, pinState);
			break;
		case 2:
			HAL_GPIO_WritePin(CH3_FIRE_GPIO_Port, CH3_FIRE_Pin, pinState);
			break;
		case 3:
			HAL_GPIO_WritePin(CH4_FIRE_GPIO_Port, CH4_FIRE_Pin, pinState);
			break;
	}
}

static void serviceLaunchChannelState(uint8_t channelID) {
	GPIO_PinState armSwitchState;
	GPIO_PinState channelReady;

	armSwitchState = getChannelArmSwitchState(channelID);
	osDelay(5);
	channelReady = getChannelReadyState(channelID);

	if (armSwitchState == GPIO_PIN_RESET) {
		rlsHandle.channelState[channelID] = RLS_CHANNEL_DISARMED;
		return;
	}

	switch (rlsHandle.channelState[channelID]) {
		case RLS_CHANNEL_DISARMED:
			if (channelReady == GPIO_PIN_SET) {
				// Igniter continuity is OK!
				rlsHandle.channelState[channelID] = RLS_CHANNEL_ARMED;
			} else {
				// No igniter. Connect igniter and reset!
				rlsHandle.channelState[channelID] = RLS_CHANNEL_NOT_READY;
			}
			break;

		case RLS_CHANNEL_NOT_READY:
			// State locked until the channel is disarmed!
			break;

		case RLS_CHANNEL_ARMED:
			// Keep checking to make sure igniter is solidly connected
			if (channelReady == GPIO_PIN_RESET) {
				rlsHandle.channelState[channelID] = RLS_CHANNEL_NOT_READY;
			}

			// Wait until launch command is received.
			if ((rlsHandle.lanuchActivated) && (rlsHandle.launchCommandReceived[channelID])) {
				rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH;
			}
			break;

		case RLS_CHANNEL_LAUNCH:
			// Channel launch task has taken over the state
			break;

		case RLS_CHANNEL_LAUNCHING:
			// State locked until the channel is disarmed!
			break;

		case RLS_CHANNEL_LAUNCH_ERROR:
			// State locked until the channel is disarmed!
			break;

		case RLS_CHANNEL_LAUNCH_GOOD:
			// State locked until the channel is disarmed!
			break;
	}

}

static void detectChannelStateChanges(void) {
	static channelState_t previousState[LAUNCH_CHANNEL_COUNT] = {0};
	bool channelChanged = false;

	for (uint8_t channelID = 0; channelID < LAUNCH_CHANNEL_COUNT; channelID++) {
		if (previousState[channelID] != rlsHandle.channelState[channelID]) {
			previousState[channelID] = rlsHandle.channelState[channelID];
			channelChanged = true;
			break;
		}
	}

	if (channelChanged) {
		// Send new updates to mobile app
		osThreadFlagsSet(RxStatusProcessId, 1);
	}
}

static void measureBattery(void) {

	uint32_t batteryRaw = 0;
	uint32_t vrefRaw = 0;

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	vrefRaw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	batteryRaw = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	uint32_t vdda_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefRaw, ADC_RESOLUTION_12B);

	// Find the true battery voltage
	batteryRaw = (batteryRaw * vdda_voltage) / 4096;
	float scalingFactor = (float)BAT_VOLT_RESISTOR_BOT / (float)(BAT_VOLT_RESISTOR_TOP + BAT_VOLT_RESISTOR_BOT);
	rlsHandle.batteryInfo.batteryVoltage = batteryRaw / scalingFactor;
}

static void calculateBatterySOC(void) {
	uint8_t socLevel;
	uint8_t levelNum = sizeof(BAT_SOC_MAP) / sizeof(BAT_SOC_MAP[0]);

	uint16_t batVolt = rlsHandle.batteryInfo.batteryVoltage;

	if (batVolt <= BAT_SOC_MAP[0][1]) {
		// Lowest Level
		rlsHandle.batteryInfo.batterySOC = 0;
	} else if (batVolt >= BAT_SOC_MAP[levelNum - 1][1]) {
		// Highest Level
		rlsHandle.batteryInfo.batterySOC = 100;
	} else {
		// Find Level
		for (socLevel = 0; socLevel < levelNum; socLevel++) {
			if (batVolt < BAT_SOC_MAP[socLevel + 1][1]) {
				break;
			}
		}
	}

	uint16_t voltLow = BAT_SOC_MAP[socLevel][1];
	uint16_t voltHigh = BAT_SOC_MAP[socLevel + 1][1];
	uint8_t socLow = BAT_SOC_MAP[socLevel][0];
	uint8_t socHigh = BAT_SOC_MAP[socLevel + 1][0];

	float mapCalc = 0;
	mapCalc = (float)(batVolt - voltLow) / (float)(voltHigh - voltLow);
	mapCalc *= (socHigh - socLow);
	mapCalc += socLow;

	rlsHandle.batteryInfo.batterySOC = (uint8_t)mapCalc;
}


static void setBatteryIndicator(uint8_t percent) {

	if (percent > 100) {
		return;
	}

	// Reset all LED's to OFF
	HAL_GPIO_WritePin(GPIOB, BAT_LED1_Pin | BAT_LED2_Pin | BAT_LED3_Pin | BAT_LED4_Pin, 1);
	HAL_GPIO_WritePin(GPIOD, BAT_LED5_Pin | BAT_LED6_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, BAT_LED7_Pin | BAT_LED8_Pin | BAT_LED9_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, BAT_LED10_Pin, 1);

	if (percent == 0) {
		return;
	} else if (percent <= 10) {
		HAL_GPIO_WritePin(BAT_LED1_GPIO_Port, BAT_LED1_Pin, 0);
		return;
	} else if (percent <= 20) {
		HAL_GPIO_WritePin(BAT_LED2_GPIO_Port, BAT_LED2_Pin, 0);
		return;
	} else if (percent <= 30) {
		HAL_GPIO_WritePin(BAT_LED3_GPIO_Port, BAT_LED3_Pin, 0);
		return;
	} else if (percent <= 40) {
		HAL_GPIO_WritePin(BAT_LED4_GPIO_Port, BAT_LED4_Pin, 0);
		return;
	} else if (percent <= 50) {
		HAL_GPIO_WritePin(BAT_LED5_GPIO_Port, BAT_LED5_Pin, 0);
		return;
	} else if (percent <= 60) {
		HAL_GPIO_WritePin(BAT_LED6_GPIO_Port, BAT_LED6_Pin, 0);
		return;
	} else if (percent <= 70) {
		HAL_GPIO_WritePin(BAT_LED7_GPIO_Port, BAT_LED7_Pin, 0);
		return;
	} else if (percent <= 80) {
		HAL_GPIO_WritePin(BAT_LED8_GPIO_Port, BAT_LED8_Pin, 0);
		return;
	} else if (percent <= 90) {
		HAL_GPIO_WritePin(BAT_LED9_GPIO_Port, BAT_LED9_Pin, 0);
		return;
	} else {
		HAL_GPIO_WritePin(BAT_LED10_GPIO_Port, BAT_LED10_Pin, 0);
		return;
	}
}

/********************************************************************************
 * RTOS PROCESSES
 *******************************************************************************/


// Launch process instantiated whenever a new channel should be fired
static void RlsLaunchProcess(void *argument) {

	uint8_t channelID = (uint8_t)(uintptr_t)argument;

	// Channel must be in the launching state
	if (rlsHandle.channelState[channelID] != RLS_CHANNEL_LAUNCHING) {
		rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH_ERROR;
		osThreadExit();
	}

	// Global launch must be active
	if (rlsHandle.lanuchActivated != true) {
		rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH_ERROR;
		osThreadExit();
	}

	// Double check that the launch command has been received
	if (rlsHandle.launchCommandReceived[channelID] != true) {
		rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH_ERROR;
		osThreadExit();
	}

	// Check to see if the igniter still has continuity
	if (getChannelReadyState(channelID) != GPIO_PIN_SET) {
		rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH_ERROR;
		osThreadExit();
	}

	// Everything looks good. Fire the igniter!
	writeChannelFirePin(channelID, 1);

	rlsHandle.launchCommandReceived[channelID] = false;

	osDelay(LAUNCH_DURATION);

	writeChannelFirePin(channelID, 0);

	// Check to see if the igniter has continuity
	if (getChannelReadyState(channelID) == GPIO_PIN_SET) {
		// Igniter shouldn't have continuity, something went wrong!
		rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH_ERROR;
	} else {
		// Launch executed properly.
		rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCH_GOOD;
	}

	osThreadExit();
}


static void RlsProcess(void *argument) {
  UNUSED(argument);

  // Reset RLS Handle
  memset(&rlsHandle, 0, sizeof(rlsHandle_t));

  // Make sure all launch channels load in the not ready state for safety.
  for (uint8_t channelID = 0; channelID < LAUNCH_CHANNEL_COUNT; channelID++) {
	  rlsHandle.channelState[channelID] = RLS_CHANNEL_NOT_READY;
  }

  for(;;) {
    // Check BLE Connection Status
    APP_BLE_ConnStatus_t bleConnStatus = APP_BLE_Get_Server_Connection_Status();
    if (bleConnStatus == APP_BLE_CONNECTED_SERVER) {
    	rlsHandle.rlsBleStatus = RLS_BLE_CONNECTED;
    } else {
    	rlsHandle.rlsBleStatus = RLS_BLE_DISCONNECTED;
    	rlsHandle.lanuchActivated = false; // If disconnected, disable the launch mode
    }

    // Check each channel switch for updates
    for (uint8_t channelID = 0; channelID < LAUNCH_CHANNEL_COUNT; channelID++) {
    	serviceLaunchChannelState(channelID);
    }

    // Detect State Changes
    detectChannelStateChanges();

    // Fire any pending launch channels
    for (uint8_t channelID = 0; channelID < LAUNCH_CHANNEL_COUNT; channelID++) {

    	if (rlsHandle.channelState[channelID] != RLS_CHANNEL_LAUNCH) {
    		continue;
    	}

    	// Create a new thread to launch the channel
    	rlsHandle.channelState[channelID] = RLS_CHANNEL_LAUNCHING;
    	osThreadNew(RlsLaunchProcess, (void *)(uintptr_t)channelID, &RlsLaunchProcess_attr);
    }

    osDelay(50);
  }
}

static void RlsBatteryProcess(void *argument) {
  UNUSED(argument);

  for(;;) {

	  // Measure the battery voltage
	  measureBattery();

	  // Calculate the SOC from the voltage
	  calculateBatterySOC();

	  // Display the SOC
	  setBatteryIndicator(rlsHandle.batteryInfo.batterySOC);

	  osDelay(30000);
  }
}

static void RlsLedProcess(void *argument) {
  UNUSED(argument);

  LedAddr_Init();

  for(;;) {

	  // Set the Power LEDs
	  LedAddr_SetColor(0, 0, 255, 0);
	  LedAddr_SetColor(1, 0, 255, 0);
	  LedAddr_SetColor(2, 0, 255, 0);

	  // Check the RLS State
	  if (rlsHandle.rlsBleStatus == RLS_BLE_DISCONNECTED) {
		  LedAddr_SetColor(3, 0, 0, 0);
		  LedAddr_SetColor(4, 0, 0, 0);
		  LedAddr_SetColor(5, 0, 0, 0);
	  } else {
		  LedAddr_SetColor(3, 0, 0, 255);
		  LedAddr_SetColor(4, 0, 0, 255);
		  LedAddr_SetColor(5, 0, 0, 255);
	  }

	  // Check the Channel States

	  for (uint8_t channelID = 0; channelID < LAUNCH_CHANNEL_COUNT; channelID++) {
		  uint8_t ledIndex = (channelID * 3) + 6;
		  switch(rlsHandle.channelState[channelID]) {
			case RLS_CHANNEL_DISARMED:
				// Off
				LedAddr_SetColor(ledIndex, 0, 0, 0);
				LedAddr_SetColor(ledIndex + 1, 0, 0, 0);
				LedAddr_SetColor(ledIndex + 2, 0, 0, 0);
				break;
			case RLS_CHANNEL_NOT_READY:
				// Blinking Red
				if (rlsHandle.ledBlinkState) {
					LedAddr_SetColor(ledIndex, 255, 0, 0);
					LedAddr_SetColor(ledIndex + 1, 255, 0, 0);
					LedAddr_SetColor(ledIndex + 2, 255, 0, 0);
				} else {
					LedAddr_SetColor(ledIndex, 0, 0, 0);
					LedAddr_SetColor(ledIndex + 1, 0, 0, 0);
					LedAddr_SetColor(ledIndex + 2, 0, 0, 0);
				}
				break;
			case RLS_CHANNEL_ARMED:
				// Green
				LedAddr_SetColor(ledIndex, 0, 255, 0);
				LedAddr_SetColor(ledIndex + 1, 0, 255, 0);
				LedAddr_SetColor(ledIndex + 2, 0, 255, 0);
				break;
			case RLS_CHANNEL_LAUNCH:
				// Green
				LedAddr_SetColor(ledIndex, 0, 255, 0);
				LedAddr_SetColor(ledIndex + 1, 0, 255, 0);
				LedAddr_SetColor(ledIndex + 2, 0, 255, 0);
				break;
			case RLS_CHANNEL_LAUNCHING:
				// Green
				LedAddr_SetColor(ledIndex, 0, 255, 255);
				LedAddr_SetColor(ledIndex + 1, 0, 255, 255);
				LedAddr_SetColor(ledIndex + 2, 0, 255, 255);
				break;
			case RLS_CHANNEL_LAUNCH_ERROR:
				// Blinking Yellow
				if (rlsHandle.ledBlinkState) {
					LedAddr_SetColor(ledIndex, 255, 150, 0);
					LedAddr_SetColor(ledIndex + 1, 255, 150, 0);
					LedAddr_SetColor(ledIndex + 2, 255, 150, 0);
				} else {
					LedAddr_SetColor(ledIndex, 0, 0, 0);
					LedAddr_SetColor(ledIndex + 1, 0, 0, 0);
					LedAddr_SetColor(ledIndex + 2, 0, 0, 0);
				}
				break;
			case RLS_CHANNEL_LAUNCH_GOOD:
				// Blue
				LedAddr_SetColor(ledIndex, 0, 0, 255);
				LedAddr_SetColor(ledIndex + 1, 0, 0, 255);
				LedAddr_SetColor(ledIndex + 2, 0, 0, 255);
				break;
		  }
	  }

	  LedAddr_Update();

	  osDelay(50);
  }
}

static void RlsLedBlinkProcess(void *argument) {
	for(;;) {
		rlsHandle.ledBlinkState = !rlsHandle.ledBlinkState;
		osDelay(LED_BLINK_DELAY);
	}
}

static void RlsAlarmProcess(void *argument) {
  UNUSED(argument);

  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (1 & ENABLE_ALARM_OUTPUT));
  osDelay(50);
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, 0);

  for(;;) {
	  if (rlsHandle.lanuchActivated) {
		  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (1 & ENABLE_ALARM_OUTPUT));
		  osDelay(ALARM_ON_DURATION);
		  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, 0);
		  osDelay(ALARM_OFF_DURATION);
	  } else {
		  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, 0);
		  osDelay(100);
	  }
  }
}

static void RlsBeaconProcess(void *argument) {
  UNUSED(argument);

  for(;;) {
	  if (rlsHandle.lanuchActivated) {
		  HAL_GPIO_WritePin(BEACON_LED1_GPIO_Port, BEACON_LED1_Pin, 1);
		  HAL_GPIO_WritePin(BEACON_LED2_GPIO_Port, BEACON_LED2_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED3_GPIO_Port, BEACON_LED3_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED4_GPIO_Port, BEACON_LED4_Pin, 0);
		  osDelay(100);

		  HAL_GPIO_WritePin(BEACON_LED1_GPIO_Port, BEACON_LED1_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED2_GPIO_Port, BEACON_LED2_Pin, 1);
		  HAL_GPIO_WritePin(BEACON_LED3_GPIO_Port, BEACON_LED3_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED4_GPIO_Port, BEACON_LED4_Pin, 0);
		  osDelay(100);

		  HAL_GPIO_WritePin(BEACON_LED1_GPIO_Port, BEACON_LED1_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED2_GPIO_Port, BEACON_LED2_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED3_GPIO_Port, BEACON_LED3_Pin, 1);
		  HAL_GPIO_WritePin(BEACON_LED4_GPIO_Port, BEACON_LED4_Pin, 0);
		  osDelay(100);

		  HAL_GPIO_WritePin(BEACON_LED1_GPIO_Port, BEACON_LED1_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED2_GPIO_Port, BEACON_LED2_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED3_GPIO_Port, BEACON_LED3_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED4_GPIO_Port, BEACON_LED4_Pin, 1);
		  osDelay(100);
	  } else {
		  HAL_GPIO_WritePin(BEACON_LED1_GPIO_Port, BEACON_LED1_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED2_GPIO_Port, BEACON_LED2_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED3_GPIO_Port, BEACON_LED3_Pin, 0);
		  HAL_GPIO_WritePin(BEACON_LED4_GPIO_Port, BEACON_LED4_Pin, 0);
		  osDelay(100);
	  }
  }
}

/********************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	LedAddr_Callback();
}

/**
  * @brief  One-Line Description of the Function
  * @note   Document any notes if needed
  * @param  <paramName> List and describe input parameters
  * @retval <retvalName> List and describe return value
  */
void RLS_Init(void) {
	RlsProcessId = osThreadNew(RlsProcess, NULL, &RlsProcess_attr);
	RlsBatteryProcessId = osThreadNew(RlsBatteryProcess, NULL, &RlsBatteryProcess_attr);
	RlsLedProcessId = osThreadNew(RlsLedProcess, NULL, &RlsLedProcess_attr);
	RlsLedProcessId = osThreadNew(RlsLedBlinkProcess, NULL, &RlsLedBlinkProcess_attr);
	RlsAlarmProcessId = osThreadNew(RlsAlarmProcess, NULL, &RlsAlarmProcess_attr);
	RlsBeaconProcessId = osThreadNew(RlsBeaconProcess, NULL, &RlsBeaconProcess_attr);
}

/*** end of file ***/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "cmsis_os.h"

#include "rls.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* RLS_Service */
  uint8_t               Rls_sts_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
typedef enum {
	RLS_LAUNCH_CODE = 0,
	RLS_BATTERY_SOC,
	RLS_CHANNEL_STATE,
	RLS_LAUNCH_COMMAND,

	// Shall be last in list
	RLS_SEND_COMPLETE = 99,
} rls_status_id;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

osThreadId_t RxStatusProcessId;

const osThreadAttr_t RxStatusProcess_attr = {
    .name = RX_BLE_STATE_PROCESS_NAME,
    .attr_bits = RX_BLE_STATE_PROCESS_ATTR_BITS,
    .cb_mem = RX_BLE_STATE_PROCESS_CB_MEM,
    .cb_size = RX_BLE_STATE_PROCESS_CB_SIZE,
    .stack_mem = RX_BLE_STATE_PROCESS_STACK_MEM,
    .priority = RX_BLE_STATE_PROCESS_PRIORITY,
    .stack_size = RX_BLE_STATE_PROCESS_STACK_SIZE
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* RLS_Service */
static void Custom_Rls_sts_Update_Char(void);
static void Custom_Rls_sts_Send_Notification(void);

/* USER CODE BEGIN PFP */
void parseCommand(Custom_STM_App_Notification_evt_t *pNotification);
void receiveStatus(Custom_STM_App_Notification_evt_t *pNotification);

static void RxStatusProcess(void *argument);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* RLS_Service */
    case CUSTOM_STM_RLS_CMD_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RLS_CMD_WRITE_EVT */
    	parseCommand(pNotification);
      /* USER CODE END CUSTOM_STM_RLS_CMD_WRITE_EVT */
      break;

    case CUSTOM_STM_RLS_STS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RLS_STS_READ_EVT */

      /* USER CODE END CUSTOM_STM_RLS_STS_READ_EVT */
      break;

    case CUSTOM_STM_RLS_STS_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RLS_STS_WRITE_EVT */
    	receiveStatus(pNotification);
      /* USER CODE END CUSTOM_STM_RLS_STS_WRITE_EVT */
      break;

    case CUSTOM_STM_RLS_STS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RLS_STS_NOTIFY_ENABLED_EVT */
    	Custom_App_Context.Rls_sts_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_RLS_STS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_RLS_STS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RLS_STS_NOTIFY_DISABLED_EVT */
    	Custom_App_Context.Rls_sts_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_RLS_STS_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	RxStatusProcessId = osThreadNew(RxStatusProcess, NULL, &RxStatusProcess_attr);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* RLS_Service */
void Custom_Rls_sts_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 1;

  /* USER CODE BEGIN Rls_sts_UC_1*/

  /* USER CODE END Rls_sts_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RLS_STS, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Rls_sts_UC_Last*/
  memset(UpdateCharData, 0, sizeof(UpdateCharData));
  /* USER CODE END Rls_sts_UC_Last*/
  return;
}

void Custom_Rls_sts_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rls_sts_NS_1*/

  /* USER CODE END Rls_sts_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RLS_STS, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Rls_sts_NS_Last*/

  /* USER CODE END Rls_sts_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/






/**
 * @brief  Parses an incoming command from the BLE App
 * @param  <pNotification> Pointer to the BLE Notification Packet
 * @retval <NONE>
 */
void parseCommand(Custom_STM_App_Notification_evt_t *pNotification) {
	if (memcmp(pNotification->DataTransfered.pPayload, "sendStatus", 10) == 0) {
		osThreadFlagsSet(RxStatusProcessId, 1);
	}

	if (memcmp(pNotification->DataTransfered.pPayload, "disableLaunch", 13) == 0) {
		rlsHandle.lanuchActivated = false;
	}

}

/**
 * @brief  Receives a new setting from the mobile app then updates the settings
 * @note	The payload is structured as the settingID in bit[0] and a string starting at bit [1]
 * 		All settings here are located and updated within the eeprom.
 * @param  <NONE>
 * @retval <NONE>
 */
void receiveStatus(Custom_STM_App_Notification_evt_t *pNotification) {

	rls_status_id statusID = pNotification->DataTransfered.pPayload[0];

	switch (statusID) {
		case RLS_LAUNCH_CODE: // Received a launch code.
			// TODO - Validate launch code against stored PIN. For now, just assume OK.
			rlsHandle.lanuchActivated = true;

			// Send the launch code response back to the app
			UpdateCharData[0] = RLS_LAUNCH_CODE;
			UpdateCharData[1] = 1;
			if (Custom_App_Context.Rls_sts_Notification_Status) {
				Custom_Rls_sts_Update_Char();
			}
			break;

		case RLS_LAUNCH_COMMAND: // Received a launch command.
			if (rlsHandle.lanuchActivated != true) {
				break;
			}

			// Find the commanded launch channels
			for (uint8_t i = 0; i < LAUNCH_CHANNEL_COUNT; i++) {

				bool launchChannelSelected = pNotification->DataTransfered.pPayload[i + 1];

				if (!launchChannelSelected) {
					continue;
				}

				// Make sure the channel is able to be launched.
				if (rlsHandle.channelState[i] != RLS_CHANNEL_ARMED) {
					continue;
				}

				rlsHandle.launchCommandReceived[i] = true;
			}

			break;

		default:
			break;
	}
}



/********************************************************************************
 * RTOS PROCESSES
 *******************************************************************************/
static void RxStatusProcess(void *argument) {
  UNUSED(argument);

  for(;;) {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

	  // Send the Battery SOC
	  UpdateCharData[0] = RLS_BATTERY_SOC;
	  UpdateCharData[1] = rlsHandle.batteryInfo.batterySOC;
	  if (Custom_App_Context.Rls_sts_Notification_Status) {
		  Custom_Rls_sts_Update_Char();
	  }

	  osDelay(5);

	  // Send the Channel States
	  for (uint8_t i = 0; i < LAUNCH_CHANNEL_COUNT; i++) {
		  UpdateCharData[0] = RLS_CHANNEL_STATE;
		  UpdateCharData[1] = i;
		  UpdateCharData[2] = rlsHandle.channelState[i];
		  if (Custom_App_Context.Rls_sts_Notification_Status) {
			  Custom_Rls_sts_Update_Char();
		  }

		  osDelay(5);
	  }

	  // Send Complete
	  UpdateCharData[0] = RLS_SEND_COMPLETE;
	  if (Custom_App_Context.Rls_sts_Notification_Status) {
		  Custom_Rls_sts_Update_Char();
	  }

  }
}



/* USER CODE END FD_LOCAL_FUNCTIONS*/

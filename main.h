/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "dwt_delay.h"
#include <stdio.h>
#include <time.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union{
	uint16_t 														istd;
	uint8_t 														cstd[2];
}std_union;

typedef union{
	uint32_t 														listd;
	uint16_t 														istd[2];
	uint8_t 														cstd[4];
}l_std_union;

typedef union{
	uint64_t 														llistd;
	uint32_t 														listd[2];
	uint16_t 														istd[4];
	uint8_t 														cstd[8];
}l_l_std_union;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef 						huart1;

extern uint8_t												*UART_RX_BUFF;
extern uint8_t												*UART_TX_BUFF;
extern uint8_t												UART_first_byte;
extern uint8_t												uart_command;
extern std_union											UART_length;

extern uint8_t												*I2C_TX_BUFF;
extern uint8_t												*I2C_RX_BUFF;

extern double 												att_steps[6];
extern double 												hmc936_steps[6];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

extern void port_extension_initial_powerup(void);
extern void port_extension_init(void);
extern void port_extension_ctrl(uint8_t reg_address, uint8_t pin_state, uint8_t cs_num);
extern void attenuator_set_att(float attenuation, uint8_t address);
extern void supply_ctrl(uint8_t supply_data_1, uint8_t supply_data_2, uint8_t supply_data_3, uint8_t supply_data_4, uint8_t supply_data_5, uint8_t supply_data_6);
extern uint8_t xor_handler(uint8_t *mass);
extern void UART_pack_parser(void);
extern void DWT_Init(void);
extern void DWT_Delay(uint32_t us);
extern void DAC_control_LMP(I2C_HandleTypeDef i2c_handle, double dac_out, uint8_t channel, uint8_t dac_address);
extern uint8_t DAC_revision_LMP(I2C_HandleTypeDef i2c_handle, uint8_t dac_address);
extern void DAC_control_MCP(I2C_HandleTypeDef i2c_handle, double dac_out, uint8_t channel, uint8_t dac_address);
extern void DAC_init_LMP(I2C_HandleTypeDef i2c_handle, uint8_t dac_address);
extern uint8_t phaseshifter_set_phase(float phase, uint8_t channel, uint8_t rf_device);
extern uint8_t attenuator_HMC424_ctrl(float att, uint8_t channel, uint8_t rf_device);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HMC8073_CLK_Pin GPIO_PIN_2
#define HMC8073_CLK_GPIO_Port GPIOE
#define RX_HMC8073_LE_Pin GPIO_PIN_3
#define RX_HMC8073_LE_GPIO_Port GPIOE
#define TX_HMC8073_LE_Pin GPIO_PIN_4
#define TX_HMC8073_LE_GPIO_Port GPIOE
#define LO_HMC8073_LE_Pin GPIO_PIN_5
#define LO_HMC8073_LE_GPIO_Port GPIOE
#define HMC8073_MOSI_Pin GPIO_PIN_6
#define HMC8073_MOSI_GPIO_Port GPIOE
#define PLUS_5V_CTRL_SUP_TX_Pin GPIO_PIN_8
#define PLUS_5V_CTRL_SUP_TX_GPIO_Port GPIOI
#define PLUS_5V_CTRL_SUP_LO_Pin GPIO_PIN_13
#define PLUS_5V_CTRL_SUP_LO_GPIO_Port GPIOC
#define PLUS_5V_CTRL_SUP_RX_Pin GPIO_PIN_14
#define PLUS_5V_CTRL_SUP_RX_GPIO_Port GPIOC
#define PLUS_6V_2_CTRL_Pin GPIO_PIN_15
#define PLUS_6V_2_CTRL_GPIO_Port GPIOC
#define PLUS_6V_1_CTRL_Pin GPIO_PIN_9
#define PLUS_6V_1_CTRL_GPIO_Port GPIOI
#define SDA_2_Pin GPIO_PIN_0
#define SDA_2_GPIO_Port GPIOF
#define SCL_2_Pin GPIO_PIN_1
#define SCL_2_GPIO_Port GPIOF
#define PG_5_5V_Pin GPIO_PIN_13
#define PG_5_5V_GPIO_Port GPIOI
#define PG_3_9V_Pin GPIO_PIN_14
#define PG_3_9V_GPIO_Port GPIOI
#define HMC952_LO_Vout_Pin GPIO_PIN_3
#define HMC952_LO_Vout_GPIO_Port GPIOF
#define HMC952_TX_CONV_Vout_Pin GPIO_PIN_4
#define HMC952_TX_CONV_Vout_GPIO_Port GPIOF
#define CUR_HMC952_LO_Pin GPIO_PIN_5
#define CUR_HMC952_LO_GPIO_Port GPIOF
#define CUR_HMC_TX_CONV_Pin GPIO_PIN_6
#define CUR_HMC_TX_CONV_GPIO_Port GPIOF
#define CUR_SGM_4_Pin GPIO_PIN_7
#define CUR_SGM_4_GPIO_Port GPIOF
#define Vgg_1_1_ADC_4_Pin GPIO_PIN_8
#define Vgg_1_1_ADC_4_GPIO_Port GPIOF
#define CUR_HMC952_4_Pin GPIO_PIN_9
#define CUR_HMC952_4_GPIO_Port GPIOF
#define HMC_Vout_ADC_4_Pin GPIO_PIN_10
#define HMC_Vout_ADC_4_GPIO_Port GPIOF
#define Vgg_2_1_ADC_4_Pin GPIO_PIN_0
#define Vgg_2_1_ADC_4_GPIO_Port GPIOC
#define Vgg_1_2_ADC_4_Pin GPIO_PIN_1
#define Vgg_1_2_ADC_4_GPIO_Port GPIOC
#define Vgg_2_2_ADC_4_Pin GPIO_PIN_2
#define Vgg_2_2_ADC_4_GPIO_Port GPIOC
#define Vgg_2_1_ADC_3_Pin GPIO_PIN_3
#define Vgg_2_1_ADC_3_GPIO_Port GPIOC
#define Vgg_1_1_ADC_3_Pin GPIO_PIN_0
#define Vgg_1_1_ADC_3_GPIO_Port GPIOA
#define CUR_SGM_3_Pin GPIO_PIN_1
#define CUR_SGM_3_GPIO_Port GPIOA
#define CUR_HMC952_3_Pin GPIO_PIN_2
#define CUR_HMC952_3_GPIO_Port GPIOA
#define Vgg_1_2_ADC_3_Pin GPIO_PIN_2
#define Vgg_1_2_ADC_3_GPIO_Port GPIOH
#define Vgg_2_2_ADC_3_Pin GPIO_PIN_3
#define Vgg_2_2_ADC_3_GPIO_Port GPIOH
#define Vgg_2_1_ADC_2_Pin GPIO_PIN_4
#define Vgg_2_1_ADC_2_GPIO_Port GPIOH
#define Vgg_1_1_ADC_2_Pin GPIO_PIN_5
#define Vgg_1_1_ADC_2_GPIO_Port GPIOH
#define HMC_Vout_ADC_3_Pin GPIO_PIN_3
#define HMC_Vout_ADC_3_GPIO_Port GPIOA
#define CUR_SGM_2_Pin GPIO_PIN_4
#define CUR_SGM_2_GPIO_Port GPIOA
#define CUR_HMC952_2_Pin GPIO_PIN_5
#define CUR_HMC952_2_GPIO_Port GPIOA
#define HMC_Vout_ADC_2_Pin GPIO_PIN_6
#define HMC_Vout_ADC_2_GPIO_Port GPIOA
#define Vgg_1_2_ADC_2_Pin GPIO_PIN_7
#define Vgg_1_2_ADC_2_GPIO_Port GPIOA
#define Vgg_2_2_ADC_2_Pin GPIO_PIN_4
#define Vgg_2_2_ADC_2_GPIO_Port GPIOC
#define Vgg_2_1_ADC_1_Pin GPIO_PIN_5
#define Vgg_2_1_ADC_1_GPIO_Port GPIOC
#define Vgg_1_1_ADC_1_Pin GPIO_PIN_0
#define Vgg_1_1_ADC_1_GPIO_Port GPIOB
#define CUR_SGM_1_Pin GPIO_PIN_1
#define CUR_SGM_1_GPIO_Port GPIOB
#define CUR_HMC952_1_Pin GPIO_PIN_11
#define CUR_HMC952_1_GPIO_Port GPIOF
#define HMC_Vout_ADC_1_Pin GPIO_PIN_12
#define HMC_Vout_ADC_1_GPIO_Port GPIOF
#define Vgg_1_2_ADC_1_Pin GPIO_PIN_13
#define Vgg_1_2_ADC_1_GPIO_Port GPIOF
#define Vgg_2_2_ADC_1_Pin GPIO_PIN_14
#define Vgg_2_2_ADC_1_GPIO_Port GPIOF
#define RX_HMC642_V1_STM_3_Pin GPIO_PIN_15
#define RX_HMC642_V1_STM_3_GPIO_Port GPIOF
#define RX_HMC642_V2_STM_3_Pin GPIO_PIN_0
#define RX_HMC642_V2_STM_3_GPIO_Port GPIOG
#define RX_HMC642_V3_STM_3_Pin GPIO_PIN_1
#define RX_HMC642_V3_STM_3_GPIO_Port GPIOG
#define RX_HMC642_V4_STM_3_Pin GPIO_PIN_7
#define RX_HMC642_V4_STM_3_GPIO_Port GPIOE
#define RX_HMC642_V5_STM_3_Pin GPIO_PIN_8
#define RX_HMC642_V5_STM_3_GPIO_Port GPIOE
#define RX_HMC642_V6_STM_3_Pin GPIO_PIN_9
#define RX_HMC642_V6_STM_3_GPIO_Port GPIOE
#define TX_HMC642_V1_STM_3_Pin GPIO_PIN_10
#define TX_HMC642_V1_STM_3_GPIO_Port GPIOE
#define TX_HMC642_V2_STM_3_Pin GPIO_PIN_11
#define TX_HMC642_V2_STM_3_GPIO_Port GPIOE
#define TX_HMC642_V3_STM_3_Pin GPIO_PIN_12
#define TX_HMC642_V3_STM_3_GPIO_Port GPIOE
#define TX_HMC642_V4_STM_3_Pin GPIO_PIN_13
#define TX_HMC642_V4_STM_3_GPIO_Port GPIOE
#define TX_HMC642_V5_STM_3_Pin GPIO_PIN_14
#define TX_HMC642_V5_STM_3_GPIO_Port GPIOE
#define TX_HMC642_V6_STM_3_Pin GPIO_PIN_15
#define TX_HMC642_V6_STM_3_GPIO_Port GPIOE
#define SCL_3_Pin GPIO_PIN_7
#define SCL_3_GPIO_Port GPIOH
#define SDA_3_Pin GPIO_PIN_8
#define SDA_3_GPIO_Port GPIOH
#define Drain_ON_1_Pin GPIO_PIN_10
#define Drain_ON_1_GPIO_Port GPIOH
#define TX_POWER_1_Pin GPIO_PIN_11
#define TX_POWER_1_GPIO_Port GPIOH
#define RX_POWER_1_Pin GPIO_PIN_12
#define RX_POWER_1_GPIO_Port GPIOH
#define RE_DE_2_Pin GPIO_PIN_12
#define RE_DE_2_GPIO_Port GPIOB
#define TX_HMC642_V6_STM_1_Pin GPIO_PIN_8
#define TX_HMC642_V6_STM_1_GPIO_Port GPIOD
#define TX_HMC642_V5_STM_1_Pin GPIO_PIN_9
#define TX_HMC642_V5_STM_1_GPIO_Port GPIOD
#define TX_HMC642_V4_STM_1_Pin GPIO_PIN_10
#define TX_HMC642_V4_STM_1_GPIO_Port GPIOD
#define TX_HMC642_V3_STM_1_Pin GPIO_PIN_11
#define TX_HMC642_V3_STM_1_GPIO_Port GPIOD
#define TX_HMC642_V2_STM_1_Pin GPIO_PIN_12
#define TX_HMC642_V2_STM_1_GPIO_Port GPIOD
#define TX_HMC642_V1_STM_1_Pin GPIO_PIN_13
#define TX_HMC642_V1_STM_1_GPIO_Port GPIOD
#define RX_HMC642_V6_STM_1_Pin GPIO_PIN_14
#define RX_HMC642_V6_STM_1_GPIO_Port GPIOD
#define RX_HMC642_V5_STM_1_Pin GPIO_PIN_15
#define RX_HMC642_V5_STM_1_GPIO_Port GPIOD
#define RX_HMC642_V4_STM_1_Pin GPIO_PIN_6
#define RX_HMC642_V4_STM_1_GPIO_Port GPIOJ
#define RX_HMC642_V3_STM_1_Pin GPIO_PIN_7
#define RX_HMC642_V3_STM_1_GPIO_Port GPIOJ
#define RX_HMC642_V2_STM_1_Pin GPIO_PIN_8
#define RX_HMC642_V2_STM_1_GPIO_Port GPIOJ
#define RX_HMC642_V1_STM_1_Pin GPIO_PIN_9
#define RX_HMC642_V1_STM_1_GPIO_Port GPIOJ
#define TX_HMC642_V6_STM_2_Pin GPIO_PIN_10
#define TX_HMC642_V6_STM_2_GPIO_Port GPIOJ
#define TX_HMC642_V5_STM_2_Pin GPIO_PIN_11
#define TX_HMC642_V5_STM_2_GPIO_Port GPIOJ
#define TX_HMC642_V4_STM_2_Pin GPIO_PIN_0
#define TX_HMC642_V4_STM_2_GPIO_Port GPIOK
#define TX_HMC642_V3_STM_2_Pin GPIO_PIN_1
#define TX_HMC642_V3_STM_2_GPIO_Port GPIOK
#define TX_HMC642_V2_STM_2_Pin GPIO_PIN_2
#define TX_HMC642_V2_STM_2_GPIO_Port GPIOK
#define TX_HMC642_V1_STM_2_Pin GPIO_PIN_2
#define TX_HMC642_V1_STM_2_GPIO_Port GPIOG
#define RX_HMC642_V6_STM_2_Pin GPIO_PIN_3
#define RX_HMC642_V6_STM_2_GPIO_Port GPIOG
#define RX_HMC642_V5_STM_2_Pin GPIO_PIN_4
#define RX_HMC642_V5_STM_2_GPIO_Port GPIOG
#define RX_HMC642_V4_STM_2_Pin GPIO_PIN_5
#define RX_HMC642_V4_STM_2_GPIO_Port GPIOG
#define RX_HMC642_V3_STM_2_Pin GPIO_PIN_6
#define RX_HMC642_V3_STM_2_GPIO_Port GPIOG
#define RX_HMC642_V2_STM_2_Pin GPIO_PIN_7
#define RX_HMC642_V2_STM_2_GPIO_Port GPIOG
#define RX_HMC642_V1_STM_2_Pin GPIO_PIN_8
#define RX_HMC642_V1_STM_2_GPIO_Port GPIOG
#define TX_POWER_2_Pin GPIO_PIN_6
#define TX_POWER_2_GPIO_Port GPIOC
#define RX_POWER_2_Pin GPIO_PIN_7
#define RX_POWER_2_GPIO_Port GPIOC
#define Drain_ON_2_Pin GPIO_PIN_8
#define Drain_ON_2_GPIO_Port GPIOC
#define TX_POWER_3_Pin GPIO_PIN_9
#define TX_POWER_3_GPIO_Port GPIOC
#define RX_POWER_3_Pin GPIO_PIN_8
#define RX_POWER_3_GPIO_Port GPIOA
#define Drain_ON_3_Pin GPIO_PIN_9
#define Drain_ON_3_GPIO_Port GPIOA
#define TX_POWER_4_Pin GPIO_PIN_10
#define TX_POWER_4_GPIO_Port GPIOA
#define RX_POWER_4_Pin GPIO_PIN_11
#define RX_POWER_4_GPIO_Port GPIOA
#define Drain_ON_4_Pin GPIO_PIN_12
#define Drain_ON_4_GPIO_Port GPIOA
#define RX_HM_936_V1_STM_Pin GPIO_PIN_12
#define RX_HM_936_V1_STM_GPIO_Port GPIOC
#define RX_HM_936_V2_STM_Pin GPIO_PIN_0
#define RX_HM_936_V2_STM_GPIO_Port GPIOD
#define RX_HM_936_V3_STM_Pin GPIO_PIN_1
#define RX_HM_936_V3_STM_GPIO_Port GPIOD
#define RX_HM_936_V4_STM_Pin GPIO_PIN_2
#define RX_HM_936_V4_STM_GPIO_Port GPIOD
#define RX_HM_936_V5_STM_Pin GPIO_PIN_3
#define RX_HM_936_V5_STM_GPIO_Port GPIOD
#define RX_HM_936_V6_STM_Pin GPIO_PIN_4
#define RX_HM_936_V6_STM_GPIO_Port GPIOD
#define RE_DE_1_Pin GPIO_PIN_7
#define RE_DE_1_GPIO_Port GPIOD
#define SPI_1_CS_3_Pin GPIO_PIN_14
#define SPI_1_CS_3_GPIO_Port GPIOJ
#define SPI_1_CS_2_Pin GPIO_PIN_15
#define SPI_1_CS_2_GPIO_Port GPIOJ
#define SPI_1_CS_1_Pin GPIO_PIN_10
#define SPI_1_CS_1_GPIO_Port GPIOG
#define TX_HMC936_V6_STM_Pin GPIO_PIN_12
#define TX_HMC936_V6_STM_GPIO_Port GPIOG
#define TX_HMC936_V5_STM_Pin GPIO_PIN_13
#define TX_HMC936_V5_STM_GPIO_Port GPIOG
#define TX_HMC936_V4_STM_Pin GPIO_PIN_14
#define TX_HMC936_V4_STM_GPIO_Port GPIOG
#define TX_HMC936_V3_STM_Pin GPIO_PIN_3
#define TX_HMC936_V3_STM_GPIO_Port GPIOK
#define TX_HMC936_V2_STM_Pin GPIO_PIN_4
#define TX_HMC936_V2_STM_GPIO_Port GPIOK
#define TX_HMC936_V1_STM_Pin GPIO_PIN_5
#define TX_HMC936_V1_STM_GPIO_Port GPIOK
#define TX_HMC642_V6_STM_4_Pin GPIO_PIN_6
#define TX_HMC642_V6_STM_4_GPIO_Port GPIOK
#define TX_HMC642_V5_STM_4_Pin GPIO_PIN_7
#define TX_HMC642_V5_STM_4_GPIO_Port GPIOK
#define TX_HMC642_V4_STM_4_Pin GPIO_PIN_15
#define TX_HMC642_V4_STM_4_GPIO_Port GPIOG
#define TX_HMC642_V3_STM_4_Pin GPIO_PIN_8
#define TX_HMC642_V3_STM_4_GPIO_Port GPIOB
#define TX_HMC642_V2_STM_4_Pin GPIO_PIN_9
#define TX_HMC642_V2_STM_4_GPIO_Port GPIOB
#define TX_HMC642_V1_STM_4_Pin GPIO_PIN_0
#define TX_HMC642_V1_STM_4_GPIO_Port GPIOE
#define RX_HMC642_V6_STM_4_Pin GPIO_PIN_2
#define RX_HMC642_V6_STM_4_GPIO_Port GPIOI
#define RX_HMC642_V5_STM_4_Pin GPIO_PIN_3
#define RX_HMC642_V5_STM_4_GPIO_Port GPIOE
#define RX_HMC642_V4_STM_4_Pin GPIO_PIN_4
#define RX_HMC642_V4_STM_4_GPIO_Port GPIOI
#define RX_HMC642_V3_STM_4_Pin GPIO_PIN_5
#define RX_HMC642_V3_STM_4_GPIO_Port GPIOI
#define RX_HMC642_V2_STM_4_Pin GPIO_PIN_6
#define RX_HMC642_V2_STM_4_GPIO_Port GPIOI
#define RX_HMC642_V1_STM_4_Pin GPIO_PIN_7
#define RX_HMC642_V1_STM_4_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

#define	ON														0x01
#define OFF														0x00

#define OK_CODE												0x00
#define ERROR_CODE										0x01

/******************************************* UART1 DEFINES *******************************************/

//SERVICE BYTES
#define	SYNCHRO												0x02
#define UART_ADDR											0x0A

#define SERVICE_BITS_LEN							0x06

//FUNCTIONS
#define ECHO													0x00
#define SUPPLY_CTRL										0x01
#define ATT_CTRL											0x02
#define MCP_DAC												0x03
#define	LMP_DAC												0x04
#define PHASESHIFT										0x05
#define ATTENUATOR										0x06

/******************************************* I2C 2 DEFINES *******************************************/
	



/******************************************* HMC8073 DEFINES *******************************************/

#define TX_ATTENUATOR									0x00
#define RX_ATTENUATOR									0x01
#define LO_ATTENUATOR									0x02

/******************************************* Supply bytes DEFINES *******************************************/

#define DD7														0
#define DD8														1
#define DD9 													2

typedef enum
{
	PLUS_5_CTRL_TX = 0,
	PLUS_5_CTRL_LO,
	PLUS_5_CTRL_RX,
	PLUS_6_CTRL_2_CONV,
	PLUS_6_CTRL_1_CONV	
}SUP_BYTE_1;

typedef enum
{
	MINUS_5_CTRL = 0,
	Vg_ON_CONV,
	PLUS_3_2_CTRL_3,
	PLUS_3_3_CTRL_3,
	PLUS_5_CTRL_1,
	PLUS_3_2_CTRL_1,
	PLUS_3_3_CTRL_1,
	PLUS_5_CTRL_2,
}SUP_BYTE_2;

typedef enum
{
	MINUS_5_CTRL_1 = 0,
	PLUS_6_CTRL_1,
	PLUS_2_6_CTRL_1,
	MINUS_5_CTRL_2,
	PLUS_3_2_CTRL_2,
	PLUS_6_CTRL_2,
	PLUS_3_3_CTRL_2,	
	PLUS_2_6_CTRL_2,	
}SUP_BYTE_3;

typedef enum
{
	PLUS_5_CTRL_3 = 0,
	MINUS_5_CTRL_3,
	PLUS_2_6_CTRL_4,
	PLUS_3_3_CTRL_4,
	PLUS_6_CTRL_4,
	PLUS_3_2_CTRL_4,
	MINUS_5_CTRL_4,	
	PLUS_5_CTRL_4,
}SUP_BYTE_4;

typedef enum
{
	PLUS_2_6_CTRL_3 = 0,
	PLUS_6_CTRL_3,
	BLANK,
	RF_DIGITAL_CTRL,
	Drain_ON_1,
	Drain_ON_2,
	Drain_ON_3,	
	Drain_ON_4,
}SUP_BYTE_5;

typedef enum
{
	Gate_ON_1_1 = 0,
	Gate_ON_2_1,
	Gate_ON_1_2,	
	Gate_ON_2_2,
	Gate_ON_1_3,
	Gate_ON_2_3,
	Gate_ON_1_4,
	Gate_ON_2_4
}SUP_BYTE_6;

typedef enum
{
	CONVERTER = 0,
	PA_1,
	PA_2,
	PA_3,
	PA_4
}DEVICES;

typedef enum
{
	CHANNEL_0 = 0,
	CHANNEL_1
}DAC_CHANNELS;

typedef enum
{
	I2C1_BUS = 0,
	I2C2_BUS,
	I2C3_BUS
}I2C_BUSES;

/******************************************* LM92066 Defines ***********************************************/

#define LMP_HMC952_GATE_ADDRESS				0x3F

#define LO_CONV_DAC_LMP             	0x00
#define TX_CONV_DAC_LMP             	0x01

#define PASSWORD_1										0xCD
#define PASSWORD_2										0xF0

//REGISTERS
#define TEMPM													0x00
#define TEMPL													0x01

#define DAC0M													0x02
#define DAC0L													0x03
#define DAC1M													0x04
#define DAC1L													0x05

#define TEMP_STATUS										0x07
#define OVRD_CNTL											0x08
#define TEMPM_OVRD										0x09
#define TEMPL_OVRD										0x0A
#define DAC0M_OVRD										0x0B
#define DAC0L_OVRD										0x0C
#define DAC1M_OVRD										0x0D
#define DAC1L_OVRD										0x0E
#define EEPROM_CNTL										0x0F
#define RESET													0x10
#define ACC_CNTL											0x11
#define BLK_CNTL											0x16
#define ADR_LK												0x18
#define DRV_STATUS										0x1E
#define VERSION												0x1F
#define BURN_CT												0x40

#define DEL0													0x41		//0x41..0x67 DEL0..DEL38

#define DAC0_BASEM										0x68
#define DAC0_BASEL										0x69
#define DAC1_BASEM										0x6A
#define DAC1_BASEL										0x6B
			
#define PAD0													0x6C		//0x6C..0x7F PAD0..PAD19

/******************************************* MCP47CVB02 Defines ***********************************************/

#define MCP_HMC573_GATE_ADDRESS				0x63

#define WRITE_OPERATION								0x00
#define READ_OPERATION								0x03

#define MCP_VOL_DAC_0									0x00
#define MCP_VOL_DAC_1									0x01

#define MCP_VOL_VREF									0x08
#define MCP_VOL_VDR										0x09
#define MCP_VOL_GAIN_STATUS						0x0A

typedef enum
{
	VR0A = 0,
	VR0B,
	VR1A,	
	VR1B
}MCP_VREF_REG;

typedef enum
{
	PD0A = 0,
	PD0B,
	PD1A,	
	PD1B
}MCP_POWERDOWN_REG;

typedef enum
{
	MTPMA = 6,
	POR,
	G0,	
	G1
}MCP_GAIN_STATUS_REG;

/******************************************* HMC936 Defines ***********************************************/

#define TX_PHASESHIFT								0x00
#define RX_PHASESHIFT								0x01

/******************************************* HMC424 Defines ***********************************************/

#define TX_ATT											0x00
#define RX_ATT											0x01

/******************************************* MAX7301 Defines ***********************************************/

#define PORT_4_SINGLE_CTRL					0x24									//PORT4....PORT31 = 0x24...0x3F
#define PORT4_11_8BITS							0x44									//PORT4..11 8 bits...PORT24..31 8 bits = 0x44...0x58
#define PORT28_31_4BITS							0x5C



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

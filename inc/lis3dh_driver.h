#ifndef INC_LIS3DH_DRIVER_H_
#define INC_LIS3DH_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**************************************************************************
    I2C ADDRESS/BITS (7 bits length; pg. 25/54)
**************************************************************************/
#define LIS3DH_I2C_DEFAULT_ADDRESS  ((uint16_t)(0x18 << 1)) // if SDO/SA0 = 1 -> 0001 1000, then 0001 1000 << 1 = 00011 0000
#define LIS3DH_I2C_UPDATED_ADDRESS  ((uint16_t)(0x19 << 1)) // if SDO/SA0 = 0 -> 0001 1001, then 0001 1001 << 1 = 00011 0010

/**************************************************************************
    I2C AUTO INCREMENT MASK (Datasheet pg. 25/54)
**************************************************************************/
#define	LIS3DH_AUTOINCREMENT_MASK	((uint8_t)0x80)

/**************************************************************************
    CONVERSION DELAY (mS)
**************************************************************************/
#define LIS3DH_CONVERSIONDELAY      (100)   //

/**************************************************************************
    ACCELERATION CONSTANT (m/s²)
**************************************************************************/
#define LIS3DH_ACCELERATION_CONST   (9.81f)   //

/**************************************************************************
    ACCELEROMETER REGISTERS (pg. 31-32/54)
**************************************************************************/
// directions [0x00-0x06] reserved
#define LIS3DH_REG_ACCEL_STATUS_REG_AUX     ((uint8_t)(0x07))	// Status Auxiliary Register
#define LIS3DH_REG_ACCEL_OUT_ADC1_L         ((uint8_t)(0x08))	// 1-Axis Acceleration Data Low Register
#define LIS3DH_REG_ACCEL_OUT_ADC1_H         ((uint8_t)(0x09))	// 1-Axis Acceleration Data High Register
#define LIS3DH_REG_ACCEL_OUT_ADC2_L         ((uint8_t)(0x0A))	// 2-Axis Acceleration Data Low Register
#define LIS3DH_REG_ACCEL_OUT_ADC2_H         ((uint8_t)(0x0B))	// 2-Axis Acceleration Data High Register
#define LIS3DH_REG_ACCEL_OUT_ADC3_L         ((uint8_t)(0x0C))	// 3-Axis Acceleration Data Low Register
#define LIS3DH_REG_ACCEL_OUT_ADC3_H         ((uint8_t)(0x0D))	// 3-Axis Acceleration Data High Register
// direction 0x0E reserved
#define LIS3DHTR_REG_ACCEL_WHO_AM_I         ((uint8_t)(0x0F))	// Device identification Register
// directions [0x10-0x1D] reserved
#define LIS3DH_REG_CTRL_REG0                ((uint8_t)(0x1E))	//
#define LIS3DH_REG_TEMP_CFG_REG             ((uint8_t)(0x1F))	// Temperature Sensor Register
#define LIS3DH_REG_ACCEL_CTRL_REG1          ((uint8_t)(0x20))	// Accelerometer Control Register 1
#define LIS3DH_REG_ACCEL_CTRL_REG2          ((uint8_t)(0x21))	// Accelerometer Control Register 2
#define LIS3DH_REG_ACCEL_CTRL_REG3          ((uint8_t)(0x22))	// Accelerometer Control Register 3
#define LIS3DH_REG_ACCEL_CTRL_REG4          ((uint8_t)(0x23))	// Accelerometer Control Register 4
#define LIS3DH_REG_ACCEL_CTRL_REG5          ((uint8_t)(0x24))	// Accelerometer Control Register 5
#define LIS3DH_REG_ACCEL_CTRL_REG6          ((uint8_t)(0x25))	// Accelerometer Control Register 6
#define LIS3DH_REG_ACCEL_REFERENCE          ((uint8_t)(0x26))	// Reference/Datacapture Register
#define LIS3DH_REG_ACCEL_STATUS_REG         ((uint8_t)(0x27))	// Status Register
#define LIS3DH_REG_ACCEL_OUT_X_L            ((uint8_t)(0x28))	// X-Axis Acceleration Data Low Register
#define LIS3DH_REG_ACCEL_OUT_X_H            ((uint8_t)(0x29))	// X-Axis Acceleration Data High Register
#define LIS3DH_REG_ACCEL_OUT_Y_L            ((uint8_t)(0x2A))	// Y-Axis Acceleration Data Low Register
#define LIS3DH_REG_ACCEL_OUT_Y_H            ((uint8_t)(0x2B))	// Y-Axis Acceleration Data High Register
#define LIS3DH_REG_ACCEL_OUT_Z_L            ((uint8_t)(0x2C))	// Z-Axis Acceleration Data Low Register
#define LIS3DH_REG_ACCEL_OUT_Z_H            ((uint8_t)(0x2D))	// Z-Axis Acceleration Data High Register
#define LIS3DH_REG_ACCEL_FIFO_CTRL          ((uint8_t)(0x2E))	// FIFO Control Register
#define LIS3DH_REG_ACCEL_FIFO_SRC           ((uint8_t)(0x2F))	// FIFO Source Register
#define LIS3DH_REG_ACCEL_INT1_CFG           ((uint8_t)(0x30))	// Interrupt 1 Configuration Register
#define LIS3DH_REG_ACCEL_INT1_SRC           ((uint8_t)(0x31))	// Interrupt 1 Source Register
#define LIS3DH_REG_ACCEL_INT1_THS           ((uint8_t)(0x32))	// Interrupt 1 Threshold Register
#define LIS3DH_REG_ACCEL_INT1_DURATION      ((uint8_t)(0x33))	// Interrupt 1 Duration Register
#define LIS3DH_REG_ACCEL_INT2_CFG           ((uint8_t)(0x34))	// Interrupt 2 Configuration Register
#define LIS3DH_REG_ACCEL_INT2_SRC           ((uint8_t)(0x35))	// Interrupt 2 Source Register
#define LIS3DH_REG_ACCEL_INT2_THS           ((uint8_t)(0x36))	// Interrupt 2 Threshold Register
#define LIS3DH_REG_ACCEL_INT2_DURATION      ((uint8_t)(0x37))	// Interrupt 2 Duration Register
#define LIS3DH_REG_ACCEL_CLICK_CFG          ((uint8_t)(0x38))	// Interrupt Click Recognition Register
#define LIS3DH_REG_ACCEL_CLICK_SRC          ((uint8_t)(0x39))	// Interrupt Click Source Register
#define LIS3DH_REG_ACCEL_CLICK_THS          ((uint8_t)(0x3A))	// Interrupt Click Threshold Register
#define LIS3DH_REG_ACCEL_TIME_LIMIT         ((uint8_t)(0x3B))	// Click Time Limit Register
#define LIS3DH_REG_ACCEL_TIME_LATENCY       ((uint8_t)(0x3C))	// Click Time Latency Register
#define LIS3DH_REG_ACCEL_TIME_WINDOW        ((uint8_t)(0x3D))	// Click Time Window Register
#define LIS3DH_ACT_THS                      ((uint8_t)(0x3E))	//
#define LIS3DH_ACT_DUR                      ((uint8_t)(0x3F))	//

#define LIS3DH_DEFAULT_ADDRES_VALUE			((uint8_t)(0x33))
#define LIS3DH_DEFAULT_CTRL_REG0_VALUE		((uint8_t)(0x10))
#define LIS3DH_DEFAULT_CTRL_REG1_VALUE		((uint8_t)(0x07))

/**************************************************************************
    OPERATING MODES Datasheet Pg.16/54
**************************************************************************/
    /*
    * Operating mode 		CTRL_REG1[3]	CTRL_REG4[3]
    * 						(LPen bit) 		 (HR bit)
    *
    * Low-power mode
    * (8-bit data output)		1 				0
    *
    * Normal mode
    * (10-bit data output) 	0 				0
    *
    * High-resolution mode
    * (12-bit data output) 	0 				1
    *
    * Not allowed 				1 				1
    */
#define LIS3DH_LOW_POWER_MODE_CTRL_REG1		((uint8_t)0x08)	// 0000 1000
#define LIS3DH_LOW_POWER_MODE_CTRL_REG4		((uint8_t)0x00)	// 0000 0000

#define LIS3DH_NORMAL_POWER_MODE_CTRL_REG1	((uint8_t)0x00)	// 0000 0000
#define LIS3DH_NORMAL_POWER_MODE_CTRL_REG4	((uint8_t)0x00)	// 0000 0000

#define LIS3DH_HIGH_RESO_MODE_CTRL_REG1		((uint8_t)0x08)	// 0000 1000
#define LIS3DH_HIGH_RESO_MODE_CTRL_REG4		((uint8_t)0x08)	// 0000 1000

/**************************************************************************
    TEMPERATURE MODES Datasheet Pg.34/54
**************************************************************************/
#define LIS3DH_TEMP_DIS_MODE				((uint8_t)0x00)
#define LIS3DH_TEMP_EN_MODE					((uint8_t)0x40)

/**************************************************************************
    ADC MODES Datasheet Pg.34/54
**************************************************************************/
#define LIS3DH_ADC_DIS_MODE					((uint8_t)0x00)
#define LIS3DH_ADC_EN_MODE					((uint8_t)0x80)

/**************************************************************************
    DATA RATE CONFIGURATION MODES Datasheet Pg.35/54
**************************************************************************/
    /*
    * ODR3		ODR2	ODR1	ODR0	Power mode selection
    *
    * 0 		0 	  	0 		0 		Power-down mode
    * 0 		0 		0 		1 		HR / Normal / Low-power mode (1 Hz)
    * 0 		0 		1 		0 		HR / Normal / Low-power mode (10 Hz)
    * 0 		0 		1 		1 		HR / Normal / Low-power mode (25 Hz)
    * 0 		1 		0 		0 		HR / Normal / Low-power mode (50 Hz)
    * 0 		1 		0 		1 		HR / Normal / Low-power mode (100 Hz)
    * 0 		1 		1 		0 		HR / Normal / Low-power mode (200 Hz)
    * 0 		1 		1 		1 		HR / Normal / Low-power mode (400 Hz)
    * 1 		0 		0 		0 		Low power mode (1.60 kHz)
    * 1 		0 		0 		1 		HR / normal (1.344 kHz); Low-power mode (5.376 kHz)
    */
#define LIS3DH_ODR_Power_down_mode				((uint8_t)0x00)
#define LIS3DH_ODR_1_Hz							((uint8_t)0x10)
#define LIS3DH_ODR_10_Hz						((uint8_t)0x20)
#define LIS3DH_ODR_25_Hz						((uint8_t)0x30)
#define LIS3DH_ODR_50_Hz						((uint8_t)0x40)
#define LIS3DH_ODR_100_Hz						((uint8_t)0x50)
#define LIS3DH_ODR_200_Hz						((uint8_t)0x60)
#define LIS3DH_ODR_400_Hz						((uint8_t)0x70)
#define LIS3DH_ODR_Low_power_mode_1_60_kHz		((uint8_t)0x80)
#define LIS3DH_ODR_HR_or_normal_1_344_kHz		((uint8_t)0x90)
#define LIS3DH_ODR_Low_power_mode_5_376_kHz		((uint8_t)0x90)

/**************************************************************************
    FULL SCALE MODES Datasheet Pg.37/54
**************************************************************************/
    /*
    * FS1		FS0		Mode
    * 0		0 		±2g
    * 0 		1 		±4g
    * 1 		0		±10g
    * 1 		1		±16g
    */
#define LIS3DH_FS_2G_MODE	((uint8_t)0x00)
#define LIS3DH_FS_4G_MODE	((uint8_t)0x10)
#define LIS3DH_FS_10G_MODE	((uint8_t)0x20)
#define LIS3DH_FS_16G_MODE	((uint8_t)0x30)

/**************************************************************************
    BLOCK DATA UPDATE Datasheet Pg.37/54
**************************************************************************/
#define LIS3DH_BDU_CONTINUES			((uint8_t)0x00)
#define LIS3DH_BDU_NOT_UNTIL_MSB_LSB	((uint8_t)0x80)

/**************************************************************************
    ACCELEROMETER STUCTS & ENUMS
**************************************************************************/
typedef struct lis3dh_s
{
    I2C_HandleTypeDef *i2cHandle;   // I2C handle
    float acc_mps2[3];              // Acceleration data [X,Y,Z] in m/s^2
    float temp_deg;                 // Temperature data in degreds
}lis3dh_t;

typedef enum lis3dh_status_e
{
    LIS3DH_OK 				= 0x00U,
    LIS3DH_ID_ERROR			= 0x01U,
    LIS3DH_STM_HAL_ERROR	= 0x02U,
    LIS3DH_STM_HAL_BUSY     = 0x03U,
    LIS3DH_STM_HAL_TIMEOUT  = 0x04U,
}lis3dh_status_t;

/**************************************************************************
    FUNCTIONS DEFINITIONS
**************************************************************************/

/**
* @brief Initialize of the peripheral
* @author Antonio Lotti Villar ()
* @date 12/07/2025
* @param dst A pointer to the lis3dh structure
* @param i2cHandle A pointer to the I2C handle
*/
HAL_StatusTypeDef lis3dh_init( lis3dh_t *dst, I2C_HandleTypeDef *i2cHandle );

/**
* @brief Read the accelerometor data from the peripheral
* @author Antonio Lotti Villar ()
* @param dst A pointer to the lis3dh structure
*/
HAL_StatusTypeDef lis3dh_read_acc( lis3dh_t *dst );

/**
* @brief Read the temperature data from the peripheral
* @author Antonio Lotti Villar ()
* @param dst A pointer to the lis3dh structure
*/
HAL_StatusTypeDef lis3dh_read_temp( lis3dh_t *dst);

/**
* @brief Read a register from the peripheral
* @author Antonio Lotti Villar ()
* @param dst A pointer to the lis3dh structure
* @param reg Register address to read
* @param data Pointer where the data will be store
*/
HAL_StatusTypeDef lis3dh_read_reg( lis3dh_t *dst, uint8_t reg, uint8_t *data );

/**
* @brief Read a register from the peripheral
* @author Antonio Lotti Villar ()
* @param dst A pointer to the lis3dh structure
* @param reg Register address to start reading
* @param data Pointer where the data will be store
* @param length How many registers will be read
*/
HAL_StatusTypeDef lis3dh_read_regs_array( lis3dh_t *dst, uint8_t reg, uint8_t *data, uint16_t length );

/**
* @brief Read a register from the peripheral
* @author Antonio Lotti Villar ()
* @param dst A pointer to the lis3dh structure
* @param reg Register address to write
* @param data Pointer to the data that will be written in the registers
*/
HAL_StatusTypeDef lis3dh_write_reg( lis3dh_t *dst, uint8_t reg, uint8_t *data);

#endif /* INC_LIS3DH_DRIVER_H_ */
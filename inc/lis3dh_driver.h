#ifndef LIS3DH_DRIVER_H
#define LIS3DH_DRIVER_H

#include "stm32f4xx_hal.h"

/**************************************************************************
    I2C ADDRESS/BITS (7 bits length; pg. 25/54)
**************************************************************************/
#define LIS3DHTR_I2C_DEFAULT_ADDRESS    (0x33 >> 1)  // if SDO/SA0 = 1 -> 00110011, then 00110011 >> 1 = 00011001  
#define LIS3DHTR_I2C_UPDATED_ADDRESS    (0x18 >> 1)  // if SDO/SA0 = 0 -> 00110001, then 00110001 >> 1 = 00011000  

/**************************************************************************
    CONVERSION DELAY (in mS)
**************************************************************************/
#define LIS3DHTR_CONVERSIONDELAY    (100)   //

/**************************************************************************
    ACCELEROMETER REGISTERS (pg. 31-32/54)
**************************************************************************/
// directions [0x00-0x06] reserved
#define LIS3DHTR_REG_ACCEL_STATUS_REG_AUX   (0x07)  // Status Register Auxiliar
#define LIS3DHTR_REG_ACCEL_OUT_ADC1_L       (0x08)  // 1-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_ADC1_H       (0x09)  // 1-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_OUT_ADC2_L       (0x0A)  // 2-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_ADC2_H       (0x0B)  // 2-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_OUT_ADC3_L       (0x0C)  // 3-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_ADC3_H       (0x0D)  // 3-Axis Acceleration Data High Register
// direction 0x0E reserved
#define LIS3DHTR_REG_ACCEL_WHO_AM_I         (0x0F)  // Device identification Register
// directions [0x10-0x1D] reserved
#define LIS3DH_CTRL_REG0                    (0x1E)  // 
#define LIS3DHTR_REG_TEMP_CFG_REG           (0x1F)  // Temperature Sensor Register
#define LIS3DHTR_REG_ACCEL_CTRL_REG1        (0x20)  // Accelerometer Control Register 1
#define LIS3DHTR_REG_ACCEL_CTRL_REG2        (0x21)  // Accelerometer Control Register 2
#define LIS3DHTR_REG_ACCEL_CTRL_REG3        (0x22)  // Accelerometer Control Register 3
#define LIS3DHTR_REG_ACCEL_CTRL_REG4        (0x23)  // Accelerometer Control Register 4
#define LIS3DHTR_REG_ACCEL_CTRL_REG5        (0x24)  // Accelerometer Control Register 5
#define LIS3DHTR_REG_ACCEL_CTRL_REG6        (0x25)  // Accelerometer Control Register 6
#define LIS3DHTR_REG_ACCEL_REFERENCE        (0x26)  // Reference/Datacapture Register
#define LIS3DHTR_REG_ACCEL_STATUS_REG       (0x27)  // Status Register
#define LIS3DHTR_REG_ACCEL_OUT_X_L          (0x28)  // X-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_X_H          (0x29)  // X-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_OUT_Y_L          (0x2A)  // Y-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_Y_H          (0x2B)  // Y-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_OUT_Z_L          (0x2C)  // Z-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_Z_H          (0x2D)  // Z-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL        (0x2E)  // FIFO Control Register
#define LIS3DHTR_REG_ACCEL_FIFO_SRC         (0x2F)  // FIFO Source Register
#define LIS3DHTR_REG_ACCEL_INT1_CFG         (0x30)  // Interrupt 1 Configuration Register
#define LIS3DHTR_REG_ACCEL_INT1_SRC         (0x31)  // Interrupt 1 Source Register
#define LIS3DHTR_REG_ACCEL_INT1_THS         (0x32)  // Interrupt 1 Threshold Register
#define LIS3DHTR_REG_ACCEL_INT1_DURATION    (0x33)  // Interrupt 1 Duration Register
#define LIS3DHTR_REG_ACCEL_INT2_CFG         (0x34)  // Interrupt 2 Configuration Register
#define LIS3DHTR_REG_ACCEL_INT2_SRC         (0x35)  // Interrupt 2 Source Register
#define LIS3DHTR_REG_ACCEL_INT2_THS         (0x36)  // Interrupt 2 Threshold Register
#define LIS3DHTR_REG_ACCEL_INT2_DURATION    (0x37)  // Interrupt 2 Duration Register
#define LIS3DHTR_REG_ACCEL_CLICK_CFG        (0x38)  // Interrupt Click Recognition Register
#define LIS3DHTR_REG_ACCEL_CLICK_SRC        (0x39)  // Interrupt Click Source Register
#define LIS3DHTR_REG_ACCEL_CLICK_THS        (0x3A)  // Interrupt Click Threshold Register
#define LIS3DHTR_REG_ACCEL_TIME_LIMIT       (0x3B)  // Click Time Limit Register
#define LIS3DHTR_REG_ACCEL_TIME_LATENCY     (0x3C)  // Click Time Latency Register
#define LIS3DHTR_REG_ACCEL_TIME_WINDOW      (0x3D)  // Click Time Window Register
#define LIS3DH_ACT_THS                      (0x3E)  // 
#define LIS3DH_ACT_DUR                      (0x3F)  // 

/**************************************************************************
    ACCELEROMETER STUCT
**************************************************************************/
typedef struct lis3dh_s
{
    I2C_HandleTypeDef *i2cHandle;   // I2C handle
    float acc_mps2[3];              // Acceleration data [X,Y,Z] in m/s^2
    float temp_deg;                 // Temperature data in degreds 
}lis3dh_t;

#endif /*LIS3DH_DRIVER_H*/

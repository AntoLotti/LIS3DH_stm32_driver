#include <lis3dh_driver.h>


HAL_StatusTypeDef lis3dh_read_reg( lis3dh_t *dst, uint8_t reg, uint8_t *data )
{
    return HAL_I2C_Mem_Read(
        dst->i2cHandle, LIS3DH_I2C_DEFAULT_ADDRESS, 
        reg, I2C_MEMADD_SIZE_8BIT, 
        data, 1,
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef lis3dh_read_regs( lis3dh_t *dst, uint8_t reg, uint8_t *data, uint16_t length )
{
    return HAL_I2C_Mem_Read(
        dst->i2cHandle, LIS3DH_I2C_DEFAULT_ADDRESS, 
        reg, I2C_MEMADD_SIZE_8BIT, 
        data, length,
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef lis3dh_write_reg( lis3dh_t *dst, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Write(
        dst->i2cHandle, LIS3DH_I2C_DEFAULT_ADDRESS, 
        reg, I2C_MEMADD_SIZE_8BIT, 
        data, 1,
        HAL_MAX_DELAY
    );
}
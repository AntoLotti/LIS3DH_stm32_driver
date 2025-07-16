#include <lis3dh_driver.h>

uint8_t lis3dh_init( lis3dh_t *dst, I2C_HandleTypeDef *i2cHandle )
{
    dst->i2cHandle      = i2cHandle;

    dst->acc_mps2[0]    = 0.0f;
    dst->acc_mps2[1]    = 0.0f;
    dst->acc_mps2[2]    = 0.0f;

    dst->temp_deg       = 0.0f

    /* Check The Device */
    //#TODO

    /* Config The Sensor */
    //#TODO
}

HAL_StatusTypeDef lis3dh_read_acc( lis3dh_t *dst )
{
    // Read Raw Data Form The Registers Device
    uint8_t regData[6];

    HAL_StatusTypeDef status = lis3dh_read_regs(dst, LIS3DH_REG_ACCEL_OUT_X_L, regData, 6);

    if (status != HAL_OK)
        return status;

    // Combine Raw Registers Datas 12 Bits Each
    int16_t raw_data[3];
    raw_data[0] = ((int16_t)( ( (int16_t)regData[1] << 8 ) | regData[0] ) >> 6 ); // X-axis
    raw_data[1] = ((int16_t)( ( (int16_t)regData[3] << 8 ) | regData[2] ) >> 6 ); // Y-axis
    raw_data[2] = ((int16_t)( ( (int16_t)regData[5] << 8 ) | regData[4] ) >> 6 ); // Z-axis

    // Calculate The Data in m/s² 
    dst->acc_mps2[0] = 0.004f * LIS3DH_ACCELERATION_CONST * raw_data[0];
    dst->acc_mps2[1] = 0.004f * LIS3DH_ACCELERATION_CONST * raw_data[1];
    dst->acc_mps2[2] = 0.004f * LIS3DH_ACCELERATION_CONST * raw_data[2];

    return status;
}

HAL_StatusTypeDef lis3dh_read_temp( lis3dh_t *dst)
{

}

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
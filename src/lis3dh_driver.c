#include <lis3dh_driver.h>

HAL_StatusTypeDef lis3dh_init( lis3dh_t *dst, I2C_HandleTypeDef *i2cHandle )
{
    dst->i2cHandle      = i2cHandle;

    dst->acc_mps2[0]    = 0.0f;
    dst->acc_mps2[1]    = 0.0f;
    dst->acc_mps2[2]    = 0.0f;

    dst->temp_deg       = 0.0f;

    /**********************
        Check Device
    **********************/
    uint8_t data_to_check 		= 0x00;
    HAL_StatusTypeDef status	= HAL_OK;

    status = lis3dh_read_reg( dst, LIS3DHTR_REG_ACCEL_WHO_AM_I, &data_to_check );
    if (status != HAL_OK)
        return status;
    if( data_to_check != LIS3DH_DEFAULT_ADDRES_VALUE )
        return HAL_ERROR;

    status = lis3dh_read_reg( dst, LIS3DH_REG_CTRL_REG0, &data_to_check );
    if (status != HAL_OK)
        return status;
    if( data_to_check != LIS3DH_DEFAULT_CTRL_REG0_VALUE )
        return HAL_ERROR;

    status = lis3dh_read_reg( dst, LIS3DH_REG_ACCEL_CTRL_REG1, &data_to_check );
    if (status != HAL_OK)
        return status;
    if( data_to_check != LIS3DH_DEFAULT_CTRL_REG1_VALUE )
        return HAL_ERROR;

    /**********************
        Accelerometer Conf
    **********************/
    // Operating Mode //
    uint8_t ctr_reg1 = LIS3DH_NORMAL_POWER_MODE_CTRL_REG1 | 0x07;
    uint8_t ctr_reg4 = LIS3DH_HIGH_RESO_MODE_CTRL_REG4;
    /*
    * TODO: add the all the modes
    * now only the high
    */

    //  Data Rate Mode //
    ctr_reg1 |= LIS3DH_ODR_100_Hz;
    /*
    * TODO: add the all the modes
    * now only the 100Hz mode
    */

    // Full Scale Mode //
    ctr_reg4 |= LIS3DH_FS_2G_MODE;
    /*
    * TODO: add the all the modes
    * now only the ±2g
    */

    // BDU //
    ctr_reg4 = LIS3DH_BDU_NOT_UNTIL_MSB_LSB;

    status = lis3dh_write_reg( dst, LIS3DH_REG_ACCEL_CTRL_REG1, &ctr_reg1 );
    if (status != HAL_OK)
        return status;

    status = lis3dh_write_reg( dst, LIS3DH_REG_ACCEL_CTRL_REG4, &ctr_reg4 );
    if (status != HAL_OK)
        return status;

    uint8_t temp1 = 0x00;
    uint8_t temp2 = 0x00;
    status = lis3dh_read_reg( dst, LIS3DH_REG_ACCEL_CTRL_REG1, &temp1 );
    status = lis3dh_read_reg( dst, LIS3DH_REG_ACCEL_CTRL_REG2, &temp2 );
    if( (temp1 != ctr_reg1 && temp2 != ctr_reg4) )
        return HAL_ERROR;

    /****************************
        Temperature Sensor Conf
    ****************************/
    uint8_t temp_cfg_reg = LIS3DH_TEMP_EN_MODE | LIS3DH_ADC_EN_MODE;
    status = lis3dh_write_reg( dst, LIS3DH_REG_TEMP_CFG_REG, &temp_cfg_reg );
    if (status != HAL_OK)
        return status;

    status = lis3dh_read_reg(dst, LIS3DH_REG_TEMP_CFG_REG, &temp_cfg_reg);
    if(temp_cfg_reg != (LIS3DH_TEMP_EN_MODE | LIS3DH_ADC_EN_MODE))
        return HAL_ERROR;

    return status;
}

HAL_StatusTypeDef lis3dh_read_acc( lis3dh_t *dst )
{
    // Read Raw Data Form The Registers Device
    uint8_t reg_data[6] = {0x00};

    HAL_StatusTypeDef status = lis3dh_read_regs_array(dst, LIS3DH_REG_ACCEL_OUT_X_L, reg_data, 6);
    if (status != HAL_OK)
        return status;

    // Combine Raw Registers Data
    int16_t raw_data[3] = {0x0000};
    raw_data[0] = ((int16_t)( ( (int16_t)reg_data[1] << 8 ) | reg_data[0] ) >> 4 ); // X-axis
    raw_data[1] = ((int16_t)( ( (int16_t)reg_data[3] << 8 ) | reg_data[2] ) >> 4 ); // Y-axis
    raw_data[2] = ((int16_t)( ( (int16_t)reg_data[5] << 8 ) | reg_data[4] ) >> 4 ); // Z-axis

    // Calculate The Data in m/s²
    dst->acc_mps2[0] = 0.001f * LIS3DH_ACCELERATION_CONST * raw_data[0];
    dst->acc_mps2[1] = 0.001f * LIS3DH_ACCELERATION_CONST * raw_data[1];
    dst->acc_mps2[2] = 0.001f * LIS3DH_ACCELERATION_CONST * raw_data[2];

    /*
    * Now only implemented the data acquisition in
    * high
    * TODO
    * Implement the others mode
    */

    return status;
}

HAL_StatusTypeDef lis3dh_read_temp( lis3dh_t *dst)
{

    // Read Raw Data Form The Registers Device
    uint8_t reg_data[2] = {0x00};
    HAL_StatusTypeDef status = lis3dh_read_regs_array(dst, LIS3DH_REG_ACCEL_OUT_ADC3_L, reg_data, 2);
    if (status != HAL_OK)
        return status;

    // Combine Raw Register Data 10 Bits
    int16_t raw_data = ((int16_t)( ( (int16_t)reg_data[1] << 8 ) | reg_data[0] ) >> 6 );

    // Calculate The Data in degree
    dst->temp_deg = raw_data;

    /*
    * Now only implemented the data acquisition in
    * normal mode
    * TODO
    * Implement the others mode
    */

    return status;
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

HAL_StatusTypeDef lis3dh_read_regs_array( lis3dh_t *dst, uint8_t reg, uint8_t *data, uint16_t length )
{
    return HAL_I2C_Mem_Read(
        dst->i2cHandle, LIS3DH_I2C_DEFAULT_ADDRESS,
        reg | LIS3DH_AUTOINCREMENT_MASK,
        I2C_MEMADD_SIZE_8BIT,
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
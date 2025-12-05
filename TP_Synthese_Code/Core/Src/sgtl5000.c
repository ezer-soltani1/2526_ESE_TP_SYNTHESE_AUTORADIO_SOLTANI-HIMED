#include "sgtl5000.h"
#include "main.h"
#include <stdio.h>

HAL_StatusTypeDef sgtl5000_i2c_write_register(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t value) {
    uint8_t data[4];
    data[0] = (reg >> 8) & 0xFF;        // High byte of the register address
    data[1] = reg & 0xFF;               // Low byte of the register address
    data[2] = (value >> 8) & 0xFF;      // High byte of the value
    data[3] = value & 0xFF;             // Low byte of the value

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(h_sgtl5000->hi2c, h_sgtl5000->i2c_address, data, 4, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("SGTL5000 Write Error: Reg 0x%04X, Val 0x%04X, Status %d\r\n", reg, value, status);
    }
    return status;
}

HAL_StatusTypeDef sgtl5000_i2c_read_register(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t *value) {
    uint8_t reg_addr[2];
    uint8_t data_rx[2];

    reg_addr[0] = (reg >> 8) & 0xFF;    // High byte of the register address
    reg_addr[1] = reg & 0xFF;           // Low byte of the register address

    // Send register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(h_sgtl5000->hi2c, h_sgtl5000->i2c_address, reg_addr, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("SGTL5000 Read Error (Addr Tx): Reg 0x%04X, Status %d\r\n", reg, status);
        return status;
    }

    // Receive data
    status = HAL_I2C_Master_Receive(h_sgtl5000->hi2c, h_sgtl5000->i2c_address, data_rx, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("SGTL5000 Read Error (Data Rx): Reg 0x%04X, Status %d\r\n", reg, status);
        return status;
    }

    *value = (data_rx[0] << 8) | data_rx[1]; // Combine bytes to 16-bit value
    return HAL_OK;
}

HAL_StatusTypeDef sgtl5000_i2c_set_bit(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t mask) {
    uint16_t current_value;
    HAL_StatusTypeDef ret = sgtl5000_i2c_read_register(h_sgtl5000, reg, &current_value);
    if (ret != HAL_OK) return ret;
    return sgtl5000_i2c_write_register(h_sgtl5000, reg, current_value | mask);
}


HAL_StatusTypeDef sgtl5000_i2c_clear_bit(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t mask) {
    uint16_t current_value;
    HAL_StatusTypeDef ret = sgtl5000_i2c_read_register(h_sgtl5000, reg, &current_value);
    if (ret != HAL_OK) return ret;
    return sgtl5000_i2c_write_register(h_sgtl5000, reg, current_value & ~mask);
}

HAL_StatusTypeDef sgtl5000_init(h_sgtl5000_t *h_sgtl5000) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint16_t chip_id_value;

    printf("SGTL5000: Starting initialization...\r\n");

    // --- 1. Read CHIP_ID to verify communication ---
    ret = sgtl5000_i2c_read_register(h_sgtl5000, SGTL5000_CHIP_ID, &chip_id_value);
    if (ret != HAL_OK) {
        printf("SGTL5000: Failed to read CHIP_ID. Status: %d\r\n", ret);
        return ret;
    }
    printf("SGTL5000: CHIP_ID = 0x%04X \r\n", chip_id_value);


    // Power up LINEOUT, HP, ADC, DAC, REFTOP, VAG (from ANA_POWER register)
    // Value: 0x6AFF (bits: LINEOUT_POWERUP, ADC_POWERUP, CAPLESS_HEADPHONE_POWERUP, DAC_POWERUP, HEADPHONE_POWERUP, REFTOP_POWERUP)
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ANA_POWER, 0x6AFF);
    printf("SGTL5000: CHIP_ANA_POWER set to 0x6AFF\r\n");

    // Configure charge pump to use VDDIO rail (bit 5 and bit 6 of LINREG_CTRL)
    // This is needed if VDDA and VDDIO are > 3.1V (e.g., 3.3V)
    ret |= sgtl5000_i2c_set_bit(h_sgtl5000, SGTL5000_CHIP_LINREG_CTRL, (1 << 5) | (1 << 6));
    printf("SGTL5000: CHIP_LINREG_CTRL charge pump configured\r\n");

    // Set ground, ADC, DAC reference voltage (VDDA/2) and bias current.
    // Assuming VDDA = 3.3V, VDDA/2 is approx 1.65V. The default 0.8V is 0x0118.
    // For 1.575V VAG_VAL (0x01FF) and BIAS_CTRL = -50% (bit 1) and SMALL_POP = 1 (bit 0)
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_REF_CTRL, 0x01FF);
    printf("SGTL5000: CHIP_REF_CTRL set to 0x01FF\r\n");

    // Set LINEOUT reference voltage to VDDIO/2 (1.65 V) and bias current to 0.36 mA
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_LINE_OUT_CTRL, 0x031E);
    printf("SGTL5000: CHIP_LINE_OUT_CTRL set to 0x031E\r\n");

    // Enable short detect mode for headphone (HP_OUT)
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_SHORT_CTRL, 0x1106);
    printf("SGTL5000: CHIP_SHORT_CTRL set to 0x1106\r\n");

    // Enable Zero-cross detect for HP_OUT and ADC (CHIP_ANA_CTRL)
    // And set ADC input to Line-In
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ANA_CTRL, 0x0004); // SELECT_ADC = LINEIN
    printf("SGTL5000: CHIP_ANA_CTRL set to 0x0004 (ADC from Line-In)\r\n");

    // Power up desired digital blocks: I2S_IN, I2S_OUT, DAC, ADC
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_DIG_POWER, 0x0073);
    printf("SGTL5000: CHIP_DIG_POWER set to 0x0073\r\n");

    // --- 3. System MCLK and Sample Clock ---

    // Configure SYS_FS (sample rate) to 48 kHz. MCLK_FREQ is for 256*Fs
    // For 48kHz, SYS_FS = 0x0002 (bits 3:2)
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_CLK_CTRL, 0x0004); // SYS_FS = 48kHz (bit 2 is 1)
    printf("SGTL5000: CHIP_CLK_CTRL set to 0x0004 (48kHz SYS_FS)\r\n");

    // Configure I2S interface in slave mode (SGTL5000 acts as slave to STM32)
    // I2S_MS=0 (Slave), I2S_DLEN=16 bits (0x10)
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_I2S_CTRL, 0x0130);
    printf("SGTL5000: CHIP_I2S_CTRL set to 0x0130 (I2S Slave, 16-bit)\r\n");

    // --- 4. Input/Output Routing ---

    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ADCDAC_CTRL, 0x0000); // DAC_MUTE = 0
    printf("SGTL5000: CHIP_ADCDAC_CTRL set to 0x0000 (DAC Unmuted)\r\n");

    // Set DAC volume (0dB, max)
    ret |= sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_DAC_VOL, 0x3C3C); // 0x3C = 0dB
    printf("SGTL5000: CHIP_DAC_VOL set to 0x3C3C (0dB)\r\n");

    if (ret != HAL_OK) {
        printf("SGTL5000: Initialization failed during one or more steps.\r\n");
    } else {
        printf("SGTL5000: Initialization complete.\r\n");
    }

    return ret;
}

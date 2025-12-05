#ifndef INC_SGTL5000_H_
#define INC_SGTL5000_H_


#include "main.h"

// SGTL5000 I2C slave address (7-bit)
#define SGTL5000_I2C_ADDR_7BIT 0x0A // 0001010b
#define SGTL5000_I2C_ADDR_WRITE (SGTL5000_I2C_ADDR_7BIT << 1) // 00010100b (0x14)
#define SGTL5000_I2C_ADDR_READ ((SGTL5000_I2C_ADDR_7BIT << 1) | 0x01)


typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t i2c_address;
} h_sgtl5000_t;

typedef enum sgtl5000_registers_enum {
    SGTL5000_CHIP_ID            = 0x0000,
    SGTL5000_CHIP_DIG_POWER     = 0x0002,
    SGTL5000_CHIP_CLK_CTRL      = 0x0004,
    SGTL5000_CHIP_I2S_CTRL      = 0x0006,
    SGTL5000_CHIP_ADCDAC_CTRL   = 0x000E,
    SGTL5000_CHIP_DAC_VOL       = 0x0010,
    SGTL5000_CHIP_ANA_CTRL      = 0x0024,
    SGTL5000_CHIP_LINREG_CTRL   = 0x0026,
    SGTL5000_CHIP_REF_CTRL      = 0x0028,
    SGTL5000_CHIP_LINE_OUT_CTRL = 0x002C,
    SGTL5000_CHIP_LINE_OUT_VOL  = 0x002E,
    SGTL5000_CHIP_ANA_POWER     = 0x0030,
    SGTL5000_CHIP_SHORT_CTRL    = 0x003C,
} sgtl5000_registers_t;


HAL_StatusTypeDef sgtl5000_i2c_write_register(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t value);
HAL_StatusTypeDef sgtl5000_i2c_read_register(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t *value);
HAL_StatusTypeDef sgtl5000_i2c_set_bit(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t mask);
HAL_StatusTypeDef sgtl5000_i2c_clear_bit(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t mask);
HAL_StatusTypeDef sgtl5000_init(h_sgtl5000_t *h_sgtl5000);


#endif

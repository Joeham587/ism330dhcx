#include <cstdint>
#include <stdio.h>


#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#pragma once

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector {
  float XAxis;
  float YAxis;
  float ZAxis;
};
#endif

enum class AccelODR {
    POWER_DOWN      = 0b0000,  // Power-down mode
    ODR_12_5Hz      = 0b0001,  // 12.5 Hz
    ODR_26Hz        = 0b0010,  // 26 Hz
    ODR_52Hz        = 0b0011,  // 52 Hz
    ODR_104Hz       = 0b0100,  // 104 Hz
    ODR_208Hz       = 0b0101,  // 208 Hz
    ODR_416Hz       = 0b0110,  // 416 Hz
    ODR_833Hz       = 0b0111,  // 833 Hz
    ODR_1660Hz      = 0b1000,  // 1.66 kHz
    ODR_3330Hz      = 0b1001,  // 3.33 kHz
    ODR_6660Hz      = 0b1010,  // 6.66 kHz
};

enum class AccelScale {
    SCALE_2G  = 0b00,  // ±2g
    SCALE_16G = 0b01,  // ±16g
    SCALE_4G  = 0b10,  // ±4g
    SCALE_8G  = 0b11,  // ±8g
};

enum class AccelLPF {
    LPF_DISABLED = 0b0,  // First stage digital filtering
    LPF_ENABLED  = 0b1,  // LPF2 second filtering stage
};

enum class GyroODR {
    POWER_DOWN      = 0b0000,  // Power-down mode
    ODR_12_5Hz     = 0b0001,  // 12.5 Hz
    ODR_26Hz        = 0b0010,  // 26 Hz
    ODR_52Hz        = 0b0011,  // 52 Hz
    ODR_104Hz       = 0b0100,  // 104 Hz
    ODR_208Hz       = 0b0101,  // 208 Hz
    ODR_416Hz       = 0b0110,  // 416 Hz
    ODR_833Hz       = 0b0111,  // 833 Hz
    ODR_1660Hz      = 0b1000,  // 1.66 kHz
    ODR_3330Hz      = 0b1001,  // 3.33 kHz
    ODR_6660Hz      = 0b1010,  // 6.66 kHz
};

enum class GyroScale {
    SCALE_250DPS  = 0b00,  // ±250 dps
    SCALE_500DPS  = 0b01,  // ±500 dps
    SCALE_1000DPS = 0b10,  // ±1000 dps
    SCALE_2000DPS = 0b11,  // ±2000 dps
};

enum class GyroFS125 {
    FS_125_DISABLED = 0b0,  // FS selected through FS_G[1:0]
    FS_125_ENABLED  = 0b1,  // FS set to ±125 dps
};

enum class GyroFS4000 {
    FS_4000_DISABLED = 0b0,  // FS selected through FS_G[1:0] or FS_125
    FS_4000_ENABLED  = 0b1,  // FS set to ±4000 dps
};

// Define register addresses and default values for ISM330DHCX
struct ISM330DHCX_Registers {
    // Register addresses
    enum Address : uint8_t {
        FUNC_CFG_ACCESS    = 0x01,
        PIN_CTRL           = 0x02,
        // Reserved: 0x03-0x06
        FIFO_CTRL1         = 0x07,
        FIFO_CTRL2         = 0x08,
        FIFO_CTRL3         = 0x09,
        FIFO_CTRL4         = 0x0A,
        COUNTER_BDR_REG1   = 0x0B,
        COUNTER_BDR_REG2   = 0x0C,
        INT1_CTRL          = 0x0D,
        INT2_CTRL          = 0x0E,
        WHO_AM_I           = 0x0F,
        CTRL1_XL           = 0x10,
        CTRL2_G            = 0x11,
        CTRL3_C            = 0x12,
        CTRL4_C            = 0x13,
        CTRL5_C            = 0x14,
        CTRL6_C            = 0x15,
        CTRL7_G            = 0x16,
        CTRL8_XL           = 0x17,
        CTRL9_XL           = 0x18,
        CTRL10_C           = 0x19,
        ALL_INT_SRC        = 0x1A,
        WAKE_UP_SRC        = 0x1B,
        TAP_SRC            = 0x1C,
        D6D_SRC            = 0x1D,
        STATUS_REG         = 0x1E,
        // Reserved: 0x1F
        OUT_TEMP_L         = 0x20,
        OUT_TEMP_H         = 0x21,
        OUTX_L_G           = 0x22,
        OUTX_H_G           = 0x23,
        OUTY_L_G           = 0x24,
        OUTY_H_G           = 0x25,
        OUTZ_L_G           = 0x26,
        OUTZ_H_G           = 0x27,
        OUTX_L_A           = 0x28,
        OUTX_H_A           = 0x29,
        OUTY_L_A           = 0x2A,
        OUTY_H_A           = 0x2B,
        OUTZ_L_A           = 0x2C,
        OUTZ_H_A           = 0x2D,
        // Reserved: 0x2E-0x34
        EMB_FUNC_STATUS_MAINPAGE = 0x35,
        FSM_STATUS_A_MAINPAGE    = 0x36,
        FSM_STATUS_B_MAINPAGE    = 0x37,
        MLC_STATUS_MAINPAGE      = 0x38,
        STATUS_MASTER_MAINPAGE   = 0x39,
        FIFO_STATUS1             = 0x3A,
        FIFO_STATUS2             = 0x3B,
        // Reserved: 0x3C-0x3F
        TIMESTAMP0               = 0x40,
        TIMESTAMP1               = 0x41,
        TIMESTAMP2               = 0x42,
        TIMESTAMP3               = 0x43,
        // Reserved: 0x44-0x55
        TAP_CFG0                  = 0x56,
        TAP_CFG1                  = 0x57,
        TAP_CFG2                  = 0x58,
        TAP_THS_6D                = 0x59,
        INT_DUR2                  = 0x5A,
        WAKE_UP_THS               = 0x5B,
        WAKE_UP_DUR               = 0x5C,
        FREE_FALL                 = 0x5D,
        MD1_CFG                   = 0x5E,
        MD2_CFG                   = 0x5F,
        // Reserved: 0x60-0x62
        INTERNAL_FREQ_FINE        = 0x63,
        // Reserved: 0x64-0x6E
        INT_OIS                   = 0x6F,
        CTRL1_OIS                 = 0x70,
        CTRL2_OIS                 = 0x71,
        CTRL3_OIS                 = 0x72,
        X_OFS_USR                 = 0x73,
        Y_OFS_USR                 = 0x74,
        Z_OFS_USR                 = 0x75,
        // Reserved: 0x76-0x77
        FIFO_DATA_OUT_TAG         = 0x78,
        FIFO_DATA_OUT_X_L         = 0x79,
        FIFO_DATA_OUT_X_H         = 0x7A,
        FIFO_DATA_OUT_Y_L         = 0x7B,
        FIFO_DATA_OUT_Y_H         = 0x7C,
        FIFO_DATA_OUT_Z_L         = 0x7D,
        FIFO_DATA_OUT_Z_H         = 0x7E,
    };

    // Default values
    static constexpr uint8_t DEFAULT_FUNC_CFG_ACCESS    = 0x01;
    static constexpr uint8_t DEFAULT_PIN_CTRL           = 0x2F;
    static constexpr uint8_t DEFAULT_FIFO_CTRL1         = 0x00;
    static constexpr uint8_t DEFAULT_FIFO_CTRL2         = 0x80;
    static constexpr uint8_t DEFAULT_FIFO_CTRL3         = 0x90;
    static constexpr uint8_t DEFAULT_FIFO_CTRL4         = 0xA0;
    static constexpr uint8_t DEFAULT_COUNTER_BDR_REG1   = 0xB0;
    static constexpr uint8_t DEFAULT_COUNTER_BDR_REG2   = 0xC0;
    static constexpr uint8_t DEFAULT_INT1_CTRL          = 0xD0;
    static constexpr uint8_t DEFAULT_INT2_CTRL          = 0xE0;
    static constexpr uint8_t DEFAULT_WHO_AM_I           = 0x6B;
    static constexpr uint8_t DEFAULT_CTRL1_XL           = 0x00;
    static constexpr uint8_t DEFAULT_CTRL2_G            = 0x00;
    static constexpr uint8_t DEFAULT_CTRL3_C            = 0x04;
    static constexpr uint8_t DEFAULT_CTRL4_C            = 0x00;
    static constexpr uint8_t DEFAULT_CTRL5_C            = 0x00;
    static constexpr uint8_t DEFAULT_CTRL6_C            = 0x00;
    static constexpr uint8_t DEFAULT_CTRL7_G            = 0x00;
    static constexpr uint8_t DEFAULT_CTRL8_XL           = 0x00;
    static constexpr uint8_t DEFAULT_CTRL9_XL           = 0xE0;
    static constexpr uint8_t DEFAULT_CTRL10_C           = 0x00;
    // Output registers have no default values
    static constexpr uint8_t DEFAULT_TAP_CFG0           = 0x00;
    static constexpr uint8_t DEFAULT_TAP_CFG1           = 0x00;
    static constexpr uint8_t DEFAULT_TAP_CFG2           = 0x00;
    static constexpr uint8_t DEFAULT_TAP_THS_6D         = 0x00;
    static constexpr uint8_t DEFAULT_INT_DUR2           = 0x00;
    static constexpr uint8_t DEFAULT_WAKE_UP_THS        = 0x00;
    static constexpr uint8_t DEFAULT_WAKE_UP_DUR        = 0x00;
    static constexpr uint8_t DEFAULT_FREE_FALL          = 0x00;
    static constexpr uint8_t DEFAULT_MD1_CFG            = 0x00;
    static constexpr uint8_t DEFAULT_MD2_CFG            = 0x00;
    static constexpr uint8_t DEFAULT_INT_OIS            = 0x00;
    static constexpr uint8_t DEFAULT_CTRL1_OIS          = 0x00;
    static constexpr uint8_t DEFAULT_CTRL2_OIS          = 0x00;
    static constexpr uint8_t DEFAULT_CTRL3_OIS          = 0x00;
    static constexpr uint8_t DEFAULT_X_OFS_USR          = 0x00;
    static constexpr uint8_t DEFAULT_Y_OFS_USR          = 0x00;
    static constexpr uint8_t DEFAULT_Z_OFS_USR          = 0x00;
};

class ISM330DHCX{
  private:
    uint8_t address = 0x6a;

    uint8_t sda_pin = 2;
    uint8_t scl_pin = 3;
    i2c_inst_t *i2c_default_ada = i2c1; // or i2c1 if you prefer
    
    uint16_t accelerometer_scale;
    float gyro_sensity; 
    void read_data(uint8_t address, uint8_t *buffer);
    void set_accelerometer_scale(AccelScale input_scale);
    void set_gyrosocpe_sensitivity(GyroScale input_sensitivity);
  public:
    ISM330DHCX(uint8_t sda = 2, uint8_t scl = 3)
      : sda_pin(sda), scl_pin(scl){}
    int begin();
    int init();

    int configure_accelerometer(AccelODR aodr=AccelODR::ODR_833Hz, AccelScale ascale = AccelScale::SCALE_2G, AccelLPF alpf = AccelLPF::LPF_DISABLED);
    int configure_gyroscope(GyroODR godr = GyroODR::ODR_833Hz, GyroScale gscale = GyroScale::SCALE_250DPS, GyroFS125 gf125 = GyroFS125::FS_125_ENABLED, GyroFS4000 gf400 = GyroFS4000::FS_4000_DISABLED);

    Vector get_accelerometer();
    Vector get_gyroscope();

    int set_threshold();

    float read_temp();

    int accelerometer_config();
};
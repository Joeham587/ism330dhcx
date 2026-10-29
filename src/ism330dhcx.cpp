#include "ism330dhcx.hpp"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <cstdint>
#include <cstdio>

int ISM330DHCX::begin(){
  i2c_init(i2c_default_ada, 400 * 1000);
  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);
  gpio_pull_up(sda_pin);
  gpio_pull_up(scl_pin);

  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
  return 0;
}

int ISM330DHCX::init(){
  uint8_t reg_addr = ISM330DHCX_Registers::WHO_AM_I; // Register address to read
  uint8_t buff[1];
  i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, &reg_addr, 1, true );
  i2c_read_blocking(i2c_default_ada, ISM330DHCX::address, buff, 1, false);

  if(buff[0] != 0x6b){
    return 1;
  }
  printf("Buffer %d\n", buff[0]);

  // CTRL3_C is shared by accel and gyro, so set it before either: BDU (bit 6)
  // stops the output registers updating mid-read, so a read never mixes the
  // low byte of one sample with the high byte of the next. IF_INC (bit 2,
  // on at reset) keeps the register address auto-incrementing for the
  // 6-byte reads.
  constexpr uint8_t kCtrl3Bdu = 1 << 6;
  constexpr uint8_t kCtrl3IfInc = 1 << 2;
  if (!write_register(ISM330DHCX_Registers::CTRL3_C, kCtrl3Bdu | kCtrl3IfInc)) {
    return 1;
  }

  ISM330DHCX::configure_accelerometer();
  if (ISM330DHCX::configure_gyroscope() != 0) return 1;

  return 0;
}

int ISM330DHCX::accelerometer_config(){
  uint8_t data = 0;
  return 0;
}

float ISM330DHCX::read_temp(){
  return 0.0;
}

int ISM330DHCX::configure_accelerometer(AccelODR aodr, AccelScale ascale, AccelLPF alpf){
  uint8_t data = 0;
  uint8_t ctrl1_xl_addr = ISM330DHCX_Registers::CTRL1_XL;
  data |= (static_cast<uint8_t>(aodr) << 4);
  data |= (static_cast<uint8_t>(ascale) << 2);
  data |= static_cast<uint8_t>(alpf);

  uint8_t data_write[2] = {ctrl1_xl_addr, data};

  i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, data_write, 2, false);

  ISM330DHCX::set_accelerometer_scale(ascale);
  return 0;
}

int ISM330DHCX::set_threshold(){
  return 0;
}

void ISM330DHCX::set_accelerometer_scale(AccelScale input_scale){
  switch (input_scale) {
   case AccelScale::SCALE_2G:  // 0b00
        accelerometer_scale = 16384;
        break;
    case AccelScale::SCALE_16G: // 0b01
        accelerometer_scale = 2048;
        break;
    case AccelScale::SCALE_4G:  // 0b10
        accelerometer_scale = 8192;
        break;
    case AccelScale::SCALE_8G:  // 0b11
         accelerometer_scale = 4096;
        break;
    default:
        // Handle unexpected values
        break;
  
  }
}

int ISM330DHCX::configure_gyroscope(GyroODR godr, GyroScale gscale, GyroFS125 gf125, GyroFS4000 gf400, GyroLPF1 glpf1){
  uint8_t data = 0;
  uint8_t ctrl2_g_addr = ISM330DHCX_Registers::CTRL2_G;

  data |= (static_cast<uint8_t>(godr) << 4);
  data |= (static_cast<uint8_t>(gscale) << 2);
  data |= (static_cast<uint8_t>(gf125) << 1);
  data |= (static_cast<uint8_t>(gf400));

  if (!write_register(ctrl2_g_addr, data)) return 1;

  ISM330DHCX::set_gyrosocpe_sensitivity(gscale);

  // CTRL4_C and CTRL6_C also hold non-gyro bits; this driver leaves those at
  // their reset value (0), so writing the whole register is safe.
  constexpr uint8_t kCtrl4Lpf1SelG = 1 << 1;
  const bool lpf1_on = glpf1 != GyroLPF1::LPF1_DISABLED;
  if (!write_register(ISM330DHCX_Registers::CTRL4_C,
                      lpf1_on ? kCtrl4Lpf1SelG : 0)) {
    return 1;
  }
  if (lpf1_on && !write_register(ISM330DHCX_Registers::CTRL6_C,
                                 static_cast<uint8_t>(glpf1))) {
    return 1;
  }

  // High-pass filter off (HP_EN_G = 0): it removed slow, steady rotation
  // from the rate, which the angle integration needs.
  if (!write_register(ISM330DHCX_Registers::CTRL7_G, 0x00)) return 1;

  return 0;
}

bool ISM330DHCX::write_register(uint8_t reg, uint8_t value){
  uint8_t data_write[2] = {reg, value};
  return i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, data_write,
                            2, false) == 2;
}

void ISM330DHCX::set_gyrosocpe_sensitivity(GyroScale input_sensitivity){
  switch (input_sensitivity) {
        case GyroScale::SCALE_250DPS:   // 0b00
            gyro_sensity = 8.75f;    // mdps/LSB
            break;
        case GyroScale::SCALE_500DPS:   // 0b01
            gyro_sensity = 17.50f;   // mdps/LSB
            break;
        case GyroScale::SCALE_1000DPS:  // 0b10
            gyro_sensity = 35.0f;    // mdps/LSB
            break;
        case GyroScale::SCALE_2000DPS:  // 0b11
            gyro_sensity = 70.0f;    // mdps/LSB
            break;
        default:
            gyro_sensity = 8.75;    // Default to ±250 dps sensitivity
    }
}

bool ISM330DHCX::get_accelerometer(Vector &out){
  uint8_t OUTX_L_A_addr = ISM330DHCX_Registers::OUTX_L_A;
  uint8_t buff_acc[6];

  int wrote = i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, &OUTX_L_A_addr, 1, true);
  if (wrote != 1) return false;

  int read = i2c_read_blocking(i2c_default_ada, ISM330DHCX::address, buff_acc, 6, false);
  if (read != 6) return false;

  int16_t acc_x = (int16_t)((buff_acc[1] << 8) | buff_acc[0]);
  int16_t acc_y = (int16_t)((buff_acc[3] << 8) | buff_acc[2]);
  int16_t acc_z = (int16_t)((buff_acc[5] << 8) | buff_acc[4]);

  out.XAxis = (float)acc_x / (accelerometer_scale);
  out.YAxis = (float)acc_y / (accelerometer_scale);
  out.ZAxis = (float)acc_z / (accelerometer_scale);

  return true;
}

bool ISM330DHCX::get_gyroscope(Vector &out){
  uint8_t OUTX_L_G_addr = ISM330DHCX_Registers::OUTX_L_G;
  uint8_t buff_gyro[6];

  int wrote = i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, &OUTX_L_G_addr, 1, true);
  if (wrote != 1) return false;

  int read = i2c_read_blocking(i2c_default_ada, ISM330DHCX::address, buff_gyro, 6, false);
  if (read != 6) return false;

  int16_t gyro_x = (int16_t)((buff_gyro[1] << 8) | buff_gyro[0]);
  int16_t gyro_y = (int16_t)((buff_gyro[3] << 8) | buff_gyro[2]);
  int16_t gyro_z = (int16_t)((buff_gyro[5] << 8) | buff_gyro[4]);

  // gyro_sensity is mdps/LSB -- multiply, then /1000 for dps, don't divide.
  out.XAxis = (float)gyro_x * gyro_sensity / 1000.0f;
  out.YAxis = (float)gyro_y * gyro_sensity / 1000.0f;
  out.ZAxis = (float)gyro_z * gyro_sensity / 1000.0f;

  return true;
}


#ifndef MAIN
//int main(){
//  stdio_init_all();
//
//  sleep_ms(1000);
//  printf("Begin\n");
//  ISM330DHCX acc = ISM330DHCX(2, 3);
//  acc.begin();
//  acc.init();
//  Vector read_acc;
//  Vector read_gyro;
//  while (true) {
//    read_acc = acc.get_accelerometer();
//    read_gyro = acc.get_gyroscope();
//
//    printf("a_x: %.3f, a_y: %.3f, a_z: %.3f\n", read_acc.XAxis, read_acc.YAxis, read_acc.ZAxis);
//    printf("g_x: %.3f, g_y: %.3f, g_z: %.3f\n", read_gyro.XAxis, read_gyro.YAxis, read_gyro.ZAxis);
//    sleep_ms(500);
//  }
//
//  return 0;
//}
#endif
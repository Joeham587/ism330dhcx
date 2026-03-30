#include "ism330dhcx.hpp"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <cstdint>
#include <cstdio>

int ISM330DHCX::begin(){
  i2c_init(i2c_default_ada, 100 * 1000);
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

  ISM330DHCX::configure_accelerometer();
  ISM330DHCX::configure_gyroscope();

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

int ISM330DHCX::configure_gyroscope(GyroODR godr, GyroScale gscale, GyroFS125 gf125, GyroFS4000 gf400){
  uint8_t data = 0;
  uint8_t ctrl2_g_addr = ISM330DHCX_Registers::CTRL2_G;

  data |= (static_cast<uint8_t>(godr) << 4);
  data |= (static_cast<uint8_t>(gscale) << 2);
  data |= (static_cast<uint8_t>(gf125) << 1);
  data |= (static_cast<uint8_t>(gf400));

  uint8_t data_write[2] = {ctrl2_g_addr, data};

  i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, data_write, 2, false);

  ISM330DHCX::set_gyrosocpe_sensitivity(gscale);

  uint8_t high_pass_filter = 0b01000000;
  uint8_t data_high_pass_filter[2] = {ISM330DHCX_Registers::CTRL7_G, high_pass_filter };

  i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, data_high_pass_filter, 2, false);

  return 0;
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

Vector ISM330DHCX::get_accelerometer(){
  uint8_t OUTX_L_A_addr = ISM330DHCX_Registers::OUTX_L_A;
  uint8_t buff_acc[6];  
  Vector accel;
  i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, &OUTX_L_A_addr, 1, true);
  i2c_read_blocking(i2c_default_ada, ISM330DHCX::address, buff_acc, 6, false);

  int16_t acc_x = (int16_t)((buff_acc[1] << 8) | buff_acc[0]);
  int16_t acc_y = (int16_t)((buff_acc[3] << 8) | buff_acc[2]);
  int16_t acc_z = (int16_t)((buff_acc[5] << 8) | buff_acc[4]);

  accel.XAxis = (float)acc_x / (accelerometer_scale);
  accel.YAxis = (float)acc_y / (accelerometer_scale);
  accel.ZAxis = (float)acc_z / (accelerometer_scale);
  
  return accel;
}

Vector ISM330DHCX::get_gyroscope(){
  Vector gyro;
  uint8_t OUTX_L_G_addr = ISM330DHCX_Registers::OUTX_L_G;
  uint8_t buff_gyro[6];

  i2c_write_blocking(i2c_default_ada, ISM330DHCX::address, &OUTX_L_G_addr, 1, true);
  i2c_read_blocking(i2c_default_ada, ISM330DHCX::address, buff_gyro, 6, false);

  int16_t gyro_x = (int16_t)((buff_gyro[1] << 8) | buff_gyro[0]);
  int16_t gyro_y = (int16_t)((buff_gyro[3] << 8) | buff_gyro[2]);
  int16_t gyro_z = (int16_t)((buff_gyro[5] << 8) | buff_gyro[4]);

  gyro.XAxis = (float)gyro_x / (gyro_sensity);
  gyro.YAxis = (float)gyro_y / (gyro_sensity);
  gyro.ZAxis = (float)gyro_z / (gyro_sensity);

  return gyro;
}

int main(){
  stdio_init_all();

  sleep_ms(1000);
  printf("Begin\n");
  ISM330DHCX acc = ISM330DHCX(2, 3);
  acc.begin();
  acc.init();
  Vector read_acc;
  Vector read_gyro;
  while (true) {
    read_acc = acc.get_accelerometer();
    read_gyro = acc.get_gyroscope();

    printf("a_x: %.3f, a_y: %.3f, a_z: %.3f\n", read_acc.XAxis, read_acc.YAxis, read_acc.ZAxis);
    printf("g_x: %.3f, g_y: %.3f, g_z: %.3f\n", read_gyro.XAxis, read_gyro.YAxis, read_gyro.ZAxis);
    sleep_ms(500);
  }

  return 0;
}

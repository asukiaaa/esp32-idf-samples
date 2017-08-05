#include "MPU9250_asukiaaa.h"

void delay(unsigned long milli_seconds) {
  vTaskDelay(milli_seconds / portTICK_RATE_MS);
}

static esp_err_t i2c_master_write_slave(uint8_t address, uint8_t* data_wr, size_t size) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( address << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t i2c_master_read_slave(uint8_t address, uint8_t register_address, uint8_t* data_rd, size_t size) {
  if (size == 0) {
    return ESP_OK;
  }
  esp_err_t ret = i2c_master_write_slave(address, &register_address, 1);
  if (ret == ESP_FAIL) {
    return ret;
  }
  delay(30);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( address << 1 ) | READ_BIT, ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static void i2c_master_init(mpu9250_t *data) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = data->sdaPin;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = data->sclPin;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_MASTER_RX_BUF_DISABLE,
                     I2C_MASTER_TX_BUF_DISABLE, 0);
}

void mpu9250_mag_set_mode(uint8_t mode) {
  static uint8_t buf[] = {AK8963_RA_CNTL1, 0x00};
  buf[1] = mode;
  printf("set mode\n");
  i2c_master_write_slave(MPU9250_MAG_ADDRESS, buf, 2);
  delay(10);
}

void mpu9250_mag_read_adjust_values(mpu9250_t *data) {
  mpu9250_mag_set_mode(AK8963_MODE_POWERDOWN);
  mpu9250_mag_set_mode(AK8963_MODE_FUSEROM);

  uint8_t buff[3];
  i2c_master_read_slave(MPU9250_MAG_ADDRESS, AK8963_RA_ASAX, buff, 3);
  printf("adjust values: %03d, %03d, %03d\n", buff[0], buff[1], buff[2]);
  data->magXAdjust = buff[0];
  data->magYAdjust = buff[1];
  data->magZAdjust = buff[2];
}

void mpu9250_mag_begin(mpu9250_t *data) {
  printf("begin magnetometor\n");
  i2c_master_init(data);
  delay(10);

  // turn on magnetometor
  static uint8_t buf[] = {0x37, 0x02};
  i2c_master_write_slave(data->address, buf, 2);
  delay(20);

  mpu9250_mag_read_adjust_values(data);
  mpu9250_mag_set_mode(AK8963_MODE_POWERDOWN);
  mpu9250_mag_set_mode(AK8963_MODE_CONTINUOUS_8HZ);
}

void mpu9250_mag_update(mpu9250_t *data) {
  i2c_master_read_slave(MPU9250_MAG_ADDRESS, AK8963_RA_HXL, data->magBuf, 7);
}

int16_t mpu9250_mag_get(mpu9250_t *data, uint8_t high_index, uint8_t low_index) {
  return (((int16_t)data->magBuf[high_index]) << 8) | data->magBuf[low_index];
}

int16_t mag_adjust_value(uint16_t value, uint8_t adjust) {
  return (value * ((((adjust - 128) * 0.5) / 128) + 1));
}

int16_t mpu9250_mag_x(mpu9250_t *data) {
  return mag_adjust_value(mpu9250_mag_get(data, 1, 0), data->magXAdjust) + data->magXOffset;;
}

int16_t mpu9250_mag_y(mpu9250_t *data) {
  return mag_adjust_value(mpu9250_mag_get(data, 3, 2), data->magYAdjust) + data->magYOffset;;
}

int16_t mpu9250_mag_z(mpu9250_t *data) {
  return mag_adjust_value(mpu9250_mag_get(data, 5, 4), data->magZAdjust) + data->magZOffset;;
}

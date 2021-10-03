#include <stdio.h>
#include "freertos\FreeRTOS.h"
#include "freertos\task.h"
#include "driver\i2c.h"
#include "esp_types.h"

#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define SDA_GPIO 18
#define SCL_GPIO 19
#define INA219_ADD 0x40
#define reg_addr 0x04  //00 = config | 01 = vShunt | 02 = vBus | 03 = Power | 04 = Current 
#define cal_addr 0x05  //Calibration register address


void app_main(void)
{
  printf("i2c INA219\n");

  i2c_config_t i2c_config = 
  {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA_GPIO,
    .scl_io_num = SCL_GPIO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
  };

  i2c_param_config(I2C_NUM_0, &i2c_config);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

  //Write Calibration Value for 1A max current = 0x34 0x6D
  i2c_cmd_handle_t cmd_calibrate = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd_calibrate));
  //Write Slave Address Byte
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_calibrate, (INA219_ADD << 1) | I2C_MASTER_WRITE, true));
  //Write Register Pointer Byte
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_calibrate, cal_addr, true));
  //Write Calibration MSB
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_calibrate, (uint8_t)0x34, true));
  //Write Calibration LSB
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_calibrate, (uint8_t)0x6D, true));

  ESP_ERROR_CHECK(i2c_master_stop(cmd_calibrate));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd_calibrate, 1000 / portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd_calibrate);
  vTaskDelay(100/portTICK_PERIOD_MS);

  //Read Calibration Register Value to confirm value written
  uint8_t cal_raw[2];
  i2c_cmd_handle_t cmd_read_cal = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd_read_cal));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_read_cal, (INA219_ADD << 1) | I2C_MASTER_READ, true)); 
  ESP_ERROR_CHECK(i2c_master_read(cmd_read_cal, (uint8_t *) &cal_raw, 2, ACK_VAL));
  ESP_ERROR_CHECK(i2c_master_stop(cmd_read_cal));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd_read_cal, 1000 / portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd_read_cal);

  int16_t cal_data = cal_raw[0] << 8 | cal_raw[1];
  printf("Calibration Register Value: %i\n", cal_data);

  //Calibration End


  //Move Pointer to desired register
  uint16_t d = 0;
  uint8_t temp;
  temp = (uint8_t)d;
  d >>= 8;

	uint8_t cmddata[3];
	cmddata[0]=reg_addr; // sends register address to read from
	cmddata[1]=(uint8_t)d; // write data hibyte
	cmddata[2]=temp; // write data lobyte;

  i2c_cmd_handle_t cmd_write = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd_write));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_write, (INA219_ADD << 1) | I2C_MASTER_WRITE, true));
  ESP_ERROR_CHECK(i2c_master_write(cmd_write, &cmddata[0], 3, true));
  ESP_ERROR_CHECK(i2c_master_stop(cmd_write));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd_write, 1000 / portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd_write);
  vTaskDelay(100/portTICK_PERIOD_MS);
  //End

  //Read the previously set register
  uint8_t raw[2];
  i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd_read));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd_read, (INA219_ADD << 1) | I2C_MASTER_READ, true)); 
  ESP_ERROR_CHECK(i2c_master_read(cmd_read, (uint8_t *) &raw, 2, ACK_VAL));
  ESP_ERROR_CHECK(i2c_master_stop(cmd_read));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd_read, 1000 / portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd_read);
  //End

  //Print the two bytes read from the register
  for(int i=0; i < (sizeof(raw)/sizeof(raw[0])); i++)
    {
      printf("%x \n", raw[i]);
    }
    printf("\n");
  
  int16_t data = raw[0] << 8 | raw[1];

  if(reg_addr == 0x01){
    //vShunt = raw register value multiplied by 10uV
    double vShunt = data * .00001;
    printf("Shunt Voltage: %fV \n", vShunt);
  }else if(reg_addr == 0x02){
    //vBus = raw register value shifted right by 3 bits and then multiplied by 4mV
    double vBus = (data >> 3) * 0.004;
    printf("Bus Voltage: %fV \n", vBus);
  }else if(reg_addr == 0x03){
    //Power = raw register value x (20 x Current_LSB)
    double Power = data * (20 * 0.00003052);
    printf("Power: %fW \n", Power);
  }else if(reg_addr == 0x04){
    //Current = raw register data x Current LSB
    double Current = data * 0.00003052;
    printf("Current: %fA \n", Current);
  }
 
  
}

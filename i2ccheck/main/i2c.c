
#include "i2c.h"
#include <driver/i2c.h>
#include <i2cdev.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static gpio_num_t i2c_gpio_sda = CONFIG_EXAMPLE_I2C_MASTER_SDA;
static gpio_num_t i2c_gpio_scl = CONFIG_EXAMPLE_I2C_MASTER_SCL;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

SemaphoreHandle_t i2c_mutex;

const char *TAG = "i2c";

static esp_err_t i2c_master_driver_initialize(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = i2c_gpio_sda,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = i2c_gpio_scl,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = i2c_frequency,
      // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_*
      // flags to choose i2c source clock here. */
  };
  return i2c_param_config(i2c_port, &conf);
}

void perform_i2c_scan_bus(esp_err_t *scan, size_t size) {
  if (size < 128) {
    ESP_LOGE(TAG, "i2c scan buffer size < 128");
    return;
  }
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);

  for (int i = 0; i < 128; i++) {
    uint8_t address = i;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    scan[i] = ret;
  }
  xSemaphoreGive(i2c_mutex);
}

void i2c_scan_bus() {
  uint8_t address;
  esp_err_t scan[128];
  perform_i2c_scan_bus(scan, sizeof(scan));

  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
  for (int i = 0; i < 128; i += 16) {
    printf("%02x: ", i);
    for (int j = 0; j < 16; j++) {
      fflush(stdout);
      address = i + j;
      esp_err_t ret = scan[address];
      if (ret == ESP_OK) {
        printf("%02x ", address);
      } else if (ret == ESP_ERR_TIMEOUT) {
        printf("UU ");
      } else {
        printf("-- ");
      }
    }
    printf("\r\n");
  }
}

void i2c_scan_task(void *pvParameters) {
  ESP_LOGD(TAG, "%s", __FUNCTION__);

  while (true) {
    ESP_LOGD(TAG, "i2c scan loop");
    i2c_scan_bus();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void init_i2c(void) {
  ESP_LOGI(TAG, "init_i2c");

  ESP_ERROR_CHECK(i2cdev_init());

  ESP_LOGI(TAG, "I2C SDA = %d", CONFIG_EXAMPLE_I2C_MASTER_SDA);
  ESP_LOGI(TAG, "I2C SCL = %d", CONFIG_EXAMPLE_I2C_MASTER_SCL);

#if 0
  ESP_LOGD(TAG, "Initializing i2c...");
  i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
                     I2C_MASTER_TX_BUF_DISABLE, 0);
  i2c_master_driver_initialize();

#endif

  i2c_mutex = xSemaphoreCreateMutex();
}
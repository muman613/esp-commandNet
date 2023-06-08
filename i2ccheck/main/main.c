#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include "i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp9808.h>
#include <stdio.h>
#include <string.h>

#define ENABLE_TEMP_TASK 1
#define ENABLE_SCAN_TASK 1

static const char *TAG = "i2ccheck";

// #define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
// #define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
// #define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
// #define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from
// slave*/ #define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from
// slave */ #define ACK_VAL 0x0       /*!< I2C ack value */ #define NACK_VAL 0x1
// /*!< I2C nack value */

// static gpio_num_t i2c_gpio_sda = 18;
// static gpio_num_t i2c_gpio_scl = 19;
// static uint32_t i2c_frequency = 100000;
// static i2c_port_t i2c_port = I2C_NUM_0;
// static SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t i2c_mutex;

/**
 * @brief Convert temperature from Celsius to Farenheit.
 *
 * @param t
 * @return float
 */
inline float temp_c_to_f(float t) { return (t * 9.0 / 5.0) + 32.0; }

void temp_task(void *pvParameters) {
  float temperature;
  esp_err_t res;

  //   xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  i2c_dev_t dev;
  memset(&dev, 0, sizeof(i2c_dev_t));

  //   ESP_LOGD(TAG, "Initializing i2c...");

  ESP_ERROR_CHECK(mcp9808_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, 0,
                                    CONFIG_EXAMPLE_I2C_MASTER_SDA,
                                    CONFIG_EXAMPLE_I2C_MASTER_SCL));
  ESP_ERROR_CHECK(mcp9808_init(&dev));

  while (1) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    // Get the values and do something with them.
    if ((res = mcp9808_get_temperature(&dev, &temperature, NULL, NULL, NULL)) ==
        ESP_OK)
      printf("Temperature: %.2f °F\n", temp_c_to_f(temperature));
    else
      printf("Could not get results: %d (%s)", res, esp_err_to_name(res));
    xSemaphoreGive(i2c_mutex);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void) {
  ESP_LOGD(TAG, "app_main entered!");
  BaseType_t xReturned;
  TaskHandle_t xTaskHandle1 = NULL, xTaskHandle2 = NULL;

  // #ifndef NDEBUG
  //   ESP_LOGD(TAG, "I2C SDA = %d", CONFIG_EXAMPLE_I2C_MASTER_SDA);
  //   ESP_LOGD(TAG, "I2C SCL = %d", CONFIG_EXAMPLE_I2C_MASTER_SCL);
  // #endif

  init_i2c();

#ifdef ENABLE_TEMP_TASK
  xReturned = xTaskCreate(temp_task, "temp_task", configMINIMAL_STACK_SIZE * 8,
                          (void *)NULL, 0, &xTaskHandle1);
  if (xReturned != pdPASS) {
    ESP_LOGW(TAG, "Unable to start i2c task");
  }
#endif // ENABLE_TEMP_TASK
#ifdef ENABLE_SCAN_TASK
  xReturned = xTaskCreate(i2c_scan_task, "i2c_scan_task", 4096, (void *)NULL, 5,
                          &xTaskHandle2);
  if (xReturned != pdPASS) {
    ESP_LOGW(TAG, "Unable to start scan task");
  }
#endif // ENABLE_SCAN_TASK

  //   i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
  //                      I2C_MASTER_TX_BUF_DISABLE, 0);
  //   i2c_master_driver_initialize();

  //   while (true) {
  //     uint8_t address;
  //     printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
  //     for (int i = 0; i < 128; i += 16) {
  //       printf("%02x: ", i);
  //       for (int j = 0; j < 16; j++) {
  //         fflush(stdout);
  //         address = i + j;
  //         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //         i2c_master_start(cmd);
  //         i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT,
  //         ACK_CHECK_EN); i2c_master_stop(cmd); esp_err_t ret =
  //             i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_PERIOD_MS);
  //         i2c_cmd_link_delete(cmd);
  //         if (ret == ESP_OK) {
  //           printf("%02x ", address);
  //         } else if (ret == ESP_ERR_TIMEOUT) {
  //           printf("UU ");
  //         } else {
  //           printf("-- ");
  //         }
  //       }
  //       printf("\r\n");
  //     }
  //     vTaskDelay(2000 / portTICK_PERIOD_MS);
  //   }
}
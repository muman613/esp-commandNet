#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <i2cdev.h>
#include <mcp9808.h>
#include <stdio.h>
#include <string.h>

#include <semaphore.h>
#include <ssd1306.h>

const char *TAG = "i2cdev-test";

#define I2C_FREQ_HZ 400000 // 400 kHz

#define I2C_SDA 18
#define I2C_SCL 19

#define MCP9808_ADDRESS 0x18 ///< i2c address

#define MCP9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 0x03   ///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolutin

SemaphoreHandle_t i2c_mutex;
QueueHandle_t temp_queue = NULL;

esp_err_t init_i2c_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port,
                        gpio_num_t sda_gpio, gpio_num_t scl_gpio) {

  // I2C Address verification is not needed because the device may have a custom
  // factory address

  dev->port = port;
  dev->addr = addr;
  dev->cfg.sda_io_num = sda_gpio;
  dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
  dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

  return i2c_dev_create_mutex(dev);
}

i2c_dev_t i2c_mcp9808;

uint16_t i2c_read_reg16(i2c_dev_t *dev, uint8_t reg) {
  uint16_t result = 0;

  i2c_dev_read_reg(dev, reg, &result, sizeof(result));

  result = ((result & 0xff) << 8) | ((result & 0xff00) >> 8);

  return result;
}

bool verify_mcp9808_exists() {
  if (i2c_dev_probe(&i2c_mcp9808, 0) != ESP_OK) {
    ESP_LOGW(TAG, "No device @ i2c address %02x.", MCP9808_ADDRESS);
    return false;
  }

  if (i2c_read_reg16(&i2c_mcp9808, MCP9808_REG_MANUF_ID) != 0x0054) {
    ESP_LOGW(TAG, "Device @ i2c address %02x is not MCP9808.", MCP9808_ADDRESS);
    return false;
  }

  ESP_LOGI(TAG, "-- found mcp9808 @ 0x%02x", MCP9808_ADDRESS);
  return true;
}

float get_mcp9808_ambient_temp_c() {
  float result = 0.0;
  uint16_t t = i2c_read_reg16(&i2c_mcp9808, MCP9808_REG_AMBIENT_TEMP);

  if (t != 0xFFFF) {
    result = t & 0x0FFF;
    result /= 16.0;
    if (t & 0x1000)
      result -= 256;
  }

  return result;
}

float get_mcp9808_ambient_temp_f() {
  float temp = get_mcp9808_ambient_temp_c();
  return (temp * 9.0 / 5.0 + 32);
}

void temp_task(void *parm) {
  ESP_LOGD(TAG, "%s", __FUNCTION__);

  while (true) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    float temp_f = get_mcp9808_ambient_temp_f();
    xSemaphoreGive(i2c_mutex);

    printf("Temperature is %.2f° F\n", temp_f);
    xQueueSend(temp_queue, &temp_f, portMAX_DELAY);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void display_task(void *parm) {
  ESP_LOGD(TAG, "%s", __FUNCTION__);
  SSD1306_t ssd1306_dev = {
      ._address = 0x3c,
      ._flip = false,
  };

  //   i2c_master_init(&ssd1306_dev, I2C_SDA, I2C_SCL, -1);
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  ssd1306_init(&ssd1306_dev, 128, 64);
  ssd1306_clear_screen(&ssd1306_dev, false);
  xSemaphoreGive(i2c_mutex);

  char buffer[128];
//   int value = 0;
  while (true) {

    float temp = 0.0;
    if (xQueueReceive(temp_queue, &temp, portMAX_DELAY) == true) {
      snprintf(buffer, 128, " Temp : %.2f", temp);
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      ssd1306_display_text(&ssd1306_dev, 4, buffer, strlen(buffer), false);
      xSemaphoreGive(i2c_mutex);
    }
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "app_main()");

  ESP_ERROR_CHECK(i2cdev_init());

  i2c_mutex = xSemaphoreCreateMutex();
  temp_queue = xQueueCreate(4, sizeof(float));

  if (temp_queue == NULL) {
    ESP_LOGW(TAG, "Unable to create queue");
  }

  if (init_i2c_desc(&i2c_mcp9808, 0x18, 0, I2C_SDA, I2C_SCL) != ESP_OK) {
    ESP_LOGW(TAG, "Unable to init i2c device");
  }
  BaseType_t res = 0;

  if (verify_mcp9808_exists()) {
    TaskHandle_t temp_task_handle = NULL;

    res = xTaskCreate(temp_task, "Temp Task", configMINIMAL_STACK_SIZE * 8,
                      NULL, 0, &temp_task_handle);
    if (res != pdPASS) {
      ESP_LOGE(TAG, "Unable to create temp task");
    } else {
      ESP_LOGI(TAG, "mcp9808 discovered, starting task");
    }
  }

  TaskHandle_t disp_task_handle = NULL;

  //   SSD1306_t ssd1306_dev = {
  //       ._address = 0x3c,
  //       ._flip = false,
  //   };

  //   //   i2c_master_init(&ssd1306_dev, I2C_SDA, I2C_SCL, -1);
  //   ssd1306_init(&ssd1306_dev, 128, 64);
  //   ssd1306_clear_screen(&ssd1306_dev, false);

  res = xTaskCreate(display_task, "Display Task", configMINIMAL_STACK_SIZE * 8,
                    NULL, 0, &disp_task_handle);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Unable to create display task");
  } else {
    ESP_LOGI(TAG, "Display task started");
  }
}
